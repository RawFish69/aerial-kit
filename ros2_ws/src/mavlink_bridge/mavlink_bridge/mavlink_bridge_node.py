import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from drone_msgs.msg import Telemetry

from pymavlink import mavutil

from mavlink_bridge.frame_transforms import (
    ned_body_to_enu_quaternion,
    ned_to_enu_position,
    ned_to_enu_velocity,
)

# Ignore position, acceleration, and yaw-angle fields; keep velocity + yaw_rate.
_SETPOINT_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge_node')

        self.declare_parameter('connection_url', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('flight_stack', 'px4')
        self.declare_parameter('system_id', 245)
        self.declare_parameter('component_id', 191)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)

        self.declare_parameter('backend_cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('backend_enable_topic', '/uav/backend/enable')
        self.declare_parameter('backend_odom_topic', '/uav/backend/odom')
        self.declare_parameter('telemetry_raw_topic', '/uav/backend/telemetry_raw')

        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('heartbeat_timeout_sec', 3.0)
        self.declare_parameter('disarm_max_speed_mps', 0.3)
        self.declare_parameter('disarm_max_altitude_m', 0.3)
        self.declare_parameter('yaw_rate_sign', -1.0)

        self.connection_url = self.get_parameter('connection_url').value
        self.baud = int(self.get_parameter('baud').value)
        self.flight_stack = self.get_parameter('flight_stack').value
        self.system_id = int(self.get_parameter('system_id').value)
        self.component_id = int(self.get_parameter('component_id').value)
        self.target_system = int(self.get_parameter('target_system').value)
        self.target_component = int(self.get_parameter('target_component').value)

        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.heartbeat_timeout_sec = float(self.get_parameter('heartbeat_timeout_sec').value)
        self.disarm_max_speed_mps = float(self.get_parameter('disarm_max_speed_mps').value)
        self.disarm_max_altitude_m = float(self.get_parameter('disarm_max_altitude_m').value)
        self.yaw_rate_sign = float(self.get_parameter('yaw_rate_sign').value)

        backend_cmd_topic = self.get_parameter('backend_cmd_topic').value
        backend_enable_topic = self.get_parameter('backend_enable_topic').value
        backend_odom_topic = self.get_parameter('backend_odom_topic').value
        telemetry_raw_topic = self.get_parameter('telemetry_raw_topic').value
        setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)

        self.pub_odom = self.create_publisher(Odometry, backend_odom_topic, 20)
        self.pub_telemetry_raw = self.create_publisher(Telemetry, telemetry_raw_topic, 20)

        self.create_subscription(Twist, backend_cmd_topic, self._on_cmd_twist, 20)
        self.create_subscription(Bool, backend_enable_topic, self._on_enable, 10)

        self._last_cmd_twist = Twist()
        self._last_cmd_time = None
        self._enabled = False
        self._disarm_pending = False
        self._arm_ack_command_id = None

        self._link_alive = False
        self._last_heartbeat_time = None
        self._fc_armed = False
        self._autopilot_logged = False

        self._last_local_pos = None  # (x_ned, y_ned, z_ned, vx, vy, vz)
        self._last_attitude_q = None  # (w, x, y, z) NED body quaternion
        self._battery_percent = -1.0

        self.get_logger().info(
            f'Connecting to MAVLink at {self.connection_url} (baud={self.baud}, '
            f'flight_stack={self.flight_stack})')
        self.mav = mavutil.mavlink_connection(
            self.connection_url,
            baud=self.baud,
            source_system=self.system_id,
            source_component=self.component_id,
        )

        period = 1.0 / setpoint_rate_hz
        self.create_timer(period, self._tick)
        self.create_timer(1.0, self._send_heartbeat)

    # -- ROS callbacks -----------------------------------------------------

    def _on_cmd_twist(self, msg):
        self._last_cmd_twist = msg
        self._last_cmd_time = time.monotonic()

    def _on_enable(self, msg):
        want_enabled = bool(msg.data)
        if want_enabled and not self._enabled:
            self._send_arm(True)
        elif not want_enabled and self._enabled:
            self._disarm_pending = True
        self._enabled = want_enabled

    # -- outbound MAVLink ----------------------------------------------------

    def _tick(self):
        self._drain_incoming()

        now = time.monotonic()
        if self._last_heartbeat_time is not None:
            self._link_alive = (now - self._last_heartbeat_time) <= self.heartbeat_timeout_sec
        if not self._link_alive:
            return

        if self._disarm_pending:
            self._try_disarm_if_safe()

        if not self._enabled:
            return

        twist = self._last_cmd_twist
        stale = (
            self._last_cmd_time is None
            or (now - self._last_cmd_time) > self.command_timeout_sec
        )
        if stale:
            vx_mav, vy_mav, vz_mav, yaw_rate = 0.0, 0.0, 0.0, 0.0
        else:
            # Body convention (mission_executor_node): +Y = nose/forward, +X = right.
            # MAVLink body FRD: x = forward, y = right, z = down.
            vx_mav = twist.linear.y
            vy_mav = twist.linear.x
            vz_mav = -twist.linear.z
            yaw_rate = self.yaw_rate_sign * twist.angular.z

        self.mav.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            _SETPOINT_TYPE_MASK,
            0.0, 0.0, 0.0,
            vx_mav, vy_mav, vz_mav,
            0.0, 0.0, 0.0,
            0.0, yaw_rate,
        )

    def _send_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _send_arm(self, arm):
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        self.get_logger().info(f'Sent {"arm" if arm else "disarm"} command')

    def _try_disarm_if_safe(self):
        if not self._fc_armed:
            self._disarm_pending = False
            return

        if self._last_local_pos is None:
            self.get_logger().warning(
                'Disarm requested but no position telemetry yet; withholding disarm.',
                throttle_duration_sec=2.0)
            return

        _, _, z_ned, vx, vy, vz = self._last_local_pos
        altitude = -z_ned
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)

        if speed <= self.disarm_max_speed_mps and altitude <= self.disarm_max_altitude_m:
            self._send_arm(False)
            self._disarm_pending = False
        else:
            self.get_logger().warning(
                f'Disarm deferred: altitude={altitude:.2f}m speed={speed:.2f}m/s '
                f'still above threshold (alt<{self.disarm_max_altitude_m}, '
                f'speed<{self.disarm_max_speed_mps})',
                throttle_duration_sec=2.0)

    # -- inbound MAVLink -------------------------------------------------

    def _drain_incoming(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                return
            msg_type = msg.get_type()
            if msg_type == 'HEARTBEAT':
                self._on_heartbeat(msg)
            elif msg_type == 'LOCAL_POSITION_NED':
                self._on_local_position(msg)
            elif msg_type == 'ATTITUDE_QUATERNION':
                self._last_attitude_q = (msg.q1, msg.q2, msg.q3, msg.q4)
            elif msg_type == 'SYS_STATUS':
                if msg.battery_remaining >= 0:
                    self._battery_percent = float(msg.battery_remaining)
            elif msg_type == 'COMMAND_ACK':
                self._on_command_ack(msg)

    def _on_heartbeat(self, msg):
        if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            return
        self._last_heartbeat_time = time.monotonic()
        self._link_alive = True
        self._fc_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if not self._autopilot_logged:
            self.get_logger().info(
                f'MAVLink heartbeat received: autopilot={msg.autopilot} type={msg.type}')
            self._autopilot_logged = True

    def _on_local_position(self, msg):
        self._last_local_pos = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
        self._publish_odom()
        self._publish_telemetry()

    def _on_command_ack(self, msg):
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            entry = mavutil.mavlink.enums['MAV_RESULT'].get(msg.result)
            result_name = entry.name if entry is not None else str(msg.result)
            self.get_logger().info(f'Arm/disarm COMMAND_ACK: {result_name}')

    # -- publishing --------------------------------------------------------

    def _publish_odom(self):
        if self._last_local_pos is None:
            return
        x_ned, y_ned, z_ned, vx_ned, vy_ned, vz_ned = self._last_local_pos

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        ex, ey, ez = ned_to_enu_position(x_ned, y_ned, z_ned)
        odom.pose.pose.position.x = ex
        odom.pose.pose.position.y = ey
        odom.pose.pose.position.z = ez

        if self._last_attitude_q is not None:
            qw, qx_b, qy_b, qz_b = self._last_attitude_q
            qx, qy, qz, qw_out = ned_body_to_enu_quaternion(qw, qx_b, qy_b, qz_b)
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw_out
        else:
            odom.pose.pose.orientation.w = 1.0

        vex, vey, vez = ned_to_enu_velocity(vx_ned, vy_ned, vz_ned)
        odom.twist.twist.linear.x = vex
        odom.twist.twist.linear.y = vey
        odom.twist.twist.linear.z = vez

        self.pub_odom.publish(odom)

    def _publish_telemetry(self):
        if self._last_local_pos is None:
            return
        x_ned, y_ned, z_ned, vx_ned, vy_ned, vz_ned = self._last_local_pos

        telem = Telemetry()
        telem.header.stamp = self.get_clock().now().to_msg()
        telem.header.frame_id = 'map'

        ex, ey, ez = ned_to_enu_position(x_ned, y_ned, z_ned)
        telem.pose.position.x = ex
        telem.pose.position.y = ey
        telem.pose.position.z = ez
        if self._last_attitude_q is not None:
            qw, qx_b, qy_b, qz_b = self._last_attitude_q
            qx, qy, qz, qw_out = ned_body_to_enu_quaternion(qw, qx_b, qy_b, qz_b)
            telem.pose.orientation.x = qx
            telem.pose.orientation.y = qy
            telem.pose.orientation.z = qz
            telem.pose.orientation.w = qw_out
        else:
            telem.pose.orientation.w = 1.0

        vex, vey, vez = ned_to_enu_velocity(vx_ned, vy_ned, vz_ned)
        telem.twist.linear.x = vex
        telem.twist.linear.y = vey
        telem.twist.linear.z = vez

        telem.battery_percent = self._battery_percent
        telem.armed = self._fc_armed

        self.pub_telemetry_raw.publish(telem)


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
