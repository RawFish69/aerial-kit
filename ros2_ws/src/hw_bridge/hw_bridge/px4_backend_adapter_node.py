import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from pymavlink import mavutil
except Exception:  # pragma: no cover - dependency/runtime environment specific
    mavutil = None


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class Px4BackendAdapterNode(Node):
    """PX4 offboard adapter: backend cmd+enable <-> MAVLink setpoints and odometry."""

    def __init__(self) -> None:
        super().__init__("px4_backend_adapter_node")
        self.declare_parameter("mavlink_url", "udpin:0.0.0.0:14540")
        self.declare_parameter("backend_cmd_topic", "/uav/backend/cmd_twist")
        self.declare_parameter("backend_enable_topic", "/uav/backend/enable")
        self.declare_parameter("backend_odom_topic", "/uav/backend/odom")
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("heartbeat_rate_hz", 2.0)
        self.declare_parameter("recv_rate_hz", 50.0)

        g = self.get_parameter
        self.mavlink_url = str(g("mavlink_url").value)
        self.setpoint_rate_hz = float(g("setpoint_rate_hz").value)
        self.heartbeat_rate_hz = float(g("heartbeat_rate_hz").value)
        self.recv_rate_hz = float(g("recv_rate_hz").value)

        self.enabled = False
        self.prev_enabled = False
        self.last_cmd = Twist()
        self.last_ned: Optional[tuple[float, float, float, float, float, float]] = None
        self.last_att_rpy: Optional[tuple[float, float, float]] = None

        self.pub_odom = self.create_publisher(Odometry, str(g("backend_odom_topic").value), 20)
        self.create_subscription(Twist, str(g("backend_cmd_topic").value), self._on_cmd, 20)
        self.create_subscription(Bool, str(g("backend_enable_topic").value), self._on_enable, 20)

        if mavutil is None:
            raise RuntimeError("pymavlink is required for px4_backend_adapter_node")

        self.master = mavutil.mavlink_connection(self.mavlink_url)
        self.get_logger().info(f"Connecting to PX4 via {self.mavlink_url}")
        self.master.wait_heartbeat(timeout=10)
        self.get_logger().info("PX4 heartbeat received")

        self.create_timer(1.0 / max(self.setpoint_rate_hz, 1.0), self._send_setpoint_tick)
        self.create_timer(1.0 / max(self.heartbeat_rate_hz, 1.0), self._heartbeat_tick)
        self.create_timer(1.0 / max(self.recv_rate_hz, 1.0), self._recv_tick)

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _set_offboard_and_arm(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6,  # PX4 OFFBOARD custom mode
            0,
            0,
            0,
            0,
            0,
        )
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.get_logger().info("PX4 set to OFFBOARD and armed")

    def _set_hold_and_disarm(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # PX4 HOLD custom mode
            0,
            0,
            0,
            0,
            0,
        )
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.get_logger().info("PX4 switched to HOLD and disarmed")

    def _send_setpoint_tick(self) -> None:
        if self.enabled and not self.prev_enabled:
            self._set_offboard_and_arm()
        elif (not self.enabled) and self.prev_enabled:
            self._set_hold_and_disarm()
        self.prev_enabled = self.enabled

        cmd = self.last_cmd if self.enabled else Twist()
        # Backend body frame: x=right, y=forward, z=up.
        # PX4 BODY_NED: x=forward, y=right, z=down.
        vx_body = float(cmd.linear.y)
        vy_body = float(cmd.linear.x)
        vz_body = -float(cmd.linear.z)
        yaw_rate = float(cmd.angular.z)

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            int(self.get_clock().now().nanoseconds / 1_000_000),  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0.0,
            0.0,
            0.0,
            vx_body,
            vy_body,
            vz_body,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rate,
        )

    def _heartbeat_tick(self) -> None:
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _recv_tick(self) -> None:
        # Drain available MAVLink messages and keep latest position + attitude.
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "LOCAL_POSITION_NED":
                self.last_ned = (
                    float(msg.x),
                    float(msg.y),
                    float(msg.z),
                    float(msg.vx),
                    float(msg.vy),
                    float(msg.vz),
                )
            elif mtype == "ATTITUDE":
                self.last_att_rpy = (float(msg.roll), float(msg.pitch), float(msg.yaw))

        if self.last_ned is None:
            return

        x_n, y_n, z_n, vx_n, vy_n, vz_n = self.last_ned
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        # NED -> ENU
        odom.pose.pose.position.x = y_n
        odom.pose.pose.position.y = x_n
        odom.pose.pose.position.z = -z_n
        odom.twist.twist.linear.x = vy_n
        odom.twist.twist.linear.y = vx_n
        odom.twist.twist.linear.z = -vz_n

        if self.last_att_rpy is not None:
            roll_n, pitch_n, yaw_n = self.last_att_rpy
            yaw_e = (math.pi / 2.0) - yaw_n
            qx, qy, qz, qw = _quat_from_rpy(roll_n, pitch_n, yaw_e)
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
        else:
            odom.pose.pose.orientation.w = 1.0

        self.pub_odom.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4BackendAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
