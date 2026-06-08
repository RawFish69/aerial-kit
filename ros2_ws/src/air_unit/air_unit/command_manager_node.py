import math
from typing import Optional

import rclpy
from air_unit.rtl import RTL_ARRIVED, RtlParams, rtl_command
from drone_msgs.msg import Command, Telemetry
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CommandManagerNode(Node):
    """Authoritative air-unit state machine and backend command arbiter."""

    def __init__(self) -> None:
        super().__init__('command_manager_node')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('telemetry_raw_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('mission_cmd_topic', '/uav/internal/mission_cmd_vel')
        self.declare_parameter('backend_cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('backend_enable_topic', '/uav/backend/enable')
        self.declare_parameter('telemetry_topic', '/uav/telemetry')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 2.0)
        self.declare_parameter('mission_cmd_timeout_sec', 1.0)
        self.declare_parameter('takeoff_altitude_m', 1.5)
        self.declare_parameter('takeoff_rate_mps', 0.8)
        self.declare_parameter('takeoff_confirm_ticks', 8)
        self.declare_parameter('land_rate_mps', 0.5)
        self.declare_parameter('land_altitude_threshold_m', 0.15)
        self.declare_parameter('land_confirm_ticks', 5)
        # Hover hold (simple stabilizer): damps drift and holds current altitude.
        self.declare_parameter('hover_hold_enabled', True)
        self.declare_parameter('hover_velocity_damping_xy', 0.8)
        self.declare_parameter('hover_velocity_damping_z', 0.8)
        self.declare_parameter('hover_altitude_kp', 1.0)
        self.declare_parameter('hover_max_xy_speed_mps', 1.5)
        self.declare_parameter('hover_max_vertical_speed_mps', 0.8)
        self.declare_parameter('manual_scale_xy', 1.5)
        self.declare_parameter('manual_scale_z', 1.0)
        self.declare_parameter('manual_scale_yaw', 1.0)
        self.declare_parameter('rtl_altitude_m', 15.0)
        self.declare_parameter('rtl_cruise_speed_mps', 3.0)
        self.declare_parameter('rtl_arrival_radius_m', 1.0)
        self.declare_parameter('rtl_kp', 0.8)

        self.command_topic = self.get_parameter('command_topic').value
        self.telemetry_raw_topic = self.get_parameter('telemetry_raw_topic').value
        self.mission_cmd_topic = self.get_parameter('mission_cmd_topic').value
        self.backend_cmd_topic = self.get_parameter('backend_cmd_topic').value
        self.backend_enable_topic = self.get_parameter('backend_enable_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.mission_cmd_timeout_sec = float(self.get_parameter('mission_cmd_timeout_sec').value)
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.takeoff_rate_mps = float(self.get_parameter('takeoff_rate_mps').value)
        self.takeoff_confirm_ticks = int(self.get_parameter('takeoff_confirm_ticks').value)
        self.land_rate_mps = float(self.get_parameter('land_rate_mps').value)
        self.land_altitude_threshold_m = float(self.get_parameter('land_altitude_threshold_m').value)
        self.land_confirm_ticks = int(self.get_parameter('land_confirm_ticks').value)
        self.hover_hold_enabled = bool(self.get_parameter('hover_hold_enabled').value)
        self.hover_velocity_damping_xy = float(self.get_parameter('hover_velocity_damping_xy').value)
        self.hover_velocity_damping_z = float(self.get_parameter('hover_velocity_damping_z').value)
        self.hover_altitude_kp = float(self.get_parameter('hover_altitude_kp').value)
        self.hover_max_xy_speed_mps = float(self.get_parameter('hover_max_xy_speed_mps').value)
        self.hover_max_vertical_speed_mps = float(self.get_parameter('hover_max_vertical_speed_mps').value)
        self.manual_scale_xy = float(self.get_parameter('manual_scale_xy').value)
        self.manual_scale_z = float(self.get_parameter('manual_scale_z').value)
        self.manual_scale_yaw = float(self.get_parameter('manual_scale_yaw').value)
        self.rtl_altitude_m = float(self.get_parameter('rtl_altitude_m').value)
        self.rtl_cruise_speed_mps = float(self.get_parameter('rtl_cruise_speed_mps').value)
        self.rtl_arrival_radius_m = float(self.get_parameter('rtl_arrival_radius_m').value)
        self.rtl_kp = float(self.get_parameter('rtl_kp').value)

        self.pub_backend_cmd = self.create_publisher(Twist, self.backend_cmd_topic, 10)
        self.pub_backend_enable = self.create_publisher(Bool, self.backend_enable_topic, 10)
        self.pub_telemetry = self.create_publisher(Telemetry, self.telemetry_topic, 20)

        self.create_subscription(Command, self.command_topic, self._on_command, 20)
        self.create_subscription(Telemetry, self.telemetry_raw_topic, self._on_telemetry_raw, 20)
        self.create_subscription(Twist, self.mission_cmd_topic, self._on_mission_cmd, 20)

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.mode = Command.MODE_IDLE
        self.planning_mode = Command.PLANNING_OFFBOARD
        self.armed = False
        self.manual_override = False
        self.last_manual_cmd = Twist()
        self.last_mission_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_mission_cmd_time = self.get_clock().now()
        self.last_raw_telemetry: Optional[Telemetry] = None
        self.last_output = Twist()
        self.status_text = 'idle'
        self.takeoff_confirmed_count = 0
        self.land_confirmed_count = 0
        self._last_armed_state: bool | None = None  # track to suppress redundant enable publishes
        self._enable_republish_tick: int = 0  # periodic republish when armed so backend/Gazebo get it
        self._hover_target_altitude: Optional[float] = None
        self._home_xy: tuple[float, float] | None = None
        self._prev_mode = self.mode

        self.get_logger().info('Command manager started')

    def _on_command(self, msg: Command) -> None:
        self.last_cmd_time = self.get_clock().now()
        self.planning_mode = int(msg.planning_mode)
        self.manual_override = bool(msg.manual_override)
        was_armed = self.armed

        if msg.disarm:
            self.armed = False
            self.mode = Command.MODE_IDLE
            self.status_text = 'disarmed'
            self._home_xy = None
        elif msg.arm:
            self.armed = True
            self.status_text = 'armed'

        self.mode = int(msg.mode_request)
        self.last_manual_cmd = self._command_to_manual_twist(msg)
        if msg.arm and not was_armed and self.last_raw_telemetry is not None and self._home_xy is None:
            self._home_xy = (
                float(self.last_raw_telemetry.pose.position.x),
                float(self.last_raw_telemetry.pose.position.y),
            )

    def _on_telemetry_raw(self, msg: Telemetry) -> None:
        self.last_raw_telemetry = msg
        self._publish_enriched_telemetry(msg)

    def _on_mission_cmd(self, msg: Twist) -> None:
        self.last_mission_cmd = msg
        self.last_mission_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        cmd_timeout = (now - self.last_cmd_time).nanoseconds / 1e9
        mission_timeout = (now - self.last_mission_cmd_time).nanoseconds / 1e9

        if cmd_timeout > self.command_timeout_sec and self.armed and not self.manual_override:
            if self.mode not in (Command.MODE_HOVER, Command.MODE_LAND):
                self.mode = Command.MODE_HOVER
                self.status_text = f'command timeout {cmd_timeout:.1f}s -> hover'

        enable = Bool()
        enable.data = bool(self.armed)
        cmd = Twist()
        # Publish enable on state change; also republish periodically when armed so
        # late-joining bridge or Gazebo reliably receive it (avoids drone never taking off).
        _publish_enable = (self._last_armed_state != enable.data)
        self._last_armed_state = enable.data
        self._enable_republish_tick += 1
        if self.armed and self._enable_republish_tick >= int(max(1, self.control_rate_hz)):
            self._enable_republish_tick = 0
            _publish_enable = True
        altitude = self._current_altitude()

        if not self.armed:
            self.status_text = 'idle/disarmed'
            self.takeoff_confirmed_count = 0
            self._hover_target_altitude = None
        elif self.manual_override or self.mode == Command.MODE_MANUAL_OVERRIDE:
            cmd = self.last_manual_cmd
            self.status_text = 'manual override'
            self.takeoff_confirmed_count = 0
            self._hover_target_altitude = None
        elif self.mode == Command.MODE_TAKEOFF:
            if altitude is not None and altitude >= self.takeoff_altitude_m:
                self.takeoff_confirmed_count += 1
                if self.takeoff_confirmed_count >= self.takeoff_confirm_ticks:
                    self.mode = Command.MODE_HOVER
                    self.status_text = 'takeoff complete -> hover'
                else:
                    cmd.linear.z = self.takeoff_rate_mps
                    self.status_text = (
                        f'takeoff confirm ({self.takeoff_confirmed_count}/{self.takeoff_confirm_ticks})'
                    )
            else:
                self.takeoff_confirmed_count = 0
                cmd.linear.z = self.takeoff_rate_mps
                self.status_text = 'takeoff'
        elif self.mode == Command.MODE_HOVER:
            self.takeoff_confirmed_count = 0
            if self.hover_hold_enabled and self.last_raw_telemetry is not None:
                if self._prev_mode != Command.MODE_HOVER or self._hover_target_altitude is None:
                    self._hover_target_altitude = altitude
                vx = float(self.last_raw_telemetry.twist.linear.x)
                vy = float(self.last_raw_telemetry.twist.linear.y)
                vz = float(self.last_raw_telemetry.twist.linear.z)
                cmd.linear.x = max(
                    -self.hover_max_xy_speed_mps,
                    min(self.hover_max_xy_speed_mps, -self.hover_velocity_damping_xy * vx),
                )
                cmd.linear.y = max(
                    -self.hover_max_xy_speed_mps,
                    min(self.hover_max_xy_speed_mps, -self.hover_velocity_damping_xy * vy),
                )
                if altitude is not None and self._hover_target_altitude is not None:
                    alt_err = float(self._hover_target_altitude) - float(altitude)
                    z_cmd = self.hover_altitude_kp * alt_err - self.hover_velocity_damping_z * vz
                else:
                    z_cmd = -self.hover_velocity_damping_z * vz
                cmd.linear.z = max(
                    -self.hover_max_vertical_speed_mps,
                    min(self.hover_max_vertical_speed_mps, z_cmd),
                )
                self.status_text = 'hover hold'
            else:
                self.status_text = 'hover'
        elif self.mode == Command.MODE_MISSION:
            self.takeoff_confirmed_count = 0
            if mission_timeout <= self.mission_cmd_timeout_sec:
                cmd = self.last_mission_cmd
                self.status_text = 'mission tracking'
            else:
                self.status_text = 'mission waiting for setpoint'
            self._hover_target_altitude = None
        elif self.mode == Command.MODE_LAND:
            self.takeoff_confirmed_count = 0
            if altitude is not None and altitude <= self.land_altitude_threshold_m:
                self.land_confirmed_count += 1
                if self.land_confirmed_count >= self.land_confirm_ticks:
                    self.armed = False
                    enable.data = False
                    self.mode = Command.MODE_IDLE
                    self._home_xy = None
                    self.land_confirmed_count = 0
                    self.status_text = 'landed -> disarmed'
                else:
                    self.status_text = f'landing ({self.land_confirmed_count}/{self.land_confirm_ticks})'
            else:
                self.land_confirmed_count = 0
                cmd.linear.z = -abs(self.land_rate_mps)
                self.status_text = 'landing'
        elif self.mode == Command.MODE_RTL:
            self.takeoff_confirmed_count = 0
            if self._home_xy is None or altitude is None or self.last_raw_telemetry is None:
                self.mode = Command.MODE_LAND
                self.status_text = 'RTL: no home -> land in place'
            else:
                params = RtlParams(
                    self.rtl_altitude_m,
                    self.rtl_cruise_speed_mps,
                    self.rtl_arrival_radius_m,
                    self.rtl_kp,
                )
                pos = (
                    float(self.last_raw_telemetry.pose.position.x),
                    float(self.last_raw_telemetry.pose.position.y),
                    float(altitude),
                )
                vx, vy, vz, phase = rtl_command(params, pos, self._home_xy)
                if phase == RTL_ARRIVED:
                    self.mode = Command.MODE_LAND
                    self.status_text = 'RTL: arrived -> land'
                else:
                    cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
                    self.status_text = f'RTL: {phase} ({vx:.1f},{vy:.1f},{vz:.1f})'
        else:
            self.status_text = f'mode {self.mode} idle behavior'

        self.last_output = cmd
        if _publish_enable:
            self.pub_backend_enable.publish(enable)
        self.pub_backend_cmd.publish(cmd)
        self._prev_mode = self.mode

        if self.last_raw_telemetry is not None:
            self._publish_enriched_telemetry(self.last_raw_telemetry)

    def _publish_enriched_telemetry(self, raw: Telemetry) -> None:
        msg = Telemetry()
        msg.header = raw.header
        msg.pose = raw.pose
        msg.twist = raw.twist
        msg.battery_percent = raw.battery_percent
        msg.current_mode = int(self.mode)
        msg.planning_mode = int(self.planning_mode)
        msg.armed = bool(self.armed)
        msg.manual_override_active = bool(self.manual_override)
        msg.status_text = self.status_text or raw.status_text
        self.pub_telemetry.publish(msg)

    def _current_altitude(self) -> Optional[float]:
        if self.last_raw_telemetry is None:
            return None
        return float(self.last_raw_telemetry.pose.position.z)

    def _command_to_manual_twist(self, msg: Command) -> Twist:
        out = Twist()
        vs = msg.velocity_setpoint
        if not self._is_zero_twist(vs):
            out = vs
            return out

        if msg.manual.active:
            out.linear.x = float(msg.manual.pitch) * self.manual_scale_xy
            out.linear.y = float(msg.manual.roll) * self.manual_scale_xy
            out.linear.z = (float(msg.manual.throttle) - 0.5) * 2.0 * self.manual_scale_z
            out.angular.z = float(msg.manual.yaw) * self.manual_scale_yaw
        return out

    @staticmethod
    def _is_zero_twist(t: Twist) -> bool:
        vals = (
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z,
        )
        return all(math.isclose(v, 0.0, abs_tol=1e-6) for v in vals)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
