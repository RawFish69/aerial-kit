import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from hw_bridge.rc_mapping import RcMapParams, RcSticks, neutral_sticks, velocity_to_rc

_PACKET_FMT = "<ffffI"  # roll, pitch, yaw (-1..1), throttle (0..1), timestamp_ms


class CrsfBackendAdapterNode(Node):
    """Translate /uav/backend velocity+enable into <ffffI> UDP packets for the ESP32 TX."""

    def __init__(self) -> None:
        super().__init__("crsf_backend_adapter_node")
        self.declare_parameter("backend_cmd_topic", "/uav/backend/cmd_twist")
        self.declare_parameter("backend_enable_topic", "/uav/backend/enable")
        self.declare_parameter("udp_host", "192.168.4.1")
        self.declare_parameter("udp_port", 9000)
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        # rc mapping params
        self.declare_parameter("kv_xy", 0.5)
        self.declare_parameter("max_tilt_deg", 25.0)
        self.declare_parameter("hover_throttle", 0.5)
        self.declare_parameter("kz", 0.2)
        self.declare_parameter("throttle_min", 0.05)
        self.declare_parameter("throttle_max", 0.95)
        self.declare_parameter("max_yaw_rate_rps", 1.5)
        self.declare_parameter("disarmed_throttle", 0.0)

        g = self.get_parameter
        self.udp_host = str(g("udp_host").value)
        self.udp_port = int(g("udp_port").value)
        self.command_timeout_sec = float(g("command_timeout_sec").value)
        self.disarmed_throttle = float(g("disarmed_throttle").value)
        self.params = RcMapParams(
            kv_xy=float(g("kv_xy").value),
            max_tilt_deg=float(g("max_tilt_deg").value),
            hover_throttle=float(g("hover_throttle").value),
            kz=float(g("kz").value),
            throttle_min=float(g("throttle_min").value),
            throttle_max=float(g("throttle_max").value),
            max_yaw_rate_rps=float(g("max_yaw_rate_rps").value),
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.enabled = False
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self._t0 = time.monotonic()

        self.create_subscription(Twist, str(g("backend_cmd_topic").value), self._on_cmd, 10)
        self.create_subscription(Bool, str(g("backend_enable_topic").value), self._on_enable, 10)
        period = 1.0 / max(float(g("send_rate_hz").value), 1.0)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"CRSF adapter -> udp://{self.udp_host}:{self.udp_port}")

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _tick(self) -> None:
        if not self.enabled:
            sticks = RcSticks(0.0, 0.0, 0.0, self.disarmed_throttle)
        else:
            elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if elapsed > self.command_timeout_sec:
                sticks = neutral_sticks(self.params)  # level + hover throttle, NOT zero
                self.get_logger().warn(
                    f"cmd timeout {elapsed:.2f}s -> neutral hover", throttle_duration_sec=2.0
                )
            else:
                c = self.last_cmd
                sticks = velocity_to_rc(
                    self.params,
                    vx=float(c.linear.x),
                    vy=float(c.linear.y),
                    vz=float(c.linear.z),
                    wz=float(c.angular.z),
                )
        self._send(sticks)

    def _send(self, s: RcSticks) -> None:
        ts = int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF
        pkt = struct.pack(_PACKET_FMT, s.roll, s.pitch, s.yaw, s.throttle, ts)
        try:
            self.sock.sendto(pkt, (self.udp_host, self.udp_port))
        except OSError as exc:
            self.get_logger().warn(f"UDP send failed: {exc}", throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrsfBackendAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
