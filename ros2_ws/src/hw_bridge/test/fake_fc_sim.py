import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from hw_bridge.geo import GeoOrigin, enu_to_lla

_PACKET_FMT = "<ffffI"
_GRAVITY = 9.80665


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class FakeFcSimNode(Node):
    """Simple closed-loop FC simulator for CRSF adapter testing."""

    def __init__(self) -> None:
        super().__init__("fake_fc_sim")
        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 9000)
        self.declare_parameter("sim_rate_hz", 100.0)
        self.declare_parameter("max_tilt_deg", 25.0)
        self.declare_parameter("hover_throttle", 0.5)
        self.declare_parameter("throttle_accel_gain", 30.0)
        self.declare_parameter("max_yaw_rate_rps", 1.2)
        self.declare_parameter("drag_xy", 0.4)
        self.declare_parameter("drag_z", 0.5)
        self.declare_parameter("origin_lat", 37.0000000)
        self.declare_parameter("origin_lon", -122.0000000)
        self.declare_parameter("origin_alt_m", 100.0)
        self.declare_parameter("imu_topic", "/uav/hw/imu")
        self.declare_parameter("baro_topic", "/uav/hw/baro")
        self.declare_parameter("gps_topic", "/uav/hw/gps")

        g = self.get_parameter
        self.max_tilt_rad = math.radians(float(g("max_tilt_deg").value))
        self.hover_throttle = float(g("hover_throttle").value)
        self.throttle_accel_gain = float(g("throttle_accel_gain").value)
        self.max_yaw_rate_rps = float(g("max_yaw_rate_rps").value)
        self.drag_xy = float(g("drag_xy").value)
        self.drag_z = float(g("drag_z").value)
        self.origin_alt_m = float(g("origin_alt_m").value)
        self.origin = GeoOrigin(lat=float(g("origin_lat").value), lon=float(g("origin_lon").value))

        self.pub_imu = self.create_publisher(Imu, str(g("imu_topic").value), 20)
        self.pub_baro = self.create_publisher(Float64, str(g("baro_topic").value), 20)
        self.pub_gps = self.create_publisher(NavSatFix, str(g("gps_topic").value), 20)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((str(g("listen_host").value), int(g("listen_port").value)))
        self.sock.setblocking(False)

        self.roll_stick = 0.0
        self.pitch_stick = 0.0
        self.yaw_stick = 0.0
        self.throttle = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self._last_tick = time.monotonic()
        period = 1.0 / max(float(g("sim_rate_hz").value), 1.0)
        self.timer = self.create_timer(period, self._tick)

    def _recv_packets(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(128)
            except BlockingIOError:
                return
            if len(data) != struct.calcsize(_PACKET_FMT):
                continue
            try:
                roll, pitch, yaw, throttle, _ = struct.unpack(_PACKET_FMT, data)
            except struct.error:
                continue
            self.roll_stick = _clamp(float(roll), -1.0, 1.0)
            self.pitch_stick = _clamp(float(pitch), -1.0, 1.0)
            self.yaw_stick = _clamp(float(yaw), -1.0, 1.0)
            self.throttle = _clamp(float(throttle), 0.0, 1.0)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._last_tick)
        self._last_tick = now

        self._recv_packets()

        roll_angle = self.roll_stick * self.max_tilt_rad
        pitch_angle = self.pitch_stick * self.max_tilt_rad
        a_roll = _GRAVITY * math.tan(roll_angle)
        a_pitch = _GRAVITY * math.tan(pitch_angle)
        s = math.sin(self.yaw)
        c = math.cos(self.yaw)

        # Body +X is right, body +Y is forward.
        ax_world = (a_roll * c) + (a_pitch * -s)
        ay_world = (a_roll * s) + (a_pitch * c)
        az_world = ((self.throttle - self.hover_throttle) * self.throttle_accel_gain) - (
            self.drag_z * self.vz
        )

        self.ax = ax_world - (self.drag_xy * self.vx)
        self.ay = ay_world - (self.drag_xy * self.vy)
        self.az = az_world

        self.vx += self.ax * dt
        self.vy += self.ay * dt
        self.vz += self.az * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        if self.z < 0.0:
            self.z = 0.0
            if self.vz < 0.0:
                self.vz = 0.0

        self.yaw += (self.yaw_stick * self.max_yaw_rate_rps) * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self._publish_sensors()

    def _publish_sensors(self) -> None:
        stamp = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation.z = math.sin(self.yaw * 0.5)
        imu.orientation.w = math.cos(self.yaw * 0.5)
        imu.linear_acceleration.x = float(self.ax)
        imu.linear_acceleration.y = float(self.ay)
        imu.linear_acceleration.z = float(self.az)
        self.pub_imu.publish(imu)

        baro = Float64()
        baro.data = float(self.origin_alt_m + self.z)
        self.pub_baro.publish(baro)

        lat, lon = enu_to_lla(self.origin, self.x, self.y)
        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = "map"
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude = float(lat)
        gps.longitude = float(lon)
        gps.altitude = float(self.origin_alt_m + self.z)
        self.pub_gps.publish(gps)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeFcSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
