from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64

from hw_bridge.fusion import AltitudeFilter
from hw_bridge.geo import GeoOrigin, lla_to_enu


class HwStateEstimatorNode(Node):
    """Fuse IMU attitude + barometer altitude + GPS horizontal into /uav/backend/odom (ENU)."""

    def __init__(self) -> None:
        super().__init__("hw_state_estimator_node")
        self.declare_parameter("imu_topic", "/uav/hw/imu")
        self.declare_parameter("baro_topic", "/uav/hw/baro")
        self.declare_parameter("gps_topic", "/uav/hw/gps")
        self.declare_parameter("enable_topic", "/uav/backend/enable")
        self.declare_parameter("odom_topic", "/uav/backend/odom")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("baro_alpha", 0.98)
        self.declare_parameter("min_gps_status", 0)  # NavSatStatus.STATUS_FIX = 0

        g = self.get_parameter
        self.min_gps_status = int(g("min_gps_status").value)
        self.alt_filter = AltitudeFilter(alpha=float(g("baro_alpha").value))

        self.origin: Optional[GeoOrigin] = None
        self.home_baro: Optional[float] = None
        self.enabled = False
        self.last_enabled = False
        self.orientation = Quaternion(w=1.0)
        self.vz_imu = 0.0
        self.last_baro: Optional[float] = None
        self.last_fix: Optional[NavSatFix] = None
        self.east = 0.0
        self.north = 0.0
        self.prev_east: Optional[float] = None
        self.prev_north: Optional[float] = None
        self.vx = 0.0
        self.vy = 0.0

        self.pub = self.create_publisher(Odometry, str(g("odom_topic").value), 10)
        self.create_subscription(Imu, str(g("imu_topic").value), self._on_imu, 20)
        self.create_subscription(Float64, str(g("baro_topic").value), self._on_baro, 20)
        self.create_subscription(NavSatFix, str(g("gps_topic").value), self._on_gps, 10)
        self.create_subscription(Bool, str(g("enable_topic").value), self._on_enable, 10)

        self._period = 1.0 / max(float(g("publish_rate_hz").value), 1.0)
        self.timer = self.create_timer(self._period, self._tick)
        self.get_logger().info("hw_state_estimator started")

    def _on_imu(self, msg: Imu) -> None:
        self.orientation = msg.orientation
        self.vz_imu += float(msg.linear_acceleration.z) * 0.0  # placeholder; baro drives vz

    def _on_baro(self, msg: Float64) -> None:
        self.last_baro = float(msg.data)

    def _on_gps(self, msg: NavSatFix) -> None:
        self.last_fix = msg

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _maybe_capture_home(self) -> None:
        armed_edge = self.enabled and not self.last_enabled
        self.last_enabled = self.enabled
        if not armed_edge:
            return
        if self.last_fix is not None and self.last_fix.status.status >= self.min_gps_status:
            self.origin = GeoOrigin(lat=self.last_fix.latitude, lon=self.last_fix.longitude)
            self.home_baro = self.last_baro if self.last_baro is not None else 0.0
            self.alt_filter.reset()
            self.get_logger().info(
                f"Home captured: ({self.origin.lat:.7f}, {self.origin.lon:.7f}) baro={self.home_baro}"
            )
        else:
            self.get_logger().warn("Arm edge but no GPS fix — home not captured")

    def _tick(self) -> None:
        self._maybe_capture_home()

        # Altitude (relative to home baro)
        z = 0.0
        if self.last_baro is not None:
            baro_rel = self.last_baro - (
                self.home_baro if self.home_baro is not None else self.last_baro
            )
            gps_alt = None
            if self.last_fix is not None and self.home_baro is not None:
                gps_alt = None  # GPS altitude noisy; left None unless enabled later
            z = self.alt_filter.update(baro_alt=baro_rel, vz=0.0, dt=self._period, gps_alt=gps_alt)

        # Horizontal from GPS relative to home origin
        if (
            self.origin is not None
            and self.last_fix is not None
            and self.last_fix.status.status >= self.min_gps_status
        ):
            self.east, self.north = lla_to_enu(
                self.origin, self.last_fix.latitude, self.last_fix.longitude
            )
            if self.prev_east is not None and self.prev_north is not None:
                self.vx = (self.east - self.prev_east) / self._period
                self.vy = (self.north - self.prev_north) / self._period
            self.prev_east, self.prev_north = self.east, self.north

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.east)
        odom.pose.pose.position.y = float(self.north)
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation = self.orientation
        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.linear.y = float(self.vy)
        self.pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HwStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
