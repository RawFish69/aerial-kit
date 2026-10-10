from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64

from hw_bridge.fusion import BaroVerticalFilter, EkfFusion, GpsVelocity
from hw_bridge.geo import GeoOrigin, lla_to_enu


class HwStateEstimatorNode(Node):
    """Fuse IMU, barometer and GPS into /uav/backend/odom (ENU).

    ``estimator: ekf`` (default) runs aerial_kit's INS EKF - position, velocity,
    accelerometer and baro bias, GPS outliers gated. ``imu_accel_mode`` says
    what the FC puts in ``Imu.linear_acceleration``: ``none`` (default; e.g.
    Betaflight over CRSF sends attitude only), ``body_specific_force`` (REP-145)
    or ``world_linear`` (gravity removed, world axes, as ``fake_fc_sim``).
    Orientation is taken as z-up body to ENU world (REP-103) either way.

    ``estimator: complementary`` keeps the previous baro filter and per-fix GPS
    differencing.
    """

    def __init__(self, **node_kwargs) -> None:
        super().__init__("hw_state_estimator_node", **node_kwargs)
        self.declare_parameter("imu_topic", "/uav/hw/imu")
        self.declare_parameter("baro_topic", "/uav/hw/baro")
        self.declare_parameter("gps_topic", "/uav/hw/gps")
        self.declare_parameter("enable_topic", "/uav/backend/enable")
        self.declare_parameter("odom_topic", "/uav/backend/odom")
        self.declare_parameter("publish_rate_hz", 30.0)
        # Bandwidth of the baro altitude / climb-rate filter (see fusion.py).
        self.declare_parameter("baro_filter_hz", 1.0)
        self.declare_parameter("min_gps_status", 0)  # NavSatStatus.STATUS_FIX = 0
        self.declare_parameter("estimator", "ekf")  # ekf | complementary
        self.declare_parameter("imu_accel_mode", "none")  # none | body_specific_force | world_linear
        self.declare_parameter("accel_noise", -1.0)  # m/s^2/sqrt(Hz); < 0 picks per mode
        self.declare_parameter("gps_sigma_xy", 1.5)
        self.declare_parameter("gps_sigma_z", 3.0)
        self.declare_parameter("baro_sigma", 0.3)
        self.declare_parameter("use_gps_altitude", True)

        g = self.get_parameter
        self.min_gps_status = int(g("min_gps_status").value)
        self.alt_filter = BaroVerticalFilter(hz=float(g("baro_filter_hz").value))
        self.estimator = str(g("estimator").value).strip().lower()
        if self.estimator not in ("ekf", "complementary"):
            raise ValueError(f"estimator must be 'ekf' or 'complementary', got {self.estimator!r}")
        self.use_gps_altitude = bool(g("use_gps_altitude").value)
        self.fusion = None
        if self.estimator == "ekf":
            from aerial_kit.estimation import InsConfig

            accel_noise = float(g("accel_noise").value)
            self.fusion = EkfFusion(
                InsConfig(
                    accel_mode=str(g("imu_accel_mode").value).strip(),
                    accel_noise=accel_noise if accel_noise > 0.0 else None,
                    gps_sigma_xy=float(g("gps_sigma_xy").value),
                    gps_sigma_z=float(g("gps_sigma_z").value),
                    baro_sigma=float(g("baro_sigma").value),
                )
            )
        self._last_fix_t: Optional[float] = None

        self.origin: Optional[GeoOrigin] = None
        self.home_baro: Optional[float] = None
        self.enabled = False
        self.last_enabled = False
        self.orientation = Quaternion(w=1.0)
        self.vz = 0.0
        self.last_baro: Optional[float] = None
        self.last_fix: Optional[NavSatFix] = None
        self.east = 0.0
        self.north = 0.0
        self.gps_velocity = GpsVelocity()
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

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        self.orientation = msg.orientation
        if self.fusion is not None:
            q = msg.orientation
            a = msg.linear_acceleration
            self.fusion.on_imu(self._now(), [a.x, a.y, a.z], [q.w, q.x, q.y, q.z])

    def _on_baro(self, msg: Float64) -> None:
        self.last_baro = float(msg.data)
        if self.fusion is not None:
            self.fusion.on_baro(self._now(), self.last_baro)

    def _on_gps(self, msg: NavSatFix) -> None:
        self.last_fix = msg
        if self.fusion is None or self.origin is None or msg.status.status < self.min_gps_status:
            return
        stamp = msg.header.stamp
        t_fix = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
        if self._last_fix_t is not None and t_fix <= self._last_fix_t:
            return  # the same fix again
        self._last_fix_t = t_fix
        east, north = lla_to_enu(self.origin, msg.latitude, msg.longitude)
        altitude = float(msg.altitude) if self.use_gps_altitude else None
        self.fusion.on_gps(self._now(), east, north, altitude)

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
            self.gps_velocity.reset()
            if self.fusion is not None:
                self._last_fix_t = None
                self.fusion.capture_home(
                    self._now(),
                    float(self.last_fix.altitude) if self.use_gps_altitude else None,
                    self.last_baro,
                )
            self.get_logger().info(
                f"Home captured: ({self.origin.lat:.7f}, {self.origin.lon:.7f}) baro={self.home_baro}"
            )
        else:
            self.get_logger().warn("Arm edge but no GPS fix — home not captured")

    def _tick(self) -> None:
        self._maybe_capture_home()
        if self.fusion is not None:
            return self._publish_ekf()

        # Altitude (relative to home baro)
        z = 0.0
        if self.last_baro is not None:
            baro_rel = self.last_baro - (
                self.home_baro if self.home_baro is not None else self.last_baro
            )
            gps_alt = None
            if self.last_fix is not None and self.home_baro is not None:
                gps_alt = None  # GPS altitude noisy; left None unless enabled later
            z, self.vz = self.alt_filter.update(baro_alt=baro_rel, dt=self._period, gps_alt=gps_alt)

        # Horizontal from GPS relative to home origin
        if (
            self.origin is not None
            and self.last_fix is not None
            and self.last_fix.status.status >= self.min_gps_status
        ):
            self.east, self.north = lla_to_enu(
                self.origin, self.last_fix.latitude, self.last_fix.longitude
            )
            stamp = self.last_fix.header.stamp
            t_fix = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
            self.vx, self.vy = self.gps_velocity.update(self.east, self.north, t_fix)

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
        odom.twist.twist.linear.z = float(self.vz)
        self.pub.publish(odom)


    def _publish_ekf(self) -> None:
        position, velocity = self.fusion.state_at(self._now())
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = (float(c) for c in position)
        odom.pose.pose.orientation = self.orientation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = (float(c) for c in velocity)
        if self.fusion.home:
            sp = self.fusion.ekf.position_sigma()
            sv = self.fusion.ekf.velocity_sigma()
            for i in range(3):
                odom.pose.covariance[i * 7] = float(sp[i] ** 2)
                odom.twist.covariance[i * 7] = float(sv[i] ** 2)
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
