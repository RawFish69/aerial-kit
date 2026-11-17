"""ROS command/telemetry backend for a serial or simulated ELRS link."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String, UInt8MultiArray

from aerial_kit.comms.crsf import (
    AttitudeTelemetry,
    BatteryTelemetry,
    CrsfFrame,
    CrsfLink,
    CrsfParser,
    FlightModeTelemetry,
    GpsTelemetry,
    LinkStatisticsTelemetry,
    SerialTransport,
    decode_telemetry,
    make_rc_channels_frame,
)
from hw_bridge.crsf_radio_mapping import CrsfChannelMap
from hw_bridge.rc_mapping import RcSticks, neutral_sticks, params_from_airframe, velocity_to_rc

HOST_TX_TOPIC = '/uav/radio/host_tx_bytes'
HOST_RX_TOPIC = '/uav/radio/host_rx_bytes'


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


class CrsfRadioBackendNode(Node):
    """Encode velocity commands as CRSF and decode CRSF telemetry into ROS topics."""

    def __init__(self) -> None:
        super().__init__('crsf_radio_backend_node')
        defaults = {
            'transport': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baud': 420000,
            'send_rate_hz': 150.0,
            'command_timeout_sec': 0.5,
            'link_timeout_sec': 0.75,
            'airframe_name': 'quad',
            'kv_xy': 0.5,
            'hover_throttle': 0.5,
            'kz': 0.2,
            'throttle_min': 0.05,
            'throttle_max': 0.95,
            'max_yaw_rate_rps': 1.5,
            'roll_channel': 1,
            'pitch_channel': 2,
            'throttle_channel': 3,
            'yaw_channel': 4,
            'arm_channel': 5,
            'mode_channel': 6,
            'arm_threshold': 1000,
            'angle_mode_count': 992,
            'disabled_mode_count': 172,
            'vz_feedback': True,
            'vxy_feedback': False,
            'odom_timeout_sec': 0.5,
            'backend_cmd_topic': '/uav/backend/cmd_twist',
            'backend_enable_topic': '/uav/backend/enable',
            'odom_topic': '/uav/backend/odom',
            'imu_topic': '/uav/hw/imu',
            'gps_topic': '/uav/hw/gps',
            'battery_topic': '/uav/hw/battery',
            'flight_mode_topic': '/uav/radio/flight_mode',
            'link_stats_topic': '/uav/radio/link_stats',
            'status_topic': '/uav/radio/status',
            'host_tx_topic': HOST_TX_TOPIC,
            'host_rx_topic': HOST_RX_TOPIC,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        g = self.get_parameter

        self.transport_name = str(g('transport').value).strip().lower()
        self.params = params_from_airframe(
            str(g('airframe_name').value),
            kv_xy=float(g('kv_xy').value),
            hover_throttle=float(g('hover_throttle').value),
            kz=float(g('kz').value),
            throttle_min=float(g('throttle_min').value),
            throttle_max=float(g('throttle_max').value),
            max_yaw_rate_rps=float(g('max_yaw_rate_rps').value),
        )
        self.channel_map = CrsfChannelMap(
            roll=int(g('roll_channel').value),
            pitch=int(g('pitch_channel').value),
            throttle=int(g('throttle_channel').value),
            yaw=int(g('yaw_channel').value),
            arm=int(g('arm_channel').value),
            mode=int(g('mode_channel').value),
            arm_threshold=int(g('arm_threshold').value),
            angle_mode_count=int(g('angle_mode_count').value),
            disabled_mode_count=int(g('disabled_mode_count').value),
        )
        self.enabled = False
        self.last_cmd = Twist()
        self.last_cmd_time = time.monotonic()
        self.last_odom: Optional[Odometry] = None
        self.last_odom_time: Optional[float] = None
        self.last_rx_time = time.monotonic()
        self.command_timeout = float(g('command_timeout_sec').value)
        self.link_timeout = float(g('link_timeout_sec').value)
        self.odom_timeout = float(g('odom_timeout_sec').value)
        self.vz_feedback = bool(g('vz_feedback').value)
        self.vxy_feedback = bool(g('vxy_feedback').value)

        self.pub_imu = self.create_publisher(Imu, str(g('imu_topic').value), 20)
        self.pub_gps = self.create_publisher(NavSatFix, str(g('gps_topic').value), 20)
        self.pub_battery = self.create_publisher(BatteryState, str(g('battery_topic').value), 10)
        self.pub_mode = self.create_publisher(String, str(g('flight_mode_topic').value), 10)
        self.pub_link = self.create_publisher(String, str(g('link_stats_topic').value), 10)
        self.pub_status = self.create_publisher(String, str(g('status_topic').value), 10)

        self.create_subscription(Twist, str(g('backend_cmd_topic').value), self._on_cmd, 20)
        self.create_subscription(Bool, str(g('backend_enable_topic').value), self._on_enable, 20)
        self.create_subscription(Odometry, str(g('odom_topic').value), self._on_odom, 20)

        self.link: Optional[CrsfLink] = None
        self.parser = CrsfParser()
        self.pub_host_tx = None
        if self.transport_name == 'serial':
            self.link = CrsfLink(
                SerialTransport(
                    str(g('serial_port').value),
                    baudrate=int(g('serial_baud').value),
                )
            )
        elif self.transport_name == 'ros':
            self.pub_host_tx = self.create_publisher(
                UInt8MultiArray, str(g('host_tx_topic').value), 20
            )
            self.create_subscription(
                UInt8MultiArray,
                str(g('host_rx_topic').value),
                self._on_host_rx,
                20,
            )
        else:
            raise ValueError("transport must be 'serial' or 'ros'")

        rate = max(float(g('send_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'CRSF radio backend transport={self.transport_name} rate={rate:.0f} Hz'
        )

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_time = time.monotonic()

    def _on_host_rx(self, msg: UInt8MultiArray) -> None:
        for frame in self.parser.feed(bytes(msg.data)):
            self._telemetry_frame(frame)

    def _measured(self) -> tuple[Optional[tuple[float, float]], Optional[float]]:
        if self.last_odom is None or self.last_odom_time is None:
            return None, None
        if time.monotonic() - self.last_odom_time > self.odom_timeout:
            return None, None
        velocity = self.last_odom.twist.twist.linear
        vz = float(velocity.z) if self.vz_feedback else None
        vxy = None
        if self.vxy_feedback:
            q = self.last_odom.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            c, s = math.cos(yaw), math.sin(yaw)
            vxy = (
                c * float(velocity.x) + s * float(velocity.y),
                -s * float(velocity.x) + c * float(velocity.y),
            )
        return vxy, vz

    def _sticks(self) -> tuple[RcSticks, bool]:
        if not self.enabled:
            return RcSticks(0.0, 0.0, 0.0, self.params.throttle_min), False
        if time.monotonic() - self.last_cmd_time > self.command_timeout:
            return neutral_sticks(self.params), True
        measured_vxy, measured_vz = self._measured()
        command = self.last_cmd
        sticks = velocity_to_rc(
            self.params,
            vx=float(command.linear.x),
            vy=float(command.linear.y),
            vz=float(command.linear.z),
            wz=float(command.angular.z),
            measured_vxy=measured_vxy,
            measured_vz=measured_vz,
        )
        return sticks, True

    def _emit(self, frame: bytes) -> None:
        if self.link is not None:
            self.link.transport.write(frame)
        elif self.pub_host_tx is not None:
            msg = UInt8MultiArray()
            msg.data = list(frame)
            self.pub_host_tx.publish(msg)

    def _tick(self) -> None:
        sticks, armed = self._sticks()
        channels = self.channel_map.encode(sticks, enabled=armed)
        self._emit(make_rc_channels_frame(channels))

        if self.link is not None:
            for frame in self.link.poll(1024):
                self._telemetry_frame(frame)
            if time.monotonic() - self.last_rx_time > self.link_timeout:
                self._status('radio telemetry stale')

    def _status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _telemetry_frame(self, frame: CrsfFrame) -> None:
        self.last_rx_time = time.monotonic()
        try:
            telemetry = decode_telemetry(frame)
        except ValueError as exc:
            self._status(f'telemetry decode error: {exc}')
            return
        if telemetry is None:
            return

        stamp = self.get_clock().now().to_msg()
        if isinstance(telemetry, AttitudeTelemetry):
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'base_link'
            imu.orientation = quaternion_from_euler(
                telemetry.roll_rad, telemetry.pitch_rad, telemetry.yaw_rad
            )
            imu.orientation_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = -1.0
            self.pub_imu.publish(imu)
        elif isinstance(telemetry, GpsTelemetry):
            gps = NavSatFix()
            gps.header.stamp = stamp
            gps.header.frame_id = 'map'
            gps.status.status = NavSatStatus.STATUS_FIX if telemetry.satellites >= 4 else NavSatStatus.STATUS_NO_FIX
            gps.status.service = NavSatStatus.SERVICE_GPS
            gps.latitude = telemetry.latitude_deg
            gps.longitude = telemetry.longitude_deg
            gps.altitude = telemetry.altitude_m
            self.pub_gps.publish(gps)
        elif isinstance(telemetry, BatteryTelemetry):
            battery = BatteryState()
            battery.header.stamp = stamp
            battery.voltage = float(telemetry.voltage_v)
            battery.current = float(telemetry.current_a)
            battery.percentage = telemetry.remaining_percent / 100.0
            battery.present = True
            self.pub_battery.publish(battery)
        elif isinstance(telemetry, FlightModeTelemetry):
            mode = String()
            mode.data = telemetry.mode
            self.pub_mode.publish(mode)
        elif isinstance(telemetry, LinkStatisticsTelemetry):
            stats = String()
            stats.data = json.dumps({
                'rssi_dbm': telemetry.rssi_dbm,
                'uplink_rssi_1_dbm': telemetry.uplink_rssi_1_dbm,
                'uplink_rssi_2_dbm': telemetry.uplink_rssi_2_dbm,
                'uplink_link_quality': telemetry.uplink_link_quality,
                'uplink_snr_db': telemetry.uplink_snr_db,
                'downlink_rssi_dbm': telemetry.downlink_rssi_dbm,
                'downlink_link_quality': telemetry.downlink_link_quality,
            }, sort_keys=True)
            self.pub_link.publish(stats)
            self._status(
                f"radio link up lq={telemetry.uplink_link_quality}% "
                f"rssi={telemetry.rssi_dbm}dBm"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrsfRadioBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.link is not None:
            node.link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
