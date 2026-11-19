"""Decode SIL CRSF commands for the plant and encode CRSF flight telemetry."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray
from uav_msgs.msg import MultirotorManualCommand

from aerial_kit.comms.crsf import (
    BarometerTelemetry,
    make_barometer_frame,
    AttitudeTelemetry,
    BatteryTelemetry,
    CrsfParser,
    FlightModeTelemetry,
    GpsTelemetry,
    make_attitude_frame,
    make_battery_frame,
    make_flight_mode_frame,
    make_gps_frame,
    unpack_channels,
)
from hw_bridge.crsf_radio_mapping import CrsfChannelMap, ReceiverArmingGate
from hw_bridge.geo import GeoOrigin, enu_to_lla
from hw_bridge.rc_mapping import params_from_airframe


def euler_from_quaternion(q) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class CrsfSimReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__('crsf_sim_receiver_node')
        defaults = {
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
            'arm_channel': 6,
            'mode_channel': 5,
            'arm_threshold': 1300,
            'angle_mode_count': 1811,
            'disabled_mode_count': 172,
            'output_rate_hz': 150.0,
            'telemetry_rate_hz': 50.0,
            # Barometric altitude + vertical speed (CRSF 0x09): the host's fast
            # vertical channel. Off reproduces the GPS-altitude-only downlink.
            'send_barometer': True,
            'receiver_timeout_sec': 0.35,
            'arm_hold_sec': 0.5,
            'throttle_low': 0.05,
            'receiver_rx_topic': '/uav/radio/receiver_rx_bytes',
            'receiver_tx_topic': '/uav/radio/receiver_tx_bytes',
            'plant_odom_topic': '/uav/radio/plant_odom',
            # What a flight controller sends as attitude telemetry is its *own*
            # estimate. When a firmware-in-the-loop plant publishes it here
            # (roll, pitch, yaw; ENU/FLU like the odometry) it is used while
            # fresh; otherwise the plant's true attitude is sent.
            'fc_attitude_topic': '/uav/radio/fc_attitude_rpy',
            'decoded_cmd_topic': '/uav/radio/decoded_cmd_twist',
            'decoded_manual_topic': '/uav/radio/decoded_manual',
            'decoded_manual_mode': 'angle',
            'decoded_enable_topic': '/uav/radio/decoded_enable',
            'origin_lat': 37.0,
            'origin_lon': -122.0,
            'origin_alt_m': 100.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        g = self.get_parameter
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
        self.parser = CrsfParser()
        self.last_channels: Optional[tuple[int, ...]] = None
        self.last_rx_time = 0.0
        self.receiver_timeout = float(g('receiver_timeout_sec').value)
        self.arming_gate = ReceiverArmingGate(
            arm_hold_sec=max(float(g('arm_hold_sec').value), 0.0),
            throttle_low=float(g('throttle_low').value),
        )
        manual_mode = str(g('decoded_manual_mode').value).strip().lower()
        if manual_mode not in ('angle', 'velocity'):
            raise ValueError("decoded_manual_mode must be 'angle' or 'velocity'")
        self.manual_mode = {
            'angle': MultirotorManualCommand.MODE_ANGLE,
            'velocity': MultirotorManualCommand.MODE_VELOCITY,
        }[manual_mode]
        self._last_arm_request: Optional[bool] = None
        self._last_gate_armed = False
        self.last_odom: Optional[Odometry] = None
        self.origin = GeoOrigin(
            lat=float(g('origin_lat').value),
            lon=float(g('origin_lon').value),
        )
        self.origin_alt = float(g('origin_alt_m').value)
        self.battery_percent = 100.0

        self.pub_cmd = self.create_publisher(Twist, str(g('decoded_cmd_topic').value), 20)
        self.pub_manual = self.create_publisher(
            MultirotorManualCommand, str(g('decoded_manual_topic').value), 20
        )
        self.pub_enable = self.create_publisher(Bool, str(g('decoded_enable_topic').value), 20)
        self.pub_tx = self.create_publisher(UInt8MultiArray, str(g('receiver_tx_topic').value), 20)
        self.create_subscription(
            UInt8MultiArray, str(g('receiver_rx_topic').value), self._on_rx, 20
        )
        self.create_subscription(
            Odometry, str(g('plant_odom_topic').value), self._on_odom, 20
        )
        self.fc_attitude: Optional[tuple[float, float, float]] = None
        self.fc_attitude_time = 0.0
        self.create_subscription(
            Vector3, str(g('fc_attitude_topic').value), self._on_fc_attitude, 20
        )
        output_hz = max(float(g('output_rate_hz').value), 1.0)
        telemetry_hz = max(float(g('telemetry_rate_hz').value), 1.0)
        self.create_timer(1.0 / output_hz, self._tick_commands)
        self.create_timer(1.0 / telemetry_hz, self._tick_telemetry)

    def _on_rx(self, msg: UInt8MultiArray) -> None:
        for frame in self.parser.feed(bytes(msg.data)):
            if frame.frame_type == 0x16:
                try:
                    self.last_channels = unpack_channels(frame.payload)
                    self.last_rx_time = time.monotonic()
                except ValueError:
                    pass

    def _on_fc_attitude(self, msg: Vector3) -> None:
        self.fc_attitude = (float(msg.x), float(msg.y), float(msg.z))
        self.fc_attitude_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _tick_commands(self) -> None:
        now = time.monotonic()
        fresh = (
            self.last_channels is not None
            and now - self.last_rx_time <= self.receiver_timeout
        )
        decoded = None
        if fresh:
            try:
                decoded = self.channel_map.decode_sticks(self.last_channels)
                if decoded.arm_request != self._last_arm_request:
                    self.get_logger().info(
                        f'CRSF arm request={decoded.arm_request} '
                        f'throttle={decoded.sticks.throttle:.3f}'
                    )
                    self._last_arm_request = decoded.arm_request
                enabled = self.arming_gate.update(
                    decoded.arm_request, decoded.sticks.throttle, now
                )
                if enabled != self._last_gate_armed:
                    self.get_logger().info(f'CRSF receiver armed={enabled}')
                    self._last_gate_armed = enabled
            except (ValueError, IndexError):
                enabled = False
        else:
            enabled = False
            self.arming_gate.reset()
            self._last_gate_armed = False
        if not enabled:
            command = Twist()
        else:
            _, command = self.channel_map.decode(self.last_channels, self.params)
        self.last_enabled = enabled
        self.pub_cmd.publish(command)
        manual = MultirotorManualCommand()
        manual.mode = self.manual_mode
        manual.active = enabled and decoded is not None and decoded.angle_mode
        if decoded is not None:
            manual.roll = float(decoded.sticks.roll)
            manual.pitch = float(decoded.sticks.pitch)
            manual.yaw = float(decoded.sticks.yaw)
            manual.throttle = float(decoded.sticks.throttle)
        self.pub_manual.publish(manual)
        enable = Bool()
        enable.data = enabled
        self.pub_enable.publish(enable)

    def _publish(self, data: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = list(data)
        self.pub_tx.publish(msg)

    def _tick_telemetry(self) -> None:
        odom = self.last_odom
        if odom is None:
            return
        roll, pitch, yaw = euler_from_quaternion(odom.pose.pose.orientation)
        if self.fc_attitude is not None and time.monotonic() - self.fc_attitude_time < 0.5:
            roll, pitch, yaw = self.fc_attitude
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        heading = math.degrees(math.atan2(velocity.y, velocity.x)) % 360.0 if speed > 1e-6 else math.degrees(yaw) % 360.0
        lat, lon = enu_to_lla(self.origin, float(position.x), float(position.y))
        frames = [
            make_attitude_frame(AttitudeTelemetry(roll, pitch, yaw)),
            make_gps_frame(GpsTelemetry(
                latitude_deg=lat,
                longitude_deg=lon,
                groundspeed_mps=speed,
                heading_deg=heading,
                altitude_m=self.origin_alt + float(position.z),
                satellites=12,
            )),
            *([make_barometer_frame(BarometerTelemetry(
                altitude_m=self.origin_alt + float(position.z),
                vertical_speed_mps=float(velocity.z),
            ))] if bool(self.get_parameter('send_barometer').value) else []),
            make_battery_frame(BatteryTelemetry(
                voltage_v=12.6,
                current_a=5.0,
                capacity_mah=round((100.0 - self.battery_percent) * 13.0),
                remaining_percent=round(self.battery_percent),
            )),
            make_flight_mode_frame(FlightModeTelemetry(
                'DISARMED' if not bool(getattr(self, 'last_enabled', False)) else 'ANGLE'
            )),
        ]
        self._publish(b''.join(frames))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrsfSimReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
