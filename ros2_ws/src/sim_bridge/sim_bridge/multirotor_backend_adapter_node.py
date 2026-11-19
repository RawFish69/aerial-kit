"""ROS 2 MuJoCo multirotor plant: quad / hex / octo from the shared airframe specs.

Drop-in for ``fastsim_backend_adapter_node`` (same contract):

    /uav/backend/cmd_twist   geometry_msgs/Twist   ENU velocity + angular.z yaw rate
    /uav/backend/enable      std_msgs/Bool
    /uav/backend/odom        nav_msgs/Odometry     pose, attitude, world velocity, body rates

plus a manual channel:

    /uav/backend/manual      uav_msgs/MultirotorManualCommand  (velocity / angle / acro sticks)

Each tick the control source is, in order: a fresh, active, finite manual
command (plant enabled); a fresh cmd_twist (plant enabled); otherwise the
safe fallback - hold position and heading in the air, motors idle only when
already resting on the ground. Every source ends in a z-up wrench that the
airframe's own allocator turns into rotor thrusts, and MuJoCo integrates the
rotor actuators; nothing sets the pose.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray
from uav_msgs.msg import MultirotorManualCommand

for _parent in Path(__file__).resolve().parents:
    if (_parent / 'aerial_kit').is_dir() and (_parent / 'pyproject.toml').is_file():
        if str(_parent) not in sys.path:
            sys.path.insert(0, str(_parent))
        break

from aerial_kit.airframes.multirotor_specs import available_airframes, load_multirotor_spec  # noqa: E402
from aerial_kit.controllers.cascade import CascadeGains  # noqa: E402
from aerial_kit.controllers.multirotor_manual import (  # noqa: E402
    MODES,
    ManualLimits,
    MultirotorPilot,
    StickInput,
)
from aerial_kit.types import ControlTarget, SimState  # noqa: E402
from sim_py.backends.mujoco_guard import SimulationDiverged  # noqa: E402
from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend  # noqa: E402

from sim_bridge.fixedwing_backend_adapter_node import (  # noqa: E402
    VIEWER_CAMERA_TOPIC,
    ViewerCameraSwitch,
    _as_bool,
    _finite,
    close_mujoco_viewer,
    open_mujoco_viewer,
)

SOURCE_MANUAL = 'manual'
SOURCE_TWIST = 'cmd_twist'
SOURCE_HOLD = 'hold'
SOURCE_LAND = 'auto_land'
SOURCE_IDLE = 'idle'

#: Below this height above the resting pose a disarmed aircraft cuts its
#: motors and settles; above it, it lands first. A real FC would simply stop
#: the motors on disarm; the simulator refuses to drop an aircraft from height.
IDLE_BELOW_AGL_M = 0.5
LAND_RATE_MPS = 0.7

MODE_NAMES = {
    MultirotorManualCommand.MODE_VELOCITY: 'velocity',
    MultirotorManualCommand.MODE_ANGLE: 'angle',
    MultirotorManualCommand.MODE_ACRO: 'acro',
}


def validate_multirotor_parameters(values):
    """Check launch-facing parameters before any plant or viewer exists."""
    out = dict(values)
    airframe = str(values.get('airframe', 'fpv_quad_5in'))
    if not (airframe in available_airframes() or Path(airframe).expanduser().is_file()):
        raise ValueError(
            f"airframe must be one of {', '.join(available_airframes())} or a YAML path, "
            f'got {airframe!r}')
    out['airframe'] = airframe
    out['viewer'] = _as_bool('viewer', values.get('viewer', False))
    fc = str(values.get('flight_controller', 'sim')).strip().lower()
    if fc not in ('sim', 'firmware'):
        raise ValueError(f"flight_controller must be 'sim' or 'firmware', got {values.get('flight_controller')!r}")
    out['flight_controller'] = fc
    for name in ('sim_rate_hz', 'cmd_timeout_sec', 'manual_timeout_sec', 'mujoco_timestep'):
        if name in values:
            out[name] = _finite(name, values[name], positive=True)
    for name in ('start_x_m', 'start_y_m', 'start_z_m', 'start_yaw_rad', 'wind_x_mps', 'wind_y_mps'):
        if name in values:
            out[name] = _finite(name, values[name])
    if out.get('start_z_m', 0.0) < 0.0:
        raise ValueError('start_z_m must be >= 0 (the ground is z = 0)')
    return out


def manual_from_msg(msg):
    """(mode name, StickInput) from a manual message; ValueError if unusable."""
    mode = MODE_NAMES.get(int(msg.mode))
    if mode is None:
        raise ValueError(f'unknown manual mode {msg.mode}')
    sticks = StickInput(float(msg.roll), float(msg.pitch), float(msg.yaw), float(msg.throttle))
    return mode, sticks.clipped()


def select_source(now, *, enabled, manual_active, manual_time, manual_timeout_s,
                  twist_time, twist_timeout_s, on_ground, height_agl_m):
    """Precedence and fallback.

    1. enabled + fresh active manual            -> manual
    2. enabled + fresh cmd_twist                -> cmd_twist
    3. enabled, but every command stale         -> hold (link loss: brake and hover)
    4. disabled, resting or below 0.5 m AGL     -> idle (motors off)
    5. disabled higher up                       -> auto_land, then idle
    """
    if (enabled and manual_active and manual_time is not None
            and 0.0 <= now - manual_time <= manual_timeout_s):
        return SOURCE_MANUAL
    if enabled and twist_time is not None and 0.0 <= now - twist_time <= twist_timeout_s:
        return SOURCE_TWIST
    if enabled:
        return SOURCE_HOLD
    if on_ground or height_agl_m < IDLE_BELOW_AGL_M:
        return SOURCE_IDLE
    return SOURCE_LAND


def yaw_quat(yaw):
    return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


class MultirotorBackendAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('multirotor_backend_adapter_node')
        for name, default in (
            ('airframe', 'fpv_quad_5in'),
            ('backend_cmd_topic', '/uav/backend/cmd_twist'),
            ('backend_enable_topic', '/uav/backend/enable'),
            ('backend_odom_topic', '/uav/backend/odom'),
            ('manual_topic', '/uav/backend/manual'),
            ('sim_rate_hz', 100.0),
            ('cmd_timeout_sec', 0.5),
            ('manual_timeout_sec', 0.30),
            ('mujoco_timestep', 0.002),
            ('start_x_m', 0.0),
            ('start_y_m', 0.0),
            ('start_z_m', 0.0),
            ('start_yaw_rad', 0.0),
            ('wind_x_mps', 0.0),
            ('wind_y_mps', 0.0),
            ('viewer', False),
            ('flight_controller', 'sim'),
            ('firmware_max_thrust_ratio', 2.0),
            # firmware only: where the RC comes from. 'teleop' = keyboard sticks
            # on manual_topic; 'crsf' = raw ELRS receiver bytes (the radio SIL).
            ('firmware_rc_input', 'teleop'),
            ('crsf_rx_topic', '/uav/radio/receiver_rx_bytes'),
            # firmware only: 1-based AUX that selects the FC's velocity hold (0 = none).
            ('firmware_velocity_channel', 0),
            # firmware only: the FC's own attitude estimate (roll, pitch, yaw in the
            # odometry's ENU/FLU convention) for the radio's attitude telemetry.
            ('firmware_attitude_topic', '/uav/radio/fc_attitude_rpy'),
            ('viewer_camera_topic', VIEWER_CAMERA_TOPIC),
        ):
            self.declare_parameter(name, default)
        launch = validate_multirotor_parameters({
            name: self.get_parameter(name).value for name in (
                'airframe', 'viewer', 'flight_controller', 'sim_rate_hz', 'cmd_timeout_sec', 'manual_timeout_sec',
                'mujoco_timestep', 'start_x_m', 'start_y_m', 'start_z_m', 'start_yaw_rad',
                'wind_x_mps', 'wind_y_mps')})
        self.launch = launch
        spec = load_multirotor_spec(launch['airframe'])
        self.spec = spec
        self.rest_z = self._rest_height()

        self.backend = MujocoMultirotorBackend()
        airborne = launch['start_z_m'] > 0.3
        self.backend.reset(
            SimState(
                position=np.array([launch['start_x_m'], launch['start_y_m'],
                                   launch['start_z_m'] if airborne else self.rest_z]),
                velocity=np.zeros(3), t=0.0,
                attitude_quat=yaw_quat(launch['start_yaw_rad']), body_rates=np.zeros(3)),
            {},
            {'simulation': {'mujoco_multirotor': {
                'airframe': launch['airframe'],
                'timestep': launch['mujoco_timestep'],
                'wind_mps': [launch['wind_x_mps'], launch['wind_y_mps'], 0.0],
                'start_motors': 'hover' if airborne else 'idle',
            }}})
        self.firmware = None
        if launch['flight_controller'] == 'firmware':
            # The AK flight core flies this airframe; the plant only does what
            # it does (its arming, failsafe and control law, nothing simulated).
            from sim_py.fc_sil import FirmwareInTheLoop, FirmwareRadio, hover_throttle

            if spec.n_rotors != 4:
                raise ValueError(f'flight_controller:=firmware drives quad-X only; {spec.name} has {spec.n_rotors} rotors')
            self.firmware = FirmwareInTheLoop(
                launch['airframe'], start_z=None if not airborne else launch['start_z_m'],
                wind_mps=(launch['wind_x_mps'], launch['wind_y_mps'], 0.0),
                velocity_channel=int(self.get_parameter('firmware_velocity_channel').value) or None)
            self.radio = FirmwareRadio(
                hover_throttle(spec),
                max_thrust_ratio=float(self.get_parameter('firmware_max_thrust_ratio').value))
            self.backend = self.firmware.backend
            self._warned = set()
            self.rc_input = str(self.get_parameter('firmware_rc_input').value).strip().lower()
            if self.rc_input not in ('teleop', 'crsf'):
                raise ValueError(f"firmware_rc_input must be 'teleop' or 'crsf', got {self.rc_input!r}")
            self._crsf_frames: list[tuple[float, list[int]]] = []
            self._last_tick_wall: float | None = None
            if self.rc_input == 'crsf':
                from aerial_kit.comms.crsf import CrsfParser

                self._crsf_parser = CrsfParser()
        self.pilot = MultirotorPilot(
            CascadeGains(**self.backend.controller_defaults('cascade')),
            ManualLimits.from_spec(spec))

        self.viewer = None
        span = 2.0 * (spec.arm_radius_m() + spec.prop_radius_m)
        self.camera_switch = ViewerCameraSwitch(distance=max(0.8, 5.0 * span))
        if launch['viewer']:
            self.viewer = open_mujoco_viewer(
                self.backend.model, self.backend.data, self.get_logger(),
                distance=self.camera_switch.distance)

        self.enabled = False
        self.twist = None
        self.twist_time = None
        self.manual = None
        self.manual_time = None
        self.source = None
        self._last_warn = -math.inf
        self.period = 1.0 / launch['sim_rate_hz']

        self.create_subscription(Twist, self.get_parameter('backend_cmd_topic').value, self._on_twist, 20)
        self.create_subscription(Bool, self.get_parameter('backend_enable_topic').value, self._on_enable, 20)
        self.create_subscription(MultirotorManualCommand, self.get_parameter('manual_topic').value,
                                 self._on_manual, 20)
        self.create_subscription(
            String, self.get_parameter('viewer_camera_topic').value,
            lambda msg: self.camera_switch.handle(self.viewer, self.backend.model, msg.data,
                                                  self.get_logger()), 10)
        if self.firmware is not None and self.rc_input == 'crsf':
            self.create_subscription(UInt8MultiArray, self.get_parameter('crsf_rx_topic').value,
                                     self._on_crsf_bytes, 50)
        self.pub_odom = self.create_publisher(Odometry, self.get_parameter('backend_odom_topic').value, 20)
        self.pub_fc_attitude = None
        if self.firmware is not None and self.get_parameter('firmware_attitude_topic').value:
            self.pub_fc_attitude = self.create_publisher(
                Vector3, self.get_parameter('firmware_attitude_topic').value, 20)
        self.timer = self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f'multirotor backend: mujoco {spec.name} ({spec.n_rotors} rotors, '
            f'{spec.total_mass_kg:.2f} kg, TWR {spec.thrust_to_weight():.1f}), '
            f"{'airborne' if airborne else 'on the ground'} at z={self.backend.state().position[2]:.2f} m")

    def _rest_height(self) -> float:
        """Root height when resting on the ground, found by letting it settle."""
        backend = MujocoMultirotorBackend()
        backend.reset(
            SimState(position=np.array([0.0, 0.0, 0.4]), velocity=np.zeros(3), t=0.0,
                     attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]), body_rates=np.zeros(3)),
            {}, {'simulation': {'mujoco_multirotor': {
                'airframe': self.launch['airframe'], 'start_motors': 'idle'}}})
        idle = ControlTarget(accel_cmd=np.zeros(3),
                             metadata={'rotor_thrust_n': np.zeros(self.spec.n_rotors)})
        for _ in range(150):
            backend.step(idle, 0.01)
        return float(backend.state().position[2])

    # ------------------------------------------------------------- inputs
    def _warn(self, text):
        now = time.monotonic()
        if now - self._last_warn > 2.0:
            self._last_warn = now
            self.get_logger().warning(text)

    def _on_twist(self, msg: Twist) -> None:
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
        if not all(math.isfinite(v) for v in values):
            self.twist = None
            self.twist_time = None
            self._warn('cmd_twist with a non-finite value refused')
            return
        self.twist = msg
        self.twist_time = time.monotonic()

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _on_manual(self, msg: MultirotorManualCommand) -> None:
        try:
            mode, sticks = manual_from_msg(msg)
        except ValueError as exc:
            self.manual = None
            self.manual_time = None
            self._warn(f'manual command refused: {exc}')
            return
        self.manual = (bool(msg.active), mode, sticks)
        self.manual_time = time.monotonic()

    # --------------------------------------------------------------- tick
    def control_source(self, now: float) -> str:
        return select_source(
            now, enabled=self.enabled,
            manual_active=self.manual is not None and self.manual[0],
            manual_time=self.manual_time,
            manual_timeout_s=self.launch['manual_timeout_sec'],
            twist_time=self.twist_time,
            twist_timeout_s=self.launch['cmd_timeout_sec'],
            on_ground=self.backend.on_ground(),
            height_agl_m=float(self.backend.state().position[2]) - self.rest_z)

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(text)

    def _on_crsf_bytes(self, msg: UInt8MultiArray) -> None:
        """ELRS receiver -> FC UART: keep RC frames with their arrival time."""
        from aerial_kit.comms.crsf import CRSF_FRAMETYPE_RC_CHANNELS_PACKED, unpack_channels

        now = time.monotonic()
        for frame in self._crsf_parser.feed(bytes(msg.data)):
            if frame.frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                self._crsf_frames.append((now, list(unpack_channels(frame.payload))))

    def _tick_firmware_crsf(self, now: float) -> None:
        """Step the firmware 1 ms at a time, delivering each frame when it arrived."""
        start = self._last_tick_wall if self._last_tick_wall is not None else now - self.period
        self._last_tick_wall = now
        frames, self._crsf_frames = self._crsf_frames, []
        total_ms = max(1, int(round(self.period * 1000.0)))
        done_ms = 0
        state_before = self.firmware.fc.state
        try:
            for arrived, channels in frames:
                at_ms = min(total_ms, max(0, int(round((arrived - start) * 1000.0))))
                if at_ms > done_ms:
                    self.firmware.step((at_ms - done_ms) / 1000.0)
                    done_ms = at_ms
                self.firmware.receive_frame(channels)
            if total_ms > done_ms:
                self.firmware.step((total_ms - done_ms) / 1000.0)
        except SimulationDiverged as exc:
            self.diverged = True
            self.get_logger().error(f'plant diverged, simulation frozen: {exc}')
        source = 'firmware_crsf'
        if source != self.source:
            self.get_logger().info(f'control source: {self.source} -> {source}')
            self.source = source
        if self.firmware.fc.state != state_before:
            self.get_logger().info(f'firmware: {state_before} -> {self.firmware.fc.state}')

    def _tick_firmware(self, now: float) -> None:
        if self.rc_input == 'crsf':
            if not getattr(self, 'diverged', False):
                self._tick_firmware_crsf(now)
            return
        """Teleop -> radio sticks -> AK flight core (1 kHz) -> MuJoCo rotors."""
        fresh = (self.manual is not None and self.manual_time is not None
                 and 0.0 <= now - self.manual_time <= self.launch['manual_timeout_sec'])
        if self.twist_time is not None:
            self._warn_once('twist', 'flight_controller:=firmware ignores cmd_twist: '
                                     'the AK firmware has no velocity mode')
        source = SOURCE_MANUAL if fresh and self.enabled and self.manual[0] else (
            'firmware_rc_lost' if not fresh else 'firmware_disarmed')
        if source != self.source:
            self.get_logger().info(f'control source: {self.source} -> {source}')
            self.source = source
        if fresh:
            active, mode, sticks = self.manual
            if mode == 'velocity':
                self._warn_once('velocity', 'firmware has no velocity mode; flying angle mode')
            self.firmware.set_sticks(**self.radio.sticks(
                self.firmware.t, self.enabled and active, mode,
                sticks.roll, sticks.pitch, sticks.yaw, sticks.throttle))
        self.firmware.rc_alive = fresh
        state_before = self.firmware.fc.state
        if not getattr(self, 'diverged', False):
            try:
                self.firmware.step(self.period)
            except SimulationDiverged as exc:
                self.diverged = True
                self.get_logger().error(f'plant diverged, simulation frozen: {exc}')
        if self.firmware.fc.state != state_before:
            self.get_logger().info(f'firmware: {state_before} -> {self.firmware.fc.state}')

    def _tick(self) -> None:
        now = time.monotonic()
        if self.firmware is not None:
            self._tick_firmware(now)
            self._sync_viewer()
            self._publish(self.backend.state())
            if self.pub_fc_attitude is not None:
                # Firmware attitude is FRD/NED (yaw clockwise from north); odometry
                # euler angles are FLU/ENU (yaw counter-clockwise from east).
                roll, pitch, yaw = (float(a) for a in self.firmware.fc.attitude())
                yaw_enu = math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))
                self.pub_fc_attitude.publish(Vector3(x=roll, y=-pitch, z=yaw_enu))
            return
        state = self.backend.state()
        source = self.control_source(now)
        if source != self.source:
            self.get_logger().info(f'control source: {self.source} -> {source}')
            if source in (SOURCE_HOLD, SOURCE_TWIST):
                self.pilot.reset_yaw(state)
            self.source = source
        dt = self.period
        if source == SOURCE_MANUAL:
            _, mode, sticks = self.manual
            target = ControlTarget(accel_cmd=np.zeros(3), wrench=self.pilot.stick_wrench(state, mode, sticks, dt))
        elif source == SOURCE_TWIST:
            t = self.twist
            target = ControlTarget(accel_cmd=np.zeros(3), wrench=self.pilot.velocity_wrench(
                state, [t.linear.x, t.linear.y, t.linear.z], float(t.angular.z), dt))
        elif source == SOURCE_HOLD:
            target = ControlTarget(accel_cmd=np.zeros(3), wrench=self.pilot.hold_wrench(state, dt))
        elif source == SOURCE_LAND:
            target = ControlTarget(accel_cmd=np.zeros(3), wrench=self.pilot.velocity_wrench(
                state, [0.0, 0.0, -LAND_RATE_MPS], 0.0, dt))
        else:
            target = ControlTarget(accel_cmd=np.zeros(3),
                                   metadata={'rotor_thrust_n': np.zeros(self.spec.n_rotors)})
        if not getattr(self, 'diverged', False):
            try:
                self.backend.step(target, dt)
            except SimulationDiverged as exc:
                # Freeze at the last finite state instead of letting MuJoCo
                # reset the aircraft to the origin; keep publishing it.
                self.diverged = True
                self.get_logger().error(f'plant diverged, simulation frozen: {exc}')
        self._sync_viewer()
        self._publish(self.backend.state())

    def _sync_viewer(self) -> None:
        if self.viewer is not None:
            try:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    self.viewer = None
            except Exception as exc:
                self.get_logger().warning(f'MuJoCo viewer stopped, continuing headless: {exc}')
                self.viewer = None

    def _publish(self, state: SimState) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        p, q, v, w = state.position, state.attitude_quat, state.velocity, state.body_rates
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = (float(x) for x in p)
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        # Same as fastsim: linear twist is the world-frame (ENU) velocity.
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = (float(x) for x in v)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = (float(x) for x in w)
        self.pub_odom.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MultirotorBackendAdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and node.viewer is not None:
            close_mujoco_viewer(node.viewer)
            node.viewer = None
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
