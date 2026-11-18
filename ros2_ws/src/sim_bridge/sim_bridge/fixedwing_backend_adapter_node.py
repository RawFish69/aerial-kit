"""ROS 2 actuator-level headless backend for the Aerial Kit twin wing."""

from __future__ import annotations

import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from uav_msgs.msg import ActuatorCommand, AirspeedNav, Command, Telemetry

for _parent in Path(__file__).resolve().parents:
    if (_parent / 'aerial_kit').is_dir() and (_parent / 'pyproject.toml').is_file():
        if str(_parent) not in sys.path:
            sys.path.insert(0, str(_parent))
        break

from aerial_kit.airframes.fixed_wing import TwinWingAirframe
from aerial_kit.airframes.twin_wing import load_twin_wing_spec
from aerial_kit.controllers.fixed_wing import airspeed_nav_wrench
from aerial_kit.dynamics.fixed_wing import (
    FixedWingDynamics,
    level_attitude_quat,
)
from aerial_kit.estimation import SimulatedIns
from aerial_kit.types import ControlTarget, SimState
from sim_py.backends.fixedwing_backend import FixedWingBackend
from sim_py.backends.mujoco_fixedwing_backend import MujocoFixedWingBackend
from sim_py.backends.mujoco_guard import SimulationDiverged
from sim_py.backends.mujoco_single_wing_backend import MujocoSingleWingBackend
from sim_py.backends.single_wing_backend import SingleWingBackend
from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.airframes.single_wing_airframe import SingleWingAirframe
from aerial_kit.dynamics.single_wing import SingleWingDynamics


def state_for_control(estimator, estimated_state, truth_state):
    """Choose the state guidance sees without freezing the truth-backed path."""
    return estimated_state if estimator is not None else truth_state


def validate_estimator_mode(value):
    mode = str(value).lower()
    if mode not in ('truth', 'ekf'):
        raise ValueError(
            f"estimator_mode must be 'truth' or 'ekf', got {value!r}")
    return mode


BACKENDS = ('analytic', 'mujoco')
AIRFRAMES = ('twin_wing', 'single_wing')


def single_wing_manual_command(msg, spec):
    """[thrust, elevon_L, elevon_R] for the single-motor wing from ActuatorCommand.

    The single wing has one motor: its thrust is thrust_left_n +
    thrust_right_n (a twin-wing teleop sends half on each side). Elevons
    keep the message's physical left/right. Clamped to the spec's limits;
    ValueError for a non-finite value.
    """
    raw = np.array([msg.thrust_left_n + msg.thrust_right_n,
                    msg.elevon_left_rad, msg.elevon_right_rad], dtype=float)
    if not np.all(np.isfinite(raw)):
        raise ValueError('actuator command contains a non-finite value')
    out = raw.copy()
    out[0] = float(np.clip(out[0], 0.0, spec.max_thrust_n))
    out[1:] = np.clip(out[1:], -spec.max_elevon_rad, spec.max_elevon_rad)
    return out, bool(np.any(out != raw))

SOURCE_MANUAL = 'manual'
SOURCE_NAV = 'airspeed_nav'
SOURCE_FALLBACK = 'fallback'


def manual_actuator_from_msg(msg, spec, max_thrust_n):
    """Validate and clamp an ActuatorCommand into the backend's actuator order.

    Returns ``(command, clipped)``; raises ``ValueError`` for a non-finite
    value. Order and signs are physical and identical to the backend's
    ``[thrust_L, thrust_R, elevon_L, elevon_R]`` (TE-up positive).
    """
    raw = np.array([
        msg.thrust_left_n, msg.thrust_right_n,
        msg.elevon_left_rad, msg.elevon_right_rad,
    ], dtype=float)
    return spec.clamp_actuator_command(raw, max_thrust_n)


def select_control_source(now, *, enabled, manual_active, manual_time,
                          manual_timeout_s, nav_valid, nav_time,
                          nav_timeout_s):
    """Precedence: fresh active manual > fresh valid AirspeedNav > fallback.

    Freshness uses the adapter's receive time (monotonic), so a stalled or
    killed teleop expires after ``manual_timeout_s`` no matter what its
    header stamp claims.
    """
    if (enabled and manual_active and manual_time is not None
            and 0.0 <= now - manual_time <= manual_timeout_s):
        return SOURCE_MANUAL
    if (enabled and nav_valid and nav_time is not None
            and 0.0 <= now - nav_time <= nav_timeout_s):
        return SOURCE_NAV
    return SOURCE_FALLBACK

_TRUE = ('true', '1', 'yes', 'on')
_FALSE = ('false', '0', 'no', 'off', '')


def _as_bool(name, value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    raise ValueError(f'{name} must be a boolean, got {value!r}')


def _finite(name, value, *, positive=False, non_negative=False):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} must be a number, got {value!r}') from None
    if not math.isfinite(number):
        raise ValueError(f'{name} must be finite, got {value!r}')
    if positive and number <= 0.0:
        raise ValueError(f'{name} must be positive, got {value!r}')
    if non_negative and number < 0.0:
        raise ValueError(f'{name} must be non-negative, got {value!r}')
    return number


def validate_launch_parameters(values):
    """Check every launch-facing parameter; raise before anything is built.

    Runs before the plant, the estimator or any viewer/OpenGL context exists,
    so a typo such as ``estimator_mode:=ek`` or ``backend:=mujuco`` fails
    with a clear message instead of after a window has opened.
    """
    out = dict(values)
    backend = str(values.get('backend', 'analytic')).strip().lower()
    if backend not in BACKENDS:
        raise ValueError(
            f"backend must be 'analytic' or 'mujoco', got {values.get('backend')!r}")
    out['backend'] = backend
    airframe = str(values.get('airframe', 'twin_wing')).strip().lower()
    if airframe not in AIRFRAMES:
        raise ValueError(
            f"airframe must be 'twin_wing' or 'single_wing', got {values.get('airframe')!r}")
    out['airframe'] = airframe
    out['estimator_mode'] = validate_estimator_mode(
        values.get('estimator_mode', 'truth'))
    out['mujoco_viewer'] = _as_bool(
        'mujoco_viewer', values.get('mujoco_viewer', False))
    for name in ('sim_rate_hz', 'nav_timeout_sec', 'manual_timeout_sec',
                 'max_thrust_n',
                 'cruise_airspeed_mps', 'mujoco_timestep'):
        if name in values:
            out[name] = _finite(name, values[name], positive=True)
    for name in ('mujoco_thrust_tau_s',):
        if name in values:
            out[name] = _finite(name, values[name], non_negative=True)
    for name in ('wind_x_mps', 'wind_y_mps', 'wind_z_mps', 'start_x_m',
                 'start_y_m', 'start_z_m', 'start_speed_mps',
                 'start_heading_rad'):
        if name in values:
            out[name] = _finite(name, values[name])
    if 'estimator_seed' in values:
        try:
            out['estimator_seed'] = int(values['estimator_seed'])
        except (TypeError, ValueError):
            raise ValueError(
                f"estimator_seed must be an integer, got {values['estimator_seed']!r}"
            ) from None
    model_path = str(values.get('mujoco_model_path', '') or '')
    if model_path and not Path(model_path).expanduser().is_file():
        raise ValueError(f'mujoco_model_path does not exist: {model_path}')
    spec_path = str(values.get('airframe_spec', '') or '')
    if spec_path and not Path(spec_path).expanduser().is_file():
        raise ValueError(f'airframe_spec does not exist: {spec_path}')
    return out


_VIEWER_PROBE = r"""
import sys
import glfw
if not glfw.init():
    sys.exit(2)
glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
window = glfw.create_window(64, 64, 'aerial-kit-probe', None, None)
if window is None:
    sys.exit(3)
glfw.make_context_current(window)
glfw.destroy_window(window)
glfw.terminate()
"""


def viewer_unavailable_reason(env=None, timeout_s=15.0, runner=subprocess.run):
    """Return None when a GLFW window can be opened, else a reason string.

    The probe runs in a child process: a broken X/GLX/driver stack can abort
    the process that touches it, and that must never be the simulation.
    """
    env = dict(os.environ if env is None else env)
    if sys.platform.startswith('linux') and not (
            env.get('DISPLAY') or env.get('WAYLAND_DISPLAY')):
        return 'no DISPLAY or WAYLAND_DISPLAY is set'
    if env.get('MUJOCO_GL', '').lower() in ('egl', 'osmesa'):
        return f"MUJOCO_GL={env['MUJOCO_GL']} is headless-only"
    try:
        result = runner(
            [sys.executable, '-c', _VIEWER_PROBE], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            timeout=timeout_s, check=False)
    except Exception as exc:  # timeout, missing interpreter, ...
        return f'OpenGL probe failed: {exc}'
    if result.returncode != 0:
        detail = (result.stderr or b'').decode(errors='replace').strip()
        tail = detail.splitlines()[-1] if detail else ''
        return f'OpenGL probe exited with {result.returncode} {tail}'.strip()
    return None


def open_mujoco_viewer(model, data, logger, distance=2.6):
    """Open the passive viewer with a chase camera, or return None.

    Every failure is logged and swallowed: the headless plant keeps running.
    """
    reason = viewer_unavailable_reason()
    if reason is not None:
        logger.warning(f'MuJoCo viewer disabled, continuing headless: {reason}')
        return None
    try:
        import mujoco
        import mujoco.viewer

        before = set(threading.enumerate())
        viewer = mujoco.viewer.launch_passive(
            model, data, show_left_ui=False, show_right_ui=False)
        # launch_passive does not expose its render thread; remember it so
        # shutdown can wait for it (see close_mujoco_viewer).
        viewer_threads = [t for t in threading.enumerate() if t not in before]
        set_viewer_view(viewer, model, 'chase', distance)
    except Exception as exc:
        logger.warning(f'MuJoCo viewer failed, continuing headless: {exc}')
        return None
    logger.info('MuJoCo viewer open (chase camera on the airframe)')
    _VIEWER_THREADS[id(viewer)] = viewer_threads
    return viewer


_VIEWER_THREADS = {}

VIEWER_CAMERA_TOPIC = '/uav/sim/viewer_camera'
#: On-board cameras, by preference: FPV nose/front camera, else a payload camera.
ONBOARD_CAMERAS = ('fpv', 'payload_cam', 'nav_cam')
VIEWS = ('chase', 'onboard')


def onboard_camera_id(model):
    """Id of the model's on-board camera, or -1 if it has none."""
    import mujoco

    for name in ONBOARD_CAMERAS:
        cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if cam >= 0:
            return int(cam)
    return -1


def set_viewer_view(viewer, model, view, distance=2.6):
    """Point the passive viewer at 'chase' (tracking) or 'onboard' (fixed camera).

    Returns the view actually applied: a model with no on-board camera stays
    on the chase view.
    """
    import mujoco

    cam_id = onboard_camera_id(model) if view == 'onboard' else -1
    with viewer.lock():
        if cam_id >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam_id
            return 'onboard'
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'body')
        viewer.cam.distance = float(distance)
        viewer.cam.azimuth = 200.0
        viewer.cam.elevation = -15.0
    return 'chase'


def next_view(current, request):
    """'next' toggles chase <-> onboard; 'chase' / 'onboard' select directly."""
    request = str(request).strip().lower()
    if request == 'next':
        return VIEWS[(VIEWS.index(current) + 1) % len(VIEWS)] if current in VIEWS else 'chase'
    if request in VIEWS:
        return request
    raise ValueError(f"viewer camera request must be 'next', 'chase' or 'onboard', got {request!r}")


class ViewerCameraSwitch:
    """Applies viewer-camera requests (std_msgs/String) to an open viewer."""

    def __init__(self, distance=2.6):
        self.distance = float(distance)
        self.view = 'chase'

    def handle(self, viewer, model, request, logger=None):
        try:
            wanted = next_view(self.view, request)
        except ValueError as exc:
            if logger is not None:
                logger.warning(str(exc))
            return self.view
        if viewer is None:
            return self.view
        applied = set_viewer_view(viewer, model, wanted, self.distance)
        if logger is not None and applied != self.view:
            note = '' if applied == wanted else ' (this model has no on-board camera)'
            logger.info(f'viewer camera: {applied}{note}')
        self.view = applied
        return applied


def close_mujoco_viewer(viewer, timeout_s=3.0):
    """Close the passive viewer and wait for its render thread to stop.

    ``close()`` only asks the viewer thread to exit. Letting the process
    tear down (freeing the model/data and GL context) while that thread is
    still rendering segfaults at exit, so wait for it first.
    """
    threads = _VIEWER_THREADS.pop(id(viewer), [])
    try:
        viewer.close()
    except Exception:
        pass
    deadline = time.monotonic() + timeout_s
    for thread in threads:
        thread.join(max(deadline - time.monotonic(), 0.0))


class FixedWingBackendAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('fixedwing_backend_adapter_node')
        self.declare_parameter('airspeed_nav_topic', '/uav/backend/cmd_airspeed_nav')
        self.declare_parameter('enable_topic', '/uav/backend/enable')
        self.declare_parameter('telemetry_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('backend', 'analytic')
        self.declare_parameter('sim_rate_hz', 100.0)
        self.declare_parameter('nav_timeout_sec', 1.0)
        self.declare_parameter('manual_command_topic', '/uav/backend/cmd_actuators')
        self.declare_parameter('manual_timeout_sec', 0.30)
        self.declare_parameter('start_x_m', 0.0)
        self.declare_parameter('start_y_m', 0.0)
        self.declare_parameter('start_z_m', 20.0)
        self.declare_parameter('start_speed_mps', 15.0)
        self.declare_parameter('start_heading_rad', 0.0)
        self.declare_parameter('cruise_airspeed_mps', 15.0)
        self.declare_parameter('trim_throttle_n', 3.0)
        self.declare_parameter('max_thrust_n', 10.0)
        self.declare_parameter('tecs_kp_throttle', 0.15)
        self.declare_parameter('climb_rate_kp', 0.25)
        self.declare_parameter('kp_roll', 0.6)
        self.declare_parameter('kd_roll', 0.15)
        self.declare_parameter('kp_pitch', 0.8)
        self.declare_parameter('kd_pitch', 0.2)
        self.declare_parameter('kp_yaw_rate', 2.0)
        self.declare_parameter('wind_x_mps', 0.0)
        self.declare_parameter('wind_y_mps', 0.0)
        self.declare_parameter('wind_z_mps', 0.0)
        self.declare_parameter('mujoco_timestep', 0.002)
        self.declare_parameter('mujoco_thrust_tau_s', 0.03)
        self.declare_parameter('mujoco_model_path', '')
        self.declare_parameter('airframe_spec', '')
        self.declare_parameter('airframe', 'twin_wing')
        self.declare_parameter('viewer_camera_topic', VIEWER_CAMERA_TOPIC)
        self.declare_parameter('mujoco_viewer', False)
        self.declare_parameter('estimator_mode', 'truth')
        self.declare_parameter('estimator_seed', 7)
        self.declare_parameter('gps_sigma_xy_m', 0.35)
        self.declare_parameter('gps_sigma_z_m', 0.6)
        self.declare_parameter('baro_sigma_m', 0.15)
        self.declare_parameter('accel_sigma_mps2', 0.03)

        # Validate everything launch-facing first: no plant, estimator or
        # viewer/OpenGL context exists until this has passed.
        launch = validate_launch_parameters({
            name: self.get_parameter(name).value
            for name in (
                'backend', 'estimator_mode', 'estimator_seed', 'mujoco_viewer',
                'sim_rate_hz', 'nav_timeout_sec', 'manual_timeout_sec',
                'max_thrust_n', 'cruise_airspeed_mps', 'mujoco_timestep',
                'mujoco_thrust_tau_s', 'wind_x_mps', 'wind_y_mps',
                'wind_z_mps', 'start_x_m', 'start_y_m', 'start_z_m',
                'start_speed_mps', 'start_heading_rad', 'mujoco_model_path',
                'airframe_spec', 'airframe')
        })
        self.single = launch['airframe'] == 'single_wing'
        if self.single:
            self.spec = load_single_wing_spec(launch['airframe_spec'] or None)
            self.params = self.spec.dynamics_params()
        else:
            self.spec = load_twin_wing_spec(launch['airframe_spec'] or None)
            self.params = self.spec.fixed_wing_params()
        position = np.array([
            float(self.get_parameter('start_x_m').value),
            float(self.get_parameter('start_y_m').value),
            float(self.get_parameter('start_z_m').value),
        ])
        heading = float(self.get_parameter('start_heading_rad').value)
        speed = float(self.get_parameter('start_speed_mps').value)
        velocity = np.array([
            speed * np.cos(heading), speed * np.sin(heading), 0.0])
        initial_state = SimState(
            position=position,
            velocity=velocity,
            t=0.0,
            attitude_quat=level_attitude_quat(heading),
            body_rates=np.zeros(3),
        )
        cruise = float(self.get_parameter('cruise_airspeed_mps').value)
        if self.single:
            # SingleWingAirframe.allocate adds its own trim (common elevon and
            # motor-reaction compensation) and drops yaw: nothing to add here.
            self.airframe = SingleWingAirframe(self.spec)
            trim = SingleWingDynamics(self.params).compute_trim_detailed(cruise)
            self.trim_command = np.array([trim['thrust_n'], trim['elevon_left_rad'], trim['elevon_right_rad']])
            self.trim_elevon_rad = 0.0
            self.max_thrust_n = min(launch['max_thrust_n'], self.spec.max_thrust_n)
        else:
            self.airframe = TwinWingAirframe(
                mass_kg=self.params.mass_kg,
                motor_separation_m=self.params.motor_separation_m,
                max_elevon_rad=self.spec.max_elevon_rad,
                max_thrust_n=float(self.get_parameter('max_thrust_n').value),
                trim_throttle_n=float(self.get_parameter('trim_throttle_n').value),
            )
            self.trim_command = FixedWingDynamics(self.params).compute_trim(cruise)
            self.trim_elevon_rad = float(self.trim_command[2])
        self.cfg = {
            'controller': {
                'l1_tecs': {
                    'cruise_airspeed_mps': float(self.get_parameter('cruise_airspeed_mps').value),
                    # The backend knows the plant. Use the plant's own
                    # level-flight trim rather than the generic controller
                    # default, or the ROS demo introduces a thrust step and
                    # climbs until its speed/altitude error happens to balance.
                    'trim_throttle_n': float(self.trim_command[0]),
                    'max_thrust_n': (self.spec.max_thrust_n if self.single
                                     else float(self.get_parameter('max_thrust_n').value)),
                    'tecs_kp_throttle': float(self.get_parameter('tecs_kp_throttle').value),
                    'climb_rate_kp': float(self.get_parameter('climb_rate_kp').value),
                    'kp_roll': float(self.get_parameter('kp_roll').value),
                    'kd_roll': float(self.get_parameter('kd_roll').value),
                    'kp_pitch': float(self.get_parameter('kp_pitch').value),
                    'kd_pitch': float(self.get_parameter('kd_pitch').value),
                    'kp_yaw_rate': float(self.get_parameter('kp_yaw_rate').value),
                }
            }
        }
        runtime_cfg = {
            'simulation': {
                'fixedwing': {
                    'wind_mps': [
                        float(self.get_parameter('wind_x_mps').value),
                        float(self.get_parameter('wind_y_mps').value),
                        float(self.get_parameter('wind_z_mps').value),
                    ],
                    # Airframe numbers come from the shared spec, so the
                    # analytic and MuJoCo plants see the same aircraft.
                    **({} if self.single else self.spec.fixed_wing_param_overrides()),
                    'max_thrust_n': float(
                        self.get_parameter('max_thrust_n').value),
                },
                'mujoco': {
                    'timestep': float(
                        self.get_parameter('mujoco_timestep').value),
                    'thrust_tau_s': float(
                        self.get_parameter('mujoco_thrust_tau_s').value),
                    'model_path': str(
                        self.get_parameter('mujoco_model_path').value),
                    'airframe_spec': launch['airframe_spec'],
                },
            }
        }
        if self.single:
            runtime_cfg['simulation'] = {
                'singlewing': {
                    'wind_mps': runtime_cfg['simulation']['fixedwing']['wind_mps'],
                    'airframe_spec': launch['airframe_spec'],
                },
                'mujoco': {'timestep': runtime_cfg['simulation']['mujoco']['timestep']},
            }
        backend_kind = launch['backend']
        if self.single:
            self.backend = MujocoSingleWingBackend() if backend_kind == 'mujoco' else SingleWingBackend()
        elif backend_kind == 'mujoco':
            self.backend = MujocoFixedWingBackend()
        else:
            self.backend = FixedWingBackend()
        self.viewer = None
        estimator_mode = launch['estimator_mode']
        self.estimator = None
        self.estimated_state = initial_state
        if estimator_mode == 'ekf':
            self.estimator = SimulatedIns.from_config({
                'seed': int(self.get_parameter('estimator_seed').value),
                'gps_sigma_xy_m': float(
                    self.get_parameter('gps_sigma_xy_m').value),
                'gps_sigma_z_m': float(
                    self.get_parameter('gps_sigma_z_m').value),
                'baro_sigma_m': float(
                    self.get_parameter('baro_sigma_m').value),
                'accel_sigma_mps2': float(
                    self.get_parameter('accel_sigma_mps2').value),
            })
            self.estimated_state = self.estimator.reset(initial_state)
        self.backend.reset(initial_state, {}, runtime_cfg)
        if launch['mujoco_viewer']:
            if backend_kind == 'mujoco':
                self.viewer = open_mujoco_viewer(
                    self.backend.model, self.backend.data, self.get_logger())
            else:
                self.get_logger().warning(
                    'mujoco_viewer ignored: backend is analytic')
        self.enabled = False
        self.nav = None
        self.nav_time = None
        self.manual = None
        self.manual_time = None
        self.manual_timeout_s = launch['manual_timeout_sec']
        self.nav_timeout_s = launch['nav_timeout_sec']
        self.max_thrust_n = launch['max_thrust_n']
        self.source = SOURCE_FALLBACK
        self._last_manual_warning = -math.inf
        self.steps = 0

        self.create_subscription(
            AirspeedNav, self.get_parameter('airspeed_nav_topic').value,
            self._on_nav, 20)
        self.create_subscription(
            Bool, self.get_parameter('enable_topic').value,
            self._on_enable, 20)
        self.create_subscription(
            ActuatorCommand, self.get_parameter('manual_command_topic').value,
            self._on_manual, 20)
        self.camera_switch = ViewerCameraSwitch()
        self.create_subscription(
            String, self.get_parameter('viewer_camera_topic').value,
            lambda msg: self.camera_switch.handle(
                self.viewer, self.backend.model if self.viewer is not None else None,
                msg.data, self.get_logger()), 10)
        self.pub = self.create_publisher(
            Telemetry, self.get_parameter('telemetry_topic').value, 20)
        rate = max(float(self.get_parameter('sim_rate_hz').value), 1.0)
        self.period = 1.0 / rate
        self.timer = self.create_timer(self.period, self._tick)
        initial = self.backend.state()
        self.get_logger().info(
            f"fixed-wing backend: {backend_kind}, airframe={'single_wing' if self.single else 'twin_wing'}, "
            f'z={initial.position[2]:.1f} m, '
            f'airspeed={np.linalg.norm(initial.velocity):.1f} m/s, '
            f'estimator={estimator_mode}')

    def _on_nav(self, msg: AirspeedNav) -> None:
        self.nav = msg
        self.nav_time = time.monotonic()

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _warn_manual(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_manual_warning > 2.0:
            self._last_manual_warning = now
            self.get_logger().warning(text)

    def _on_manual(self, msg: ActuatorCommand) -> None:
        try:
            if self.single:
                command, clipped = single_wing_manual_command(msg, self.spec)
            else:
                command, clipped = manual_actuator_from_msg(
                    msg, self.spec, self.max_thrust_n)
        except ValueError as exc:
            # Refuse it and drop any earlier manual command too: the source
            # is misbehaving, so the plant should not keep flying its last
            # good command until the timeout.
            self.manual = None
            self.manual_time = None
            self._warn_manual(f'manual actuator command refused: {exc}')
            return
        if clipped and msg.active:
            self._warn_manual('manual actuator command clamped to airframe limits')
        self.manual = (bool(msg.active), command)
        self.manual_time = time.monotonic()

    def control_source(self, now: float) -> str:
        return select_control_source(
            now,
            enabled=self.enabled,
            manual_active=self.manual is not None and self.manual[0],
            manual_time=self.manual_time,
            manual_timeout_s=self.manual_timeout_s,
            nav_valid=self.nav is not None and bool(self.nav.valid),
            nav_time=self.nav_time,
            nav_timeout_s=self.nav_timeout_s,
        )

    def _tick(self) -> None:
        if not rclpy.ok():
            return
        now = time.monotonic()
        truth_state = self.backend.state()
        state = state_for_control(
            self.estimator, self.estimated_state, truth_state)
        source = self.control_source(now)
        if source != self.source:
            self.get_logger().info(f'control source: {self.source} -> {source}')
            self.source = source
        if source == SOURCE_MANUAL:
            # Already validated and clamped to the spec's limits; the backend
            # applies them through MuJoCo's servos, prop motors and the aero
            # model like any other actuator command.
            actuator = self.manual[1].copy()
        else:
            if source == SOURCE_NAV:
                wrench = airspeed_nav_wrench(
                    state, float(self.nav.airspeed_mps),
                    float(self.nav.bank_rad), float(self.nav.climb_rate_mps),
                    self.cfg)
            else:
                # A wing cannot stop. Losing guidance (or manual control)
                # leaves it in a shallow circle at cruise, matching the
                # aircraft's own no-link behaviour.
                wrench = airspeed_nav_wrench(
                    state,
                    float(self.get_parameter('cruise_airspeed_mps').value),
                    np.radians(25.0), 0.0, self.cfg)
            wrench.moment_body[1] += self.trim_elevon_rad
            actuator = self.airframe.allocate(wrench, state)
        if not getattr(self, 'diverged', False):
            try:
                self.backend.step(
                    ControlTarget(
                        accel_cmd=np.zeros(3),
                        metadata={'actuator_cmd': actuator},
                    ),
                    self.period,
                )
            except SimulationDiverged as exc:
                # Freeze at the last finite state instead of letting MuJoCo
                # reset the aircraft to the origin; keep publishing it.
                self.diverged = True
                self.get_logger().error(f'plant diverged, simulation frozen: {exc}')
        truth_state = self.backend.state()
        if self.estimator is not None:
            self.estimated_state = self.estimator.update(truth_state)
            state = self.estimated_state
        else:
            self.estimated_state = truth_state
            state = self.estimated_state
        if self.viewer is not None:
            try:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    self.get_logger().info('MuJoCo viewer closed; plant continues')
                    self.viewer = None
            except Exception as exc:
                self.get_logger().warning(
                    f'MuJoCo viewer stopped, continuing headless: {exc}')
                self.viewer = None
        if state.position[2] < 0.0:
            state.position[2] = 0.0
            state.velocity[2] = max(0.0, state.velocity[2])
        self.steps += 1
        self._publish(state)

    def _state(self):
        return self.backend.state()

    def _publish(self, state: SimState) -> None:
        msg = Telemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(state.position[0])
        msg.pose.position.y = float(state.position[1])
        msg.pose.position.z = float(state.position[2])
        q = state.attitude_quat
        msg.pose.orientation.w = float(q[0])
        msg.pose.orientation.x = float(q[1])
        msg.pose.orientation.y = float(q[2])
        msg.pose.orientation.z = float(q[3])
        msg.twist.linear.x = float(state.velocity[0])
        msg.twist.linear.y = float(state.velocity[1])
        msg.twist.linear.z = float(state.velocity[2])
        msg.twist.angular.x = float(state.body_rates[0])
        msg.twist.angular.y = float(state.body_rates[1])
        msg.twist.angular.z = float(state.body_rates[2])
        msg.airspeed_mps = float(
            state.airspeed_mps
            if state.airspeed_mps is not None
            else np.linalg.norm(state.velocity))
        msg.armed = self.enabled
        if self.source == SOURCE_MANUAL:
            msg.current_mode = Command.MODE_MANUAL_OVERRIDE
        else:
            msg.current_mode = (
                Command.MODE_MISSION if self.enabled else Command.MODE_IDLE)
        msg.manual_override_active = self.source == SOURCE_MANUAL
        msg.status_text = f'fixed-wing backend: {self.source}'
        try:
            self.pub.publish(msg)
        except Exception:
            # A shutdown can invalidate the context between the timer
            # callback starting and this publish. That is a normal teardown
            # race, not a controller fault.
            if not rclpy.ok():
                return
            raise


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FixedWingBackendAdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C or the launch shutting the session down: a normal exit.
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None and getattr(node, 'viewer', None) is not None:
            close_mujoco_viewer(node.viewer)
            node.viewer = None
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
