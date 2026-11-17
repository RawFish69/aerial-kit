import math

import numpy as np
import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from uav_msgs.msg import MultirotorManualCommand

import sim_bridge.multirotor_backend_adapter_node as adapter
from sim_bridge.multirotor_backend_adapter_node import (
    SOURCE_HOLD,
    SOURCE_IDLE,
    SOURCE_LAND,
    SOURCE_MANUAL,
    SOURCE_TWIST,
    manual_from_msg,
    select_source,
    validate_multirotor_parameters,
)


def _select(now, **overrides):
    values = dict(enabled=True, manual_active=True, manual_time=10.0, manual_timeout_s=0.3,
                  twist_time=10.0, twist_timeout_s=0.5, on_ground=False, height_agl_m=5.0)
    values.update(overrides)
    return select_source(now, **values)


def test_precedence_and_fallbacks():
    assert _select(10.1) == SOURCE_MANUAL
    assert _select(10.31) == SOURCE_TWIST                 # stale manual
    assert _select(10.1, manual_active=False) == SOURCE_TWIST
    assert _select(10.6) == SOURCE_HOLD                   # link loss in the air
    assert _select(10.1, enabled=False) == SOURCE_LAND    # disarmed up high: land, never drop
    assert _select(10.1, enabled=False, height_agl_m=0.2) == SOURCE_IDLE
    assert _select(10.1, enabled=False, on_ground=True) == SOURCE_IDLE


def test_parameters_are_validated():
    assert validate_multirotor_parameters({'airframe': 'research_hex', 'viewer': 'false'})['viewer'] is False
    for bad, match in (({'airframe': 'tricopter'}, 'airframe'),
                       ({'sim_rate_hz': 0.0}, 'positive'),
                       ({'start_z_m': -1.0}, '>= 0'),
                       ({'viewer': 'maybe'}, 'boolean'),
                       ({'wind_x_mps': math.nan}, 'finite')):
        with pytest.raises(ValueError, match=match):
            validate_multirotor_parameters({'airframe': 'fpv_quad_5in', **bad})


def _manual(mode=MultirotorManualCommand.MODE_ANGLE, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.5, active=True):
    msg = MultirotorManualCommand()
    msg.mode, msg.roll, msg.pitch, msg.yaw, msg.throttle, msg.active = mode, roll, pitch, yaw, throttle, active
    return msg


def test_manual_message_is_checked_and_clipped():
    mode, sticks = manual_from_msg(_manual(roll=3.0, throttle=-1.0))
    assert mode == 'angle' and sticks.roll == 1.0 and sticks.throttle == 0.0
    with pytest.raises(ValueError):
        manual_from_msg(_manual(pitch=math.nan))
    with pytest.raises(ValueError):
        manual_from_msg(_manual(mode=9))


def test_manual_message_layout():
    fields = MultirotorManualCommand.get_fields_and_field_types()
    assert list(fields) == ['header', 'sequence_id', 'mode', 'roll', 'pitch', 'yaw', 'throttle', 'active']
    assert (MultirotorManualCommand.MODE_VELOCITY, MultirotorManualCommand.MODE_ANGLE,
            MultirotorManualCommand.MODE_ACRO) == (0, 1, 2)


@pytest.fixture(params=['fpv_quad_5in', 'research_hex'])
def node(request):
    import rclpy

    rclpy.init(args=['--ros-args', '-p', f'airframe:={request.param}', '-p', 'start_z_m:=5.0'])
    n = adapter.MultirotorBackendAdapterNode()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _roll_deg(node):
    from aerial_kit.dynamics.rotations import quat_to_rotmat

    r = quat_to_rotmat(node.backend.state().attitude_quat)
    return math.degrees(math.atan2(r[2, 1], r[2, 2]))


def test_manual_angle_mode_banks_through_mujoco_rotors(node):
    from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

    assert isinstance(node.backend, MujocoMultirotorBackend)
    node._on_enable(Bool(data=True))
    for _ in range(80):
        node._on_manual(_manual(roll=1.0))
        node._tick()
    assert node.source == SOURCE_MANUAL
    limit = node.spec.control['manual']['max_angle_deg']
    assert _roll_deg(node) == pytest.approx(limit, abs=2.0)
    # Rotors on the left (+y) push harder to roll right.
    actual = node.backend.rotor_state()['actual_n']
    left = [actual[k] for k, r in enumerate(node.spec.rotors) if r.position_m[1] > 0.01]
    right = [actual[k] for k, r in enumerate(node.spec.rotors) if r.position_m[1] < -0.01]
    assert np.mean(left) > np.mean(right) or node.backend.state().velocity[1] < -0.5


def test_twist_flies_world_velocity(node):
    node._on_enable(Bool(data=True))
    cmd = Twist()
    cmd.linear.x = 2.0
    for _ in range(300):
        node._on_twist(cmd)
        node._tick()
    assert node.source == SOURCE_TWIST
    v = node.backend.state().velocity
    assert v[0] == pytest.approx(2.0, abs=0.2) and abs(v[1]) < 0.2 and abs(v[2]) < 0.2


def test_stale_manual_holds_and_disarm_lands(node, monkeypatch):
    clock = {'t': 1000.0}
    monkeypatch.setattr(adapter.time, 'monotonic', lambda: clock['t'])
    node._on_enable(Bool(data=True))
    node._on_manual(_manual(roll=1.0))
    node._tick()
    assert node.source == SOURCE_MANUAL
    for _ in range(100):
        clock['t'] += 0.01
        node._tick()
    assert node.source == SOURCE_HOLD
    assert np.linalg.norm(node.backend.state().velocity) < 1.5
    node._on_enable(Bool(data=False))
    z0 = node.backend.state().position[2]
    for _ in range(200):
        clock['t'] += 0.01
        node._tick()
    assert node.source in (SOURCE_LAND, SOURCE_IDLE)
    assert node.backend.state().position[2] < z0 - 0.8
    assert np.all(np.isfinite(node.backend.data.qpos))


def test_non_finite_inputs_are_refused(node):
    node._on_enable(Bool(data=True))
    node._on_manual(_manual())
    node._tick()
    assert node.source == SOURCE_MANUAL
    node._on_manual(_manual(throttle=math.inf))
    node._tick()
    assert node.source == SOURCE_HOLD
    bad = Twist()
    bad.linear.x = math.nan
    node._on_twist(bad)
    node._tick()
    assert node.source == SOURCE_HOLD


def test_rests_on_the_ground_when_started_there():
    import rclpy

    rclpy.init(args=['--ros-args', '-p', 'airframe:=micro_lr_quad'])
    n = adapter.MultirotorBackendAdapterNode()
    try:
        z0 = n.backend.state().position[2]
        for _ in range(100):
            n._tick()
        assert n.source == SOURCE_IDLE
        assert abs(n.backend.state().position[2] - z0) < 0.005
        assert n.backend.on_ground()
    finally:
        n.destroy_node()
        rclpy.shutdown()


# --- viewer camera toggle ------------------------------------------------------

import contextlib  # noqa: E402

from sim_bridge.fixedwing_backend_adapter_node import (  # noqa: E402
    ViewerCameraSwitch,
    next_view,
    onboard_camera_id,
    set_viewer_view,
)


class FakeViewer:
    def __init__(self):
        import mujoco

        self.cam = mujoco.MjvCamera()

    @contextlib.contextmanager
    def lock(self):
        yield


def _models():
    import mujoco

    from aerial_kit.airframes.multirotor_specs import available_airframes, load_multirotor_spec
    from aerial_kit.airframes.multirotor_specs.mjcf import multirotor_mjcf
    from aerial_kit.airframes.single_wing import load_single_wing_spec
    from aerial_kit.airframes.single_wing.mjcf import single_wing_mjcf
    from aerial_kit.airframes.twin_wing import load_twin_wing_spec
    from aerial_kit.airframes.twin_wing.mjcf import twin_wing_mjcf

    xmls = {name: multirotor_mjcf(load_multirotor_spec(name)) for name in available_airframes()}
    xmls['twin_wing'] = twin_wing_mjcf(load_twin_wing_spec())
    xmls['single_wing'] = single_wing_mjcf(load_single_wing_spec())
    return {name: mujoco.MjModel.from_xml_string(xml) for name, xml in xmls.items()}


def test_every_vehicle_has_an_onboard_camera():
    for name, model in _models().items():
        assert onboard_camera_id(model) >= 0, name


def test_next_view_toggles_and_selects():
    assert next_view('chase', 'next') == 'onboard'
    assert next_view('onboard', 'next') == 'chase'
    assert next_view('chase', 'ONBOARD') == 'onboard'
    with pytest.raises(ValueError):
        next_view('chase', 'cockpit')


def test_switch_applies_fixed_onboard_then_tracking_chase():
    import mujoco

    model = _models()['fpv_quad_5in']
    viewer = FakeViewer()
    switch = ViewerCameraSwitch(distance=1.5)
    assert switch.handle(viewer, model, 'next') == 'onboard'
    assert viewer.cam.type == mujoco.mjtCamera.mjCAMERA_FIXED
    assert viewer.cam.fixedcamid == onboard_camera_id(model)
    assert switch.handle(viewer, model, 'next') == 'chase'
    assert viewer.cam.type == mujoco.mjtCamera.mjCAMERA_TRACKING
    assert viewer.cam.distance == pytest.approx(1.5)
    # Bad requests and a closed viewer change nothing.
    assert switch.handle(viewer, model, 'sideways') == 'chase'
    assert switch.handle(None, model, 'next') == 'chase'


def test_model_without_onboard_camera_stays_on_chase():
    import mujoco

    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="body"><freejoint/>'
        '<geom size="0.1"/></body></worldbody></mujoco>')
    viewer = FakeViewer()
    assert set_viewer_view(viewer, model, 'onboard') == 'chase'
    assert viewer.cam.type == mujoco.mjtCamera.mjCAMERA_TRACKING
