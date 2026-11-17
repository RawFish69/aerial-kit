import sys

import pytest

from sim_bridge.fixedwing_backend_adapter_node import (
    state_for_control,
    validate_estimator_mode,
)


def test_truth_backend_uses_current_plant_state_not_the_initial_snapshot():
    initial = object()
    current = object()
    assert state_for_control(None, initial, current) is current


def test_ekf_backend_uses_the_estimated_state():
    estimated = object()
    truth = object()
    assert state_for_control(object(), estimated, truth) is estimated


def test_estimator_mode_requires_an_exact_supported_value():
    assert validate_estimator_mode('truth') == 'truth'
    assert validate_estimator_mode('EKF') == 'ekf'
    with pytest.raises(ValueError, match="'truth' or 'ekf'"):
        validate_estimator_mode('ek')


# --- launch validation happens before any plant or viewer exists -----------

import subprocess  # noqa: E402

import sim_bridge.fixedwing_backend_adapter_node as adapter  # noqa: E402
from sim_bridge.fixedwing_backend_adapter_node import (  # noqa: E402
    validate_launch_parameters,
    viewer_unavailable_reason,
)


GOOD = {
    'backend': 'mujoco', 'estimator_mode': 'ekf', 'estimator_seed': 7,
    'mujoco_viewer': 'true', 'sim_rate_hz': 100.0, 'nav_timeout_sec': 1.0,
    'max_thrust_n': 10.0, 'cruise_airspeed_mps': 15.0,
    'mujoco_timestep': 0.002, 'mujoco_thrust_tau_s': 0.03,
    'wind_x_mps': 3.0, 'wind_y_mps': 1.0, 'wind_z_mps': 0.0,
    'mujoco_model_path': '', 'airframe_spec': '',
}


def test_launch_parameters_normalise():
    out = validate_launch_parameters(GOOD)
    assert out['backend'] == 'mujoco'
    assert out['estimator_mode'] == 'ekf'
    assert out['mujoco_viewer'] is True
    assert validate_launch_parameters(
        {**GOOD, 'backend': 'Analytic', 'mujoco_viewer': False}
    )['backend'] == 'analytic'


@pytest.mark.parametrize('name,value,match', [
    ('backend', 'mujuco', "'analytic' or 'mujoco'"),
    ('estimator_mode', 'ek', "'truth' or 'ekf'"),
    ('mujoco_viewer', 'maybe', 'boolean'),
    ('sim_rate_hz', 0.0, 'positive'),
    ('mujoco_timestep', float('nan'), 'finite'),
    ('mujoco_thrust_tau_s', -0.1, 'non-negative'),
    ('wind_x_mps', 'fast', 'number'),
    ('estimator_seed', 'seven', 'integer'),
    ('mujoco_model_path', '/nonexistent/model.xml', 'does not exist'),
    ('airframe_spec', '/nonexistent/wing.yaml', 'does not exist'),
])
def test_bad_launch_parameters_are_rejected(name, value, match):
    with pytest.raises(ValueError, match=match):
        validate_launch_parameters({**GOOD, name: value})


@pytest.mark.parametrize('overrides', [
    {'estimator_mode': 'ek'},
    {'backend': 'mujuco'},
])
def test_node_rejects_bad_values_before_creating_a_viewer(monkeypatch, overrides):
    import rclpy

    opened = []
    monkeypatch.setattr(
        adapter, 'open_mujoco_viewer',
        lambda *args, **kwargs: opened.append(args) or None)
    built = []
    monkeypatch.setattr(
        adapter, 'MujocoFixedWingBackend',
        lambda: built.append(1) or pytest.fail('plant built before validation'))
    params = {'backend': 'mujoco', 'mujoco_viewer': 'true', **overrides}
    args = ['--ros-args'] + [
        item for key, value in params.items() for item in ('-p', f'{key}:={value}')]
    rclpy.init(args=args)
    try:
        with pytest.raises(ValueError):
            adapter.FixedWingBackendAdapterNode()
    finally:
        rclpy.shutdown()
    assert opened == []
    assert built == []


def test_viewer_probe_reports_a_missing_display_without_raising():
    reason = viewer_unavailable_reason(env={'PATH': '/usr/bin'})
    if sys.platform.startswith('linux'):
        assert 'DISPLAY' in reason


def test_viewer_probe_reports_headless_mujoco_gl():
    reason = viewer_unavailable_reason(env={'DISPLAY': ':0', 'MUJOCO_GL': 'egl'})
    assert 'MUJOCO_GL' in reason


def test_viewer_probe_survives_a_crashing_gl_stack():
    def crashing(*args, **kwargs):
        return subprocess.CompletedProcess(args, -6, b'', b'GLX: BadValue\n')

    reason = viewer_unavailable_reason(env={'DISPLAY': ':0'}, runner=crashing)
    assert '-6' in reason and 'GLX' in reason


def test_viewer_probe_survives_a_hung_gl_stack():
    def hanging(*args, **kwargs):
        raise subprocess.TimeoutExpired(args, 1.0)

    reason = viewer_unavailable_reason(env={'DISPLAY': ':0'}, runner=hanging)
    assert 'probe failed' in reason


def test_open_viewer_returns_none_when_unavailable(monkeypatch):
    warnings = []

    class Logger:
        def warning(self, msg):
            warnings.append(msg)

        def info(self, msg):
            pass

    monkeypatch.setattr(adapter, 'viewer_unavailable_reason', lambda: 'no display')
    assert adapter.open_mujoco_viewer(object(), object(), Logger()) is None
    assert warnings and 'headless' in warnings[0]


# --- manual actuator control -------------------------------------------------

import math  # noqa: E402

import numpy as np  # noqa: E402
from uav_msgs.msg import ActuatorCommand  # noqa: E402

from sim_bridge.fixedwing_backend_adapter_node import (  # noqa: E402
    SOURCE_FALLBACK,
    SOURCE_MANUAL,
    SOURCE_NAV,
    manual_actuator_from_msg,
    select_control_source,
)


def _select(now, **overrides):
    values = dict(enabled=True, manual_active=True, manual_time=10.0,
                  manual_timeout_s=0.3, nav_valid=True, nav_time=10.0,
                  nav_timeout_s=1.0)
    values.update(overrides)
    return select_control_source(now, **values)


def test_fresh_manual_beats_fresh_airspeed_nav():
    assert _select(10.1) == SOURCE_MANUAL


def test_stale_manual_does_not_take_precedence():
    assert _select(10.31) == SOURCE_NAV
    assert _select(10.31, nav_valid=False) == SOURCE_FALLBACK


def test_inactive_or_disabled_manual_does_not_take_precedence():
    assert _select(10.1, manual_active=False) == SOURCE_NAV
    assert _select(10.1, enabled=False) == SOURCE_FALLBACK
    assert _select(10.1, manual_time=None, nav_time=None) == SOURCE_FALLBACK


def _manual_msg(values, active=True):
    msg = ActuatorCommand()
    (msg.thrust_left_n, msg.thrust_right_n,
     msg.elevon_left_rad, msg.elevon_right_rad) = values
    msg.active = active
    return msg


def _spec():
    from aerial_kit.airframes.twin_wing import load_twin_wing_spec

    return load_twin_wing_spec()


def test_manual_message_keeps_physical_order():
    command, clipped = manual_actuator_from_msg(
        _manual_msg([1.0, 2.0, -0.1, 0.2]), _spec(), 10.0)
    np.testing.assert_allclose(command, [1.0, 2.0, -0.1, 0.2], atol=1e-6)
    assert not clipped


def test_manual_limits_are_enforced():
    spec = _spec()
    command, clipped = manual_actuator_from_msg(
        _manual_msg([-1.0, 99.0, 3.0, -3.0]), spec, 8.0)
    assert clipped
    np.testing.assert_allclose(
        command, [0.0, 8.0, spec.max_elevon_rad, -spec.max_elevon_rad], atol=1e-6)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_manual_values_are_refused(bad):
    with pytest.raises(ValueError, match='non-finite'):
        manual_actuator_from_msg(_manual_msg([1.0, 1.0, bad, 0.0]), _spec(), 10.0)


@pytest.fixture
def mujoco_adapter():
    import rclpy

    rclpy.init(args=['--ros-args', '-p', 'backend:=mujoco', '-p', 'start_z_m:=60.0',
                     '-p', 'manual_timeout_sec:=0.3'])
    node = adapter.FixedWingBackendAdapterNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_command_drives_the_articulated_mujoco_plant(mujoco_adapter):
    from std_msgs.msg import Bool

    from sim_py.backends.mujoco_fixedwing_backend import MujocoFixedWingBackend

    node = mujoco_adapter
    assert isinstance(node.backend, MujocoFixedWingBackend)
    node._on_enable(Bool(data=True))
    travel = node.spec.max_elevon_rad
    roll_right = _manual_msg([2.0, 2.0, -0.15, 0.15])
    for _ in range(40):
        node._on_manual(roll_right)
        node._tick()
    assert node.source == SOURCE_MANUAL
    hinges = node.backend.actuator_state()
    # The command reaches MuJoCo's servos (rate-limited), not the pose.
    assert hinges['right_elevon_rad'] == pytest.approx(0.15, abs=0.02)
    assert hinges['left_elevon_rad'] == pytest.approx(-0.15, abs=0.02)
    assert hinges['right_prop_rad_s'] > 0.0 > hinges['left_prop_rad_s']
    assert node.backend.state().body_rates[0] > 0.2
    assert abs(hinges['right_elevon_rad']) <= travel


def test_stale_manual_falls_back_to_the_safe_circle(mujoco_adapter, monkeypatch):
    from std_msgs.msg import Bool

    node = mujoco_adapter
    node._on_enable(Bool(data=True))
    clock = {'t': 1000.0}
    monkeypatch.setattr(adapter.time, 'monotonic', lambda: clock['t'])
    node._on_manual(_manual_msg([0.0, 0.0, -0.3, -0.3]))
    node._tick()
    assert node.source == SOURCE_MANUAL
    clock['t'] += 0.31
    node._tick()
    assert node.source == SOURCE_FALLBACK


def test_non_finite_manual_cancels_manual_control(mujoco_adapter):
    from std_msgs.msg import Bool

    node = mujoco_adapter
    node._on_enable(Bool(data=True))
    node._on_manual(_manual_msg([1.0, 1.0, 0.0, 0.0]))
    node._tick()
    assert node.source == SOURCE_MANUAL
    node._on_manual(_manual_msg([1.0, math.nan, 0.0, 0.0]))
    node._tick()
    assert node.source == SOURCE_FALLBACK
    assert np.all(np.isfinite(node.backend.data.qpos))


def test_manual_timeout_is_validated():
    with pytest.raises(ValueError, match='positive'):
        validate_launch_parameters({**GOOD, 'manual_timeout_sec': 0.0})


def test_viewer_close_waits_for_the_render_thread():
    import threading
    import time as _time

    stop = threading.Event()
    finished = []

    def render():
        stop.wait(5.0)
        _time.sleep(0.2)          # GL teardown after the exit request
        finished.append(True)

    class FakeViewer:
        def close(self):
            stop.set()

    viewer = FakeViewer()
    thread = threading.Thread(target=render, daemon=True)
    thread.start()
    adapter._VIEWER_THREADS[id(viewer)] = [thread]
    adapter.close_mujoco_viewer(viewer)
    assert finished == [True]
    assert not thread.is_alive()
    assert id(viewer) not in adapter._VIEWER_THREADS


# --- single-motor wing mode ----------------------------------------------------

from sim_bridge.fixedwing_backend_adapter_node import single_wing_manual_command  # noqa: E402


def test_airframe_parameter_is_validated():
    assert validate_launch_parameters({**GOOD, 'airframe': 'Single_Wing'})['airframe'] == 'single_wing'
    with pytest.raises(ValueError, match="'twin_wing' or 'single_wing'"):
        validate_launch_parameters({**GOOD, 'airframe': 'tri_wing'})


def test_single_wing_manual_command_sums_thrust_and_keeps_physical_elevons():
    from aerial_kit.airframes.single_wing import load_single_wing_spec

    spec = load_single_wing_spec()
    cmd, clipped = single_wing_manual_command(_manual_msg([1.0, 1.5, -0.1, 0.2]), spec)
    np.testing.assert_allclose(cmd, [2.5, -0.1, 0.2], atol=1e-6)
    assert not clipped
    cmd, clipped = single_wing_manual_command(_manual_msg([9.0, 9.0, 3.0, -3.0]), spec)
    assert clipped and cmd[0] == spec.max_thrust_n
    assert cmd[1] == pytest.approx(spec.max_elevon_rad) and cmd[2] == pytest.approx(-spec.max_elevon_rad)
    with pytest.raises(ValueError, match='non-finite'):
        single_wing_manual_command(_manual_msg([math.nan, 0.0, 0.0, 0.0]), spec)


def test_single_wing_node_flies_manual_commands_on_mujoco():
    import rclpy
    from std_msgs.msg import Bool

    from sim_py.backends.mujoco_single_wing_backend import MujocoSingleWingBackend

    rclpy.init(args=['--ros-args', '-p', 'backend:=mujoco', '-p', 'airframe:=single_wing',
                     '-p', 'start_z_m:=60.0'])
    node = adapter.FixedWingBackendAdapterNode()
    try:
        assert isinstance(node.backend, MujocoSingleWingBackend)
        node._on_enable(Bool(data=True))
        for _ in range(40):
            node._on_manual(_manual_msg([1.0, 1.0, -0.15, 0.15]))
            node._tick()
        assert node.source == SOURCE_MANUAL
        s = node.backend.actuator_state()
        assert s['right_elevon_rad'] == pytest.approx(0.15, abs=0.02)
        assert s['left_elevon_rad'] == pytest.approx(-0.15, abs=0.02)
        assert s['thrust_n'] == pytest.approx(2.0, abs=0.1)
        assert node.backend.state().body_rates[0] > 0.2
        # Release: falls back to the closed-loop circle, still flying.
        node._on_enable(Bool(data=False))
        for _ in range(300):
            node._tick()
        assert node.source == SOURCE_FALLBACK
        state = node.backend.state()
        assert np.all(np.isfinite(node.backend.data.qpos)) and state.airspeed_mps > 9.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
