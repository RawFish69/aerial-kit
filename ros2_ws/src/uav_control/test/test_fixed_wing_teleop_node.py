import math
import threading

import numpy as np
import pytest

from uav_control.fixed_wing_teleop_node import (
    KeyRepeatFilter,
    TeleopController,
    actuator_message,
    plant_trim,
    tk_key_token,
    tuning_from_trim,
)

TRIM_T, TRIM_E = plant_trim()


def controller(**kwargs):
    return TeleopController(tuning_from_trim(TRIM_T, TRIM_E), **kwargs)


def armed(*keys):
    c = controller()
    c.press('return')
    for key in keys:
        c.press(key)
    return c


def cmd(c):
    return c.output().actuator


# --- mapping (physical [thrust_L, thrust_R, elevon_L, elevon_R], TE-up +) ---

def test_trim_comes_from_the_plant_not_the_python_teleop_default():
    assert 0.5 < TRIM_T < 1.5
    out = controller().output()
    np.testing.assert_allclose(out.actuator, [TRIM_T, TRIM_T, TRIM_E, TRIM_E])


def test_right_bank_raises_the_right_elevon_above_the_left():
    out = cmd(armed('d'))
    assert out[3] > out[2]
    assert out[3] - TRIM_E == pytest.approx(-(out[2] - TRIM_E))
    np.testing.assert_allclose(out[:2], [TRIM_T, TRIM_T])


def test_left_bank_is_the_mirror_image():
    right, left = cmd(armed('d')), cmd(armed('a'))
    assert left[2] > left[3]
    assert left[2] == pytest.approx(right[3])
    assert left[3] == pytest.approx(right[2])


def test_pitch_keys_move_both_elevons_together():
    up, down = cmd(armed('s')), cmd(armed('w'))
    assert up[2] == pytest.approx(up[3]) and up[2] > TRIM_E
    assert down[2] == pytest.approx(down[3]) and down[2] < TRIM_E
    np.testing.assert_allclose(up[:2], [TRIM_T, TRIM_T])


def test_throttle_keys_change_both_motors_together():
    more, less = cmd(armed('space')), cmd(armed('shift'))
    assert more[0] == pytest.approx(more[1]) and more[0] > TRIM_T + 1.0
    assert less[0] == pytest.approx(less[1]) and less[0] < TRIM_T
    assert less[0] >= 0.0
    np.testing.assert_allclose(more[2:], [TRIM_E, TRIM_E])


def test_q_and_e_are_differential_thrust_only():
    q, e = cmd(armed('q')), cmd(armed('e'))
    # Q yaws left: the right motor pushes harder.
    assert q[1] > q[0]
    assert e[0] > e[1]
    assert q[0] + q[1] == pytest.approx(2 * TRIM_T)
    np.testing.assert_allclose(q[2:], [TRIM_E, TRIM_E])
    np.testing.assert_allclose(e[2:], [TRIM_E, TRIM_E])


def test_x_restores_trim():
    c = armed('d', 's', 'space')
    assert not np.allclose(cmd(c), [TRIM_T, TRIM_T, TRIM_E, TRIM_E])
    c.press('x')
    np.testing.assert_allclose(cmd(c), [TRIM_T, TRIM_T, TRIM_E, TRIM_E])
    assert c.output().active


def test_pause_holds_trim_not_zero_throttle():
    c = armed('d', 'shift')
    c.press('p')
    out = c.output()
    assert out.paused and out.active
    np.testing.assert_allclose(out.actuator, [TRIM_T, TRIM_T, TRIM_E, TRIM_E])
    assert out.actuator[0] > 0.5
    c.press('p')
    assert not np.allclose(cmd(c), [TRIM_T, TRIM_T, TRIM_E, TRIM_E])


def test_focus_loss_clears_held_keys_and_sends_trim():
    c = armed('d', 'space')
    c.set_focused(False, now=10.0)
    assert c.keys.held_keys == set()
    np.testing.assert_allclose(cmd(c), [TRIM_T, TRIM_T, TRIM_E, TRIM_E])
    c.set_focused(True, now=10.5)
    # A key that was down when focus was lost does not come back by itself.
    np.testing.assert_allclose(cmd(c), [TRIM_T, TRIM_T, TRIM_E, TRIM_E])


def test_long_focus_loss_disarms():
    c = controller(unfocused_disarm_sec=3.0)
    c.press('return')
    c.set_focused(False, now=100.0)
    c.update(now=102.9)
    assert c.armed
    c.update(now=103.0)
    assert not c.armed and not c.output().active


def test_arm_disarm_and_exit():
    c = controller()
    assert not c.output().active
    c.press('return')
    assert c.output().active
    c.press('d')
    c.press('backspace')
    out = c.output()
    assert not out.active and c.keys.held_keys == set()
    np.testing.assert_allclose(out.actuator, [TRIM_T, TRIM_T, TRIM_E, TRIM_E])
    c.press('return')
    c.press('escape')
    assert c.exit_requested and not c.armed


def test_closing_the_window_disarms_and_exits():
    c = armed('d')
    c.request_exit()
    assert c.exit_requested and not c.output().active


def test_disarmed_keys_do_not_produce_manual_authority():
    c = controller()
    c.press('d')
    out = c.output()
    assert not out.active
    np.testing.assert_allclose(out.actuator, [TRIM_T, TRIM_T, TRIM_E, TRIM_E])


# --- Tk key handling ---------------------------------------------------------

@pytest.mark.parametrize('keysym,token', [
    ('w', 'w'), ('W', 'w'), ('space', 'space'), ('Shift_L', 'shift'),
    ('Shift_R', 'shift'), ('Return', 'return'), ('KP_Enter', 'return'),
    ('BackSpace', 'backspace'), ('Escape', 'escape'), ('Up', 'up'),
])
def test_tk_keysyms_map_to_shared_tokens(keysym, token):
    assert tk_key_token(keysym) == token


class FakeTimers:
    def __init__(self):
        self.pending = {}
        self.next = 0

    def schedule(self, delay, callback):
        self.next += 1
        self.pending[self.next] = callback
        return self.next

    def cancel(self, handle):
        self.pending.pop(handle, None)

    def fire_all(self):
        for handle in list(self.pending):
            self.pending.pop(handle)()


def test_autorepeat_release_press_pairs_keep_the_key_latched():
    timers, c = FakeTimers(), armed()
    f = KeyRepeatFilter(timers.schedule, timers.cancel)
    assert f.press('d')
    c.press('d')
    for _ in range(5):          # X11 auto-repeat: release + press pairs
        f.release('d', c.release)
        assert not f.press('d')
    assert 'd' in c.keys.held_keys
    f.release('d', c.release)   # the real release
    timers.fire_all()
    assert 'd' not in c.keys.held_keys


def test_held_key_needs_no_repeats():
    timers, c = FakeTimers(), armed()
    f = KeyRepeatFilter(timers.schedule, timers.cancel)
    assert f.press('space')
    c.press('space')
    timers.fire_all()           # time passes, no repeat events at all
    assert cmd(c)[0] > TRIM_T + 1.0


def test_repeated_toggle_key_press_is_ignored():
    timers = FakeTimers()
    f = KeyRepeatFilter(timers.schedule, timers.cancel)
    assert f.press('p')
    assert not f.press('p')
    f.release('p', lambda token: None)
    timers.fire_all()
    assert f.press('p')


# --- message and node --------------------------------------------------------

def test_actuator_message_layout():
    from uav_msgs.msg import ActuatorCommand

    fields = ActuatorCommand.get_fields_and_field_types()
    assert list(fields) == [
        'header', 'sequence_id', 'thrust_left_n', 'thrust_right_n',
        'elevon_left_rad', 'elevon_right_rad', 'active']
    assert fields['sequence_id'] == 'uint32'
    assert fields['active'] == 'boolean'
    for name in ('thrust_left_n', 'thrust_right_n', 'elevon_left_rad', 'elevon_right_rad'):
        assert fields[name] == 'float'

    c = armed('d', 'space')
    msg = actuator_message(ActuatorCommand, c.output(), 42)
    out = c.output().actuator
    assert msg.sequence_id == 42 and msg.active
    assert msg.thrust_left_n == pytest.approx(out[0])
    assert msg.thrust_right_n == pytest.approx(out[1])
    assert msg.elevon_left_rad == pytest.approx(out[2])
    assert msg.elevon_right_rad == pytest.approx(out[3])
    assert msg.elevon_right_rad > msg.elevon_left_rad


def test_node_publishes_and_shuts_down_cleanly():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool
    from uav_msgs.msg import ActuatorCommand

    from uav_control.fixed_wing_teleop_node import _make_node

    threads_before = threading.active_count()
    rclpy.init()
    try:
        node = _make_node()
        listener = rclpy.create_node('teleop_test_listener')
        commands, enables = [], []
        listener.create_subscription(
            ActuatorCommand, '/uav/backend/cmd_actuators', commands.append, 10)
        listener.create_subscription(Bool, '/uav/backend/enable', enables.append, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(listener)
        node.controller.press('return')
        node.controller.press('d')
        for _ in range(20):
            node.publish_once()
            executor.spin_once(timeout_sec=0.02)
        node.publish_disarm()
        for _ in range(20):
            executor.spin_once(timeout_sec=0.02)
        assert commands and enables
        assert any(m.active and m.elevon_right_rad > m.elevon_left_rad for m in commands)
        assert not commands[-1].active and not enables[-1].data
        names = node.get_node_names()
        assert 'fixed_wing_teleop_node' in names
        executor.shutdown()
        listener.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert threading.active_count() <= threads_before
    assert math.isfinite(TRIM_E)


def test_single_wing_trim_splits_one_motor_across_both_slots():
    from aerial_kit.airframes.single_wing import load_single_wing_spec
    from aerial_kit.dynamics.single_wing import SingleWingDynamics

    per_slot, elevon = plant_trim('', 15.0, 'single_wing')
    trim = SingleWingDynamics(load_single_wing_spec().dynamics_params()).compute_trim_detailed(15.0)
    assert 2 * per_slot == pytest.approx(trim['thrust_n'])
    assert elevon == pytest.approx(trim['elevon_common_rad'])


def test_c_key_counts_camera_toggles_without_moving_controls():
    c = armed('c')
    assert c.keys.camera_toggle_requests == 1
    np.testing.assert_allclose(cmd(c), [TRIM_T, TRIM_T, TRIM_E, TRIM_E])
