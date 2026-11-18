import threading

import pytest

from uav_control.multirotor_teleop_node import (
    MultirotorTeleopController,
    default_mode,
    manual_message,
)


def armed(*keys, mode='angle'):
    c = MultirotorTeleopController(mode)
    c.press('return')
    for key in keys:
        c.press(key)
    return c


def test_default_modes_follow_the_airframe_role():
    assert default_mode('fpv') == 'angle'
    assert default_mode('research') == 'velocity'


def test_centred_sticks_when_idle_paused_or_unfocused():
    c = MultirotorTeleopController('angle')
    c.press('d')
    out = c.output()
    assert not out.active and (out.roll, out.pitch, out.yaw, out.throttle) == (0, 0, 0, 0.5)
    c = armed('d', 'space')
    c.press('p')
    assert (c.output().roll, c.output().throttle) == (0.0, 0.5)
    c = armed('d')
    c.set_focused(False, 1.0)
    assert c.output().roll == 0.0 and c.keys.held_keys == set()


@pytest.mark.parametrize('key,field,value', [
    ('d', 'roll', 1.0), ('a', 'roll', -1.0),
    ('w', 'pitch', 1.0), ('s', 'pitch', -1.0),
    ('e', 'yaw', 1.0), ('q', 'yaw', -1.0),
])
def test_stick_signs(key, field, value):
    out = armed(key).output()
    assert getattr(out, field) == value


def test_throttle_keys():
    assert armed('space').output().throttle == pytest.approx(0.85)
    assert armed('shift').output().throttle == pytest.approx(0.15)
    # Acro throttle is the raw collective, so keys move it gently.
    assert armed('space', mode='acro').output().throttle == pytest.approx(0.56)


def test_mode_key_cycles_and_recentres():
    c = armed('d', mode='velocity')
    c.press('m')
    assert c.mode == 'angle' and c.output().roll == 0.0
    c.press('m')
    c.press('m')
    assert c.mode == 'velocity'


def test_x_recentres_and_disarm_exit():
    c = armed('d', 'w')
    c.press('x')
    out = c.output()
    assert (out.roll, out.pitch) == (0.0, 0.0) and out.active
    c.press('backspace')
    assert not c.output().active
    c.press('return')
    c.press('escape')
    assert c.exit_requested and not c.armed


def test_long_focus_loss_disarms():
    c = armed()
    c.set_focused(False, 10.0)
    c.update(12.9)
    assert c.armed
    c.update(13.0)
    assert not c.armed


def test_message_encoding():
    pytest.importorskip('rclpy')
    pytest.importorskip('uav_msgs.msg')
    from uav_msgs.msg import MultirotorManualCommand

    out = armed('d', 'space', mode='acro').output()
    msg = manual_message(MultirotorManualCommand, out, 7)
    assert msg.mode == MultirotorManualCommand.MODE_ACRO
    # Acro: raw throttle with the gentle keyboard step (0.5 + 0.06).
    assert msg.roll == 1.0 and msg.throttle == pytest.approx(0.56) and msg.active
    assert msg.sequence_id == 7


def test_node_publishes_and_shuts_down_cleanly():
    pytest.importorskip('rclpy')
    pytest.importorskip('uav_msgs.msg')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool
    from uav_msgs.msg import MultirotorManualCommand

    from uav_control.multirotor_teleop_node import _make_node

    before = threading.active_count()
    rclpy.init(args=['--ros-args', '-p', 'airframe:=research_hex'])
    try:
        node = _make_node()
        assert node.controller.mode == 'velocity'
        listener = rclpy.create_node('mr_teleop_listener')
        got, enables = [], []
        listener.create_subscription(MultirotorManualCommand, '/uav/backend/manual', got.append, 10)
        listener.create_subscription(Bool, '/uav/backend/enable', enables.append, 10)
        ex = SingleThreadedExecutor()
        ex.add_node(listener)
        node.controller.press('return')
        node.controller.press('w')
        for _ in range(15):
            node.publish_once()
            ex.spin_once(timeout_sec=0.02)
        node.publish_disarm()
        for _ in range(15):
            ex.spin_once(timeout_sec=0.02)
        assert any(m.active and m.pitch == 1.0 for m in got)
        assert not got[-1].active and not enables[-1].data
        ex.shutdown()
        listener.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert threading.active_count() <= before


def test_c_key_requests_a_viewer_camera_toggle():
    pytest.importorskip('rclpy')
    pytest.importorskip('uav_msgs.msg')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    from uav_control.multirotor_teleop_node import _make_node

    rclpy.init()
    try:
        node = _make_node()
        listener = rclpy.create_node('camera_listener')
        got = []
        listener.create_subscription(String, '/uav/sim/viewer_camera', got.append, 10)
        ex = SingleThreadedExecutor()
        ex.add_node(listener)
        node.controller.press('c')
        node.controller.press('c')
        for _ in range(10):
            node.publish_once()
            ex.spin_once(timeout_sec=0.02)
        assert [m.data for m in got] == ['next', 'next']
        assert node.controller.keys.held_keys == set()     # C is not a held control
        ex.shutdown()
        listener.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
