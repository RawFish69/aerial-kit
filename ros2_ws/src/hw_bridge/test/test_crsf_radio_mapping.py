import pytest

from hw_bridge.crsf_radio_mapping import (
    BackendArmSequencer,
    CrsfChannelMap,
    ReceiverArmingGate,
    rotate_world_velocity_to_body,
    velocity_to_manual_sticks,
)
from hw_bridge.rc_mapping import RcMapParams, RcSticks, params_from_airframe, velocity_to_rc
from hw_bridge.sim_elrs_link_node import delivery_enabled_for_cut


def params() -> RcMapParams:
    return RcMapParams(
        kv_xy=0.5,
        max_tilt_deg=25.0,
        hover_throttle=0.5,
        kz=0.2,
        throttle_min=0.05,
        throttle_max=0.95,
        max_yaw_rate_rps=1.5,
    )


def test_aetr_frame_maps_arm_and_neutral_channels():
    channel_map = CrsfChannelMap()
    channels = channel_map.encode(
        RcSticks(roll=0.5, pitch=-0.25, yaw=0.1, throttle=0.7),
        enabled=True,
    )
    assert len(channels) == 16
    assert channels[0] > 992
    assert channels[1] < 992
    assert channels[2] > 992
    assert channels[3] > 992
    assert channels[4] == 1811
    assert channels[5] == 1811
    decoded = channel_map.decode_sticks(channels)
    assert decoded.arm_request
    assert decoded.angle_mode
    assert channels[6:] == (992,) * 10


def test_decode_round_trip_and_disarmed_state():
    channel_map = CrsfChannelMap()
    p = params()
    encoded = channel_map.encode(RcSticks(0.2, -0.3, 0.1, 0.65), enabled=True)
    enabled, command = channel_map.decode(encoded, p)
    assert enabled
    assert command.linear.x == pytest.approx(0.2 * 0.4363323129985824 / 0.5, abs=1e-3)
    assert command.linear.y == pytest.approx(-0.3 * 0.4363323129985824 / 0.5, abs=1e-3)
    assert command.linear.z == pytest.approx((0.65 - 0.5) / 0.2, abs=2e-3)
    assert command.angular.z == pytest.approx(0.1 * 1.5, abs=1e-3)

    disabled = channel_map.encode(RcSticks(0.0, 0.0, 0.0, 0.0), enabled=False)
    enabled, command = channel_map.decode(disabled, p)
    assert not enabled
    assert command.linear.x == command.linear.y == command.linear.z == 0.0
    assert command.angular.z == 0.0


def test_multirotor_spec_supplies_bank_limit():
    assert params_from_airframe('fpv_quad_5in').max_tilt_deg == pytest.approx(60.0)


def test_link_cut_topic_inverts_to_delivery_state():
    assert delivery_enabled_for_cut(False)
    assert not delivery_enabled_for_cut(True)


def test_backend_arm_hold_then_handover_and_disable_reverse():
    channel_map = CrsfChannelMap()
    sequencer = BackendArmSequencer(arm_hold_sec=1.0, throttle_min=0.0)
    hover = RcSticks(0.1, 0.2, -0.1, 0.5)

    hold_sticks, arm_request = sequencer.update(True, hover, 10.0)
    hold = channel_map.encode(hold_sticks, enabled=arm_request)
    assert hold[4] == 1811
    assert hold[5] == 1811
    assert hold[2] == 172
    assert hold_sticks.roll == hold_sticks.pitch == hold_sticks.yaw == 0.0

    still_holding, arm_request = sequencer.update(True, hover, 10.99)
    assert arm_request
    assert still_holding.throttle == pytest.approx(0.0)

    handed_over, arm_request = sequencer.update(True, hover, 11.0)
    assert arm_request
    assert handed_over == hover

    disarmed_sticks, arm_request = sequencer.update(False, hover, 12.0)
    disarmed = channel_map.encode(disarmed_sticks, enabled=arm_request)
    assert not arm_request
    assert disarmed[4] == 172
    assert disarmed[5] == 172
    assert disarmed[2] == 172


def test_receiver_rejects_arm_with_high_throttle_then_arms_after_low_hold():
    gate = ReceiverArmingGate(arm_hold_sec=0.5, throttle_low=0.05)
    assert not gate.update(True, 0.5, 0.0)
    assert not gate.update(True, 0.5, 1.0)
    assert not gate.update(True, 0.05, 1.1)
    assert not gate.update(True, 0.05, 1.5)
    assert gate.update(True, 0.05, 1.6)
    assert not gate.update(False, 0.05, 1.7)


def test_world_velocity_rotates_into_body_at_ninety_degrees():
    body_vx, body_vy = rotate_world_velocity_to_body(0.0, 2.0, 1.5707963267948966)
    assert (body_vx, body_vy) == pytest.approx((0.0, 2.0))
    velocity_sticks = velocity_to_manual_sticks(
        body_vx, body_vy, -1.0, 0.0,
        max_speed_mps=8.0, max_climb_mps=3.0, max_yaw_rate_rps=1.5,
    )
    assert velocity_sticks.roll == pytest.approx(0.0)
    assert velocity_sticks.pitch == pytest.approx(0.25)
    assert velocity_sticks.throttle == pytest.approx(1.0 / 3.0)
    angle_sticks = velocity_to_rc(params(), body_vx, body_vy, 0.0, 0.0)
    assert angle_sticks.roll == pytest.approx(0.0)
    assert angle_sticks.pitch > 0.0


def test_reconnect_reset_requires_a_fresh_arm_hold():
    sequencer = BackendArmSequencer(arm_hold_sec=1.0, throttle_min=0.0)
    hover = RcSticks(0.0, 0.0, 0.0, 0.5)
    sequencer.update(True, hover, 0.0)
    handed_over, armed = sequencer.update(True, hover, 1.0)
    assert armed and handed_over == hover

    sequencer.reset()
    restarted, arm_request = sequencer.update(True, hover, 2.0)
    assert arm_request
    assert restarted.throttle == pytest.approx(0.0)
    still_holding, arm_request = sequencer.update(True, hover, 2.5)
    assert arm_request
    assert still_holding.throttle == pytest.approx(0.0)
    handed_over, arm_request = sequencer.update(True, hover, 3.0)
    assert arm_request and handed_over == hover


def test_serial_reconnect_reopens_and_resets_arm_sequence(monkeypatch):
    import rclpy

    from hw_bridge import crsf_radio_backend_node as backend_module

    class FakeTransport:
        def __init__(self, port, *, baudrate=400000, timeout=0.05):
            self.port = port
            self.baudrate = baudrate
            self.closed = False

        def read(self, size=-1):
            return b''

        def write(self, data):
            return len(data)

        def close(self):
            self.closed = True

    rclpy.init()
    monkeypatch.setattr(backend_module, 'SerialTransport', FakeTransport)
    node = backend_module.CrsfRadioBackendNode()
    try:
        assert node.link is not None
        node.arm_sequencer.armed = True
        node._drop_serial('test unplug')
        assert node.link is None
        assert not node.arm_sequencer.armed

        node._last_reconnect_attempt = float('-inf')
        node._try_open_serial()
        assert node.link is not None
        assert not node.arm_sequencer.armed
        assert node.link.transport.baudrate == 400000
    finally:
        if node.link is not None:
            node.link.close()
        node.destroy_node()
        rclpy.shutdown()
