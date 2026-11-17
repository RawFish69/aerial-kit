import pytest

from hw_bridge.crsf_radio_mapping import CrsfChannelMap
from hw_bridge.rc_mapping import RcMapParams, RcSticks, params_from_airframe
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
    assert channels[5] == 992
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
