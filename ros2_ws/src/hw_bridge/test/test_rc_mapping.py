from hw_bridge.rc_mapping import RcMapParams, neutral_sticks, velocity_to_rc


def p() -> RcMapParams:
    return RcMapParams(
        kv_xy=0.5,
        max_tilt_deg=25.0,
        hover_throttle=0.5,
        kz=0.2,
        throttle_min=0.05,
        throttle_max=0.95,
        max_yaw_rate_rps=1.5,
    )


def test_zero_velocity_is_neutral_hover():
    r = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=0.0, wz=0.0)
    assert abs(r.roll) < 1e-9
    assert abs(r.pitch) < 1e-9
    assert abs(r.yaw) < 1e-9
    assert abs(r.throttle - 0.5) < 1e-9


def test_forward_velocity_pitches_forward():
    r = velocity_to_rc(p(), vx=0.0, vy=2.0, vz=0.0, wz=0.0)
    assert r.pitch > 0.0


def test_tilt_saturates_at_one():
    r = velocity_to_rc(p(), vx=100.0, vy=0.0, vz=0.0, wz=0.0)
    assert abs(r.roll - 1.0) < 1e-9


def test_throttle_clamped():
    r = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=100.0, wz=0.0)
    assert r.throttle <= 0.95
    r2 = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=-100.0, wz=0.0)
    assert r2.throttle >= 0.05


def test_yaw_maps_and_clamps():
    r = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=0.0, wz=1.5)
    assert abs(r.yaw - 1.0) < 1e-9


def test_neutral_sticks_helper():
    n = neutral_sticks(p())
    assert n.roll == 0.0 and n.pitch == 0.0 and n.yaw == 0.0
    assert abs(n.throttle - 0.5) < 1e-9
