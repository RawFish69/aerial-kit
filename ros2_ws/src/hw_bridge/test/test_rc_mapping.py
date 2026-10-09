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


def test_measured_climb_rate_closes_the_vertical_loop():
    # Already climbing at the demanded rate: no extra thrust beyond hover.
    at_rate = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=0.8, wz=0.0, measured_vz=0.8)
    assert abs(at_rate.throttle - 0.5) < 1e-9
    # Overshooting the demand: below hover throttle, i.e. braking.
    too_fast = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=0.8, wz=0.0, measured_vz=4.0)
    assert too_fast.throttle < 0.5
    # Open-loop (no measurement) keeps the old behaviour.
    open_loop = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=0.8, wz=0.0)
    assert abs(open_loop.throttle - (0.5 + 0.2 * 0.8)) < 1e-9


def test_measured_horizontal_velocity_acts_on_the_error():
    r = velocity_to_rc(p(), vx=0.0, vy=2.0, vz=0.0, wz=0.0, measured_vxy=(0.0, 2.0))
    assert abs(r.pitch) < 1e-9 and abs(r.roll) < 1e-9
    braking = velocity_to_rc(p(), vx=0.0, vy=0.0, vz=0.0, wz=0.0, measured_vxy=(1.0, 0.0))
    assert braking.roll < 0.0  # drifting right -> roll left


def test_vertical_feedback_settles_a_thrust_plant_on_the_demand():
    """Throttle sets acceleration (as in Angle mode, and fake_fc_sim). Open-loop,
    a constant climb demand never stops accelerating; with feedback the climb
    rate settles near the demand."""
    gain, drag, dt = 30.0, 0.5, 0.01

    def climb(feedback):
        vz = 0.0
        for _ in range(500):  # 5 s
            r = velocity_to_rc(p(), 0.0, 0.0, 0.8, 0.0, measured_vz=vz if feedback else None)
            vz += ((r.throttle - 0.5) * gain - drag * vz) * dt
        return vz

    assert climb(feedback=False) > 4.0  # the SITL runaway
    assert abs(climb(feedback=True) - 0.8) < 0.1
