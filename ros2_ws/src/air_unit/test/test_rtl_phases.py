from air_unit.rtl import RTL_ARRIVED, RTL_CLIMB, RTL_CRUISE, RtlParams, rtl_command


def pr() -> RtlParams:
    return RtlParams(altitude_m=15.0, cruise_speed_mps=3.0, arrival_radius_m=1.0, kp=0.8)


def test_climb_phase_when_below_altitude():
    vx, vy, vz, phase = rtl_command(pr(), pos=(5.0, 5.0, 2.0), home=(0.0, 0.0))
    assert phase == RTL_CLIMB
    assert vz > 0.0


def test_cruise_phase_heads_home():
    vx, vy, vz, phase = rtl_command(pr(), pos=(10.0, 0.0, 15.0), home=(0.0, 0.0))
    assert phase == RTL_CRUISE
    assert vx < 0.0  # move toward home (negative x)


def test_cruise_speed_clamped():
    vx, vy, vz, phase = rtl_command(pr(), pos=(100.0, 0.0, 15.0), home=(0.0, 0.0))
    speed = (vx**2 + vy**2) ** 0.5
    assert speed <= 3.0 + 1e-6


def test_arrived_when_within_radius():
    vx, vy, vz, phase = rtl_command(pr(), pos=(0.5, 0.0, 15.0), home=(0.0, 0.0))
    assert phase == RTL_ARRIVED


RTL, LAND, HOVER = 5, 4, 2  # uav_msgs/Command values


def test_repeated_rtl_request_keeps_landing_once_rtl_handed_over():
    from air_unit.rtl import resolve_mode_request

    # Before arrival, RTL is RTL.
    assert resolve_mode_request(RTL, False, rtl_mode=RTL, land_mode=LAND) == (RTL, False)
    # After RTL switched to LAND, the ground station's next RTL must not climb again.
    assert resolve_mode_request(RTL, True, rtl_mode=RTL, land_mode=LAND) == (LAND, True)
    # Any other request clears the latch and is honoured.
    assert resolve_mode_request(HOVER, True, rtl_mode=RTL, land_mode=LAND) == (HOVER, False)
    assert resolve_mode_request(RTL, False, rtl_mode=RTL, land_mode=LAND) == (RTL, False)
