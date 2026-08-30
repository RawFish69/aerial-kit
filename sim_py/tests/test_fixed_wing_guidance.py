"""Tests for L1/TECS-lite guidance and the FixedWingL1TECSController (workstream 04)."""

from __future__ import annotations

import numpy as np
import pytest

from aerial_kit.airframes.fixed_wing import TwinWingAirframe
from aerial_kit.guidance.l1 import l1_bank_command
from aerial_kit.guidance.tecs import TecsGains, tecs_command
from aerial_kit.controllers.fixed_wing import FixedWingL1TECSController, body_axis_pitch_bank
from sim_py.core.types import CommandKind, SimState, Waypoint
from aerial_kit.dynamics.fixed_wing import FixedWingDynamics, level_attitude_quat


def test_l1_bank_zero_when_target_directly_ahead():
    bank = l1_bank_command(
        position_xy=np.array([0.0, 0.0]),
        velocity_xy=np.array([15.0, 0.0]),
        target_xy=np.array([100.0, 0.0]),
        airspeed_mps=15.0,
        l1_distance_m=40.0,
        max_bank_rad=np.radians(45.0),
    )
    assert bank == pytest.approx(0.0, abs=1e-9)


def test_l1_bank_nonzero_and_clamped_for_off_axis_target():
    bank = l1_bank_command(
        position_xy=np.array([0.0, 0.0]),
        velocity_xy=np.array([15.0, 0.0]),
        target_xy=np.array([10.0, 100.0]),  # almost directly to the "left"
        airspeed_mps=15.0,
        l1_distance_m=40.0,
        max_bank_rad=np.radians(45.0),
    )
    assert abs(bank) == pytest.approx(np.radians(45.0), abs=1e-9)


def test_l1_closed_loop_kinematic_convergence():
    """A simple coordinated-turn kinematic model should reach a fixed target."""
    position = np.array([0.0, 0.0])
    heading = 0.0
    airspeed = 15.0
    target = np.array([200.0, 60.0])
    gravity = 9.81
    dt = 0.05

    for _ in range(2000):
        velocity_xy = airspeed * np.array([np.cos(heading), np.sin(heading)])
        bank = l1_bank_command(
            position_xy=position,
            velocity_xy=velocity_xy,
            target_xy=target,
            airspeed_mps=airspeed,
            l1_distance_m=30.0,
            max_bank_rad=np.radians(45.0),
            gravity_mps2=gravity,
        )
        # Standard aviation convention (matches aerial_kit.guidance.l1's docstring):
        # positive bank turns right, i.e. *decreases* world heading.
        turn_rate = -gravity * np.tan(bank) / airspeed
        heading += turn_rate * dt
        position = position + velocity_xy * dt
        if np.linalg.norm(position - target) < 5.0:
            break

    assert np.linalg.norm(position - target) < 5.0


def test_tecs_throttle_increases_when_slow_and_low():
    gains = TecsGains()
    throttle_low, _ = tecs_command(10.0, 15.0, 90.0, 100.0, gains)
    throttle_high, _ = tecs_command(15.0, 15.0, 100.0, 100.0, gains)
    assert throttle_low > throttle_high


def test_tecs_pitch_positive_when_low_and_fast():
    gains = TecsGains()
    _, pitch = tecs_command(18.0, 15.0, 90.0, 100.0, gains)
    assert pitch > 0.0


def test_tecs_clips_to_limits():
    gains = TecsGains(max_thrust_n=5.0, max_pitch_rad=0.2)
    throttle, pitch = tecs_command(0.0, 100.0, -1000.0, 1000.0, gains)
    assert throttle == pytest.approx(5.0)
    assert pitch == pytest.approx(0.2)


def test_body_axis_pitch_bank_level_is_zero():
    pitch, bank = body_axis_pitch_bank(level_attitude_quat(heading_rad=0.0))
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert bank == pytest.approx(0.0, abs=1e-9)


def test_controller_declares_airspeed_nav():
    controller = FixedWingL1TECSController()
    assert controller.command_kind == CommandKind.AIRSPEED_NAV


def test_controller_produces_finite_wrench():
    controller = FixedWingL1TECSController()
    state = SimState(
        position=np.array([0.0, 0.0, 100.0]),
        velocity=np.array([15.0, 0.0, 0.0]),
        attitude_quat=level_attitude_quat(0.0),
        body_rates=np.zeros(3),
    )
    target = Waypoint(position=np.array([200.0, 20.0, 110.0]))
    control_target = controller.compute(state, target, cfg={})
    wrench = control_target.metadata["wrench"]
    assert np.all(np.isfinite(wrench.force_body))
    assert np.all(np.isfinite(wrench.moment_body))
    assert wrench.force_body[0] >= 0.0  # throttle_cmd_n is clipped non-negative


def _step_with_controller(dyn: FixedWingDynamics, controller, target: Waypoint, cfg, dt: float) -> None:
    state = SimState(
        position=dyn.position.copy(),
        velocity=dyn.velocity.copy(),
        attitude_quat=dyn.attitude_quat.copy(),
        body_rates=dyn.body_rates.copy(),
    )
    control_target = controller.compute(state, target, cfg=cfg)
    wrench = control_target.metadata["wrench"]
    # Bypass TwinWingAirframe.allocate() saturation nuance for this groundwork
    # check -- feed the wrench's moment components directly as elevon
    # deflections (its own documented placeholder mapping) and split throttle
    # evenly, no differential-thrust yaw term.
    throttle = float(np.clip(wrench.force_body[0], 0.0, 10.0))
    roll_m, pitch_m, _yaw_m = wrench.moment_body
    max_elevon = np.radians(25.0)
    delta_pitch = float(np.clip(pitch_m, -max_elevon, max_elevon))
    delta_roll = float(np.clip(roll_m, -(max_elevon - abs(delta_pitch)), max_elevon - abs(delta_pitch)))
    actuator_cmd = np.array([throttle, throttle, delta_pitch + delta_roll, delta_pitch - delta_roll], dtype=float)
    dyn.step(actuator_cmd, dt)


def test_full_loop_tracks_a_laterally_offset_waypoint():
    """L1 + TECS + coordinated-turn attitude PID, closed over the real 6-DOF
    FixedWingDynamics, steering toward a target offset well off the nose.

    This is the turn-tracking case that motivated adding a coordinated-turn
    yaw-rate command (r_cmd = g*tan(bank)/V) to the attitude PID: banking
    without commanding the matching yaw rate produced pure sideslip and the
    aircraft flew away from the target entirely (a runaway positive-feedback
    loop, traced to a sign mismatch between aerial_kit.guidance.l1's bank
    convention and the standard-aviation convention body_axis_pitch_bank/the
    real dynamics use -- l1_bank_command now negates a_cmd before converting
    to bank for exactly this reason). With both fixed, the aircraft should
    actually close distance to a target that starts ~500 m away and 40 m off
    to the side.
    """
    dyn = FixedWingDynamics()
    cruise = 15.0
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    target = Waypoint(position=np.array([500.0, 40.0, 110.0]))
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}

    dt = 0.02
    initial_dist = float(np.linalg.norm(dyn.position[:2] - target.position[:2]))
    min_dist = initial_dist
    for _ in range(1800):  # 36 s -- enough to fly past a target ~500 m out at 15 m/s
        _step_with_controller(dyn, controller, target, cfg, dt)
        min_dist = min(min_dist, float(np.linalg.norm(dyn.position[:2] - target.position[:2])))

    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.velocity))
    assert np.all(np.isfinite(dyn.body_rates))
    assert min_dist < 50.0, f"closest approach {min_dist:.1f} m -- should have closed most of {initial_dist:.0f} m"
    airspeed, _, _ = dyn.airspeed_alpha_beta()
    assert abs(airspeed - cruise) < 3.0


def test_full_loop_holds_altitude_and_heading_flying_straight_ahead():
    """Groundwork case: target directly ahead, near-zero bank/pitch correction
    needed -- the closed loop should hold roughly level, on-heading,
    roughly-cruise-airspeed flight for 30 s rather than diverge."""
    dyn = FixedWingDynamics()
    cruise = 15.0
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    target = Waypoint(position=np.array([2000.0, 0.0, 100.0]))
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}

    dt = 0.02
    for _ in range(1500):  # 30 s
        _step_with_controller(dyn, controller, target, cfg, dt)

    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.velocity))
    assert np.all(np.isfinite(dyn.body_rates))
    assert np.linalg.norm(dyn.body_rates) < 0.5, "should have settled, not still oscillating"
    assert abs(dyn.position[2] - 100.0) < 5.0, "altitude should hold near cruise"
    assert dyn.position[0] > 300.0, "should have made steady forward progress, not stalled/reversed"
    airspeed, _, _ = dyn.airspeed_alpha_beta()
    assert abs(airspeed - cruise) < 3.0


def test_sustained_coordinated_turn_holds_altitude_through_360_degrees():
    """Plan 04's coordinated-turn acceptance test: a sustained turn holds
    altitude +/-5 m through a full 360 deg turn.

    Drives the loop with a target that always sits a fixed angle ahead on a
    circle (an orbit), forcing a sustained turn rather than the one-shot turn
    `test_full_loop_tracks_a_laterally_offset_waypoint` exercises -- this is
    what actually stresses the coordinated-turn yaw-rate command over many
    turn radii, not just an initial correction.
    """
    dyn = FixedWingDynamics()
    cruise = 15.0
    radius = 60.0
    center = np.array([0.0, radius])
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    cfg = {
        "controller": {
            "l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0]), "l1_distance_m": radius}
        }
    }

    dt = 0.02
    heading_swept = 0.0
    prev_angle = None
    altitude_min = altitude_max = float(dyn.position[2])
    for _ in range(6000):  # 120 s
        rel = dyn.position[:2] - center
        angle = float(np.arctan2(rel[1], rel[0]))
        if prev_angle is not None:
            dtheta = angle - prev_angle
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi  # unwrap
            heading_swept += dtheta
        prev_angle = angle

        look_angle = angle - 0.9  # lookahead point ahead along the orbit
        target_xy = center + radius * np.array([np.cos(look_angle), np.sin(look_angle)])
        target = Waypoint(position=np.array([target_xy[0], target_xy[1], 100.0]))
        _step_with_controller(dyn, controller, target, cfg, dt)

        altitude_min = min(altitude_min, float(dyn.position[2]))
        altitude_max = max(altitude_max, float(dyn.position[2]))

    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.body_rates))
    assert abs(heading_swept) >= 2 * np.pi, "should have completed at least one full orbit"
    assert altitude_max - 100.0 < 5.0
    assert 100.0 - altitude_min < 5.0


def test_climb_does_not_drop_below_stall_margin():
    """Plan 04's climb acceptance test: tracks a commanded climb without
    airspeed dropping below the stall margin (TwinWingAirframe's
    min_airspeed_mps, with headroom)."""
    dyn = FixedWingDynamics()
    cruise = 15.0
    climb_target_altitude = 150.0  # +50 m from release
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    target = Waypoint(position=np.array([2000.0, 0.0, climb_target_altitude]))
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}
    min_airspeed_mps = TwinWingAirframe().capabilities.min_airspeed_mps

    dt = 0.02
    min_airspeed_seen = float("inf")
    for _ in range(4000):  # 80 s
        _step_with_controller(dyn, controller, target, cfg, dt)
        airspeed, _, _ = dyn.airspeed_alpha_beta()
        min_airspeed_seen = min(min_airspeed_seen, airspeed)

    assert np.all(np.isfinite(dyn.position))
    assert min_airspeed_seen > min_airspeed_mps * 1.2, (
        f"airspeed dipped to {min_airspeed_seen:.1f} m/s during climb, "
        f"too close to stall speed {min_airspeed_mps:.1f} m/s"
    )
    assert dyn.position[2] > 100.0 + 30.0, "should have made real climb progress toward the target altitude"


def test_descent_does_not_overspeed():
    """Plan 04's descent counterpart: tracks a commanded descent without
    airspeed exceeding TwinWingAirframe's max_airspeed_mps."""
    dyn = FixedWingDynamics()
    cruise = 15.0
    descend_target_altitude = 100.0  # -50 m from release
    dyn.position = np.array([0.0, 0.0, 150.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    target = Waypoint(position=np.array([2000.0, 0.0, descend_target_altitude]))
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}
    max_airspeed_mps = TwinWingAirframe().capabilities.max_airspeed_mps

    dt = 0.02
    max_airspeed_seen = 0.0
    for _ in range(4000):  # 80 s
        _step_with_controller(dyn, controller, target, cfg, dt)
        airspeed, _, _ = dyn.airspeed_alpha_beta()
        max_airspeed_seen = max(max_airspeed_seen, airspeed)

    assert np.all(np.isfinite(dyn.position))
    assert max_airspeed_seen < max_airspeed_mps, (
        f"airspeed spiked to {max_airspeed_seen:.1f} m/s during descent, "
        f"at or past the capability limit {max_airspeed_mps:.1f} m/s"
    )
    assert dyn.position[2] < 150.0 - 30.0, "should have made real descent progress toward the target altitude"


def test_stall_guard_impossible_climb_clamps_and_does_not_stall():
    """Plan 04's stall-guard acceptance test: commanding an impossible climb
    clamps rather than driving the aircraft into a stall.

    The clamp mechanism is TecsGains.max_pitch_rad -- tecs_command() clips its
    pitch output regardless of how extreme the altitude error is (see
    aerial_kit/guidance/tecs.py), which is exactly plan 04's "hard limits,
    enforced, not advisory" requirement. This closes the loop over the real
    6-DOF dynamics with a deliberately absurd target (5000 m altitude, 50 m
    ahead) to confirm the clamp actually prevents a stall in practice, not
    just that the TECS function clips its own output in isolation.
    """
    dyn = FixedWingDynamics()
    cruise = 15.0
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    target = Waypoint(position=np.array([50.0, 0.0, 5000.0]))  # absurd climb demand
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}
    min_airspeed_mps = TwinWingAirframe().capabilities.min_airspeed_mps
    max_pitch_rad = TecsGains().max_pitch_rad

    dt = 0.02
    min_airspeed_seen = float("inf")
    max_pitch_cmd_seen = 0.0
    for _ in range(4000):  # 80 s
        state = SimState(
            position=dyn.position.copy(),
            velocity=dyn.velocity.copy(),
            attitude_quat=dyn.attitude_quat.copy(),
            body_rates=dyn.body_rates.copy(),
        )
        control_target = controller.compute(state, target, cfg=cfg)
        max_pitch_cmd_seen = max(max_pitch_cmd_seen, abs(control_target.metadata["pitch_cmd"]))
        wrench = control_target.metadata["wrench"]
        throttle = float(np.clip(wrench.force_body[0], 0.0, 10.0))
        roll_m, pitch_m, _yaw_m = wrench.moment_body
        max_elevon = np.radians(25.0)
        delta_pitch = float(np.clip(pitch_m, -max_elevon, max_elevon))
        delta_roll = float(np.clip(roll_m, -(max_elevon - abs(delta_pitch)), max_elevon - abs(delta_pitch)))
        actuator_cmd = np.array(
            [throttle, throttle, delta_pitch + delta_roll, delta_pitch - delta_roll], dtype=float
        )
        dyn.step(actuator_cmd, dt)
        airspeed, _, _ = dyn.airspeed_alpha_beta()
        min_airspeed_seen = min(min_airspeed_seen, airspeed)

    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.body_rates))
    assert max_pitch_cmd_seen <= max_pitch_rad + 1e-9, "pitch command should never exceed the clamp"
    assert min_airspeed_seen > min_airspeed_mps * 1.2, (
        f"airspeed dipped to {min_airspeed_seen:.1f} m/s chasing an impossible climb -- "
        "the pitch clamp should have prevented this"
    )


def test_motor_out_stays_controllable_in_roll_pitch():
    """Plan 04's motor-out acceptance test: one motor at zero -- remains
    controllable in roll/pitch, yaw degrades gracefully.

    Differential thrust is this airframe's *only* yaw authority (no rudder),
    so losing a motor means losing yaw authority almost entirely -- the plan
    explicitly expects yaw to degrade, not stay perfect. What must hold is
    roll/pitch/altitude staying controlled (finite, bounded, not tumbling)
    despite the sustained asymmetric-thrust disturbance.
    """
    dyn = FixedWingDynamics()
    cruise = 15.0
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    target = Waypoint(position=np.array([2000.0, 40.0, 100.0]))
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}
    min_airspeed_mps = TwinWingAirframe().capabilities.min_airspeed_mps

    dt = 0.02
    for _ in range(3000):  # 60 s
        state = SimState(
            position=dyn.position.copy(),
            velocity=dyn.velocity.copy(),
            attitude_quat=dyn.attitude_quat.copy(),
            body_rates=dyn.body_rates.copy(),
        )
        control_target = controller.compute(state, target, cfg=cfg)
        wrench = control_target.metadata["wrench"]
        throttle = float(np.clip(wrench.force_body[0], 0.0, 10.0))
        roll_m, pitch_m, _yaw_m = wrench.moment_body
        max_elevon = np.radians(25.0)
        delta_pitch = float(np.clip(pitch_m, -max_elevon, max_elevon))
        delta_roll = float(np.clip(roll_m, -(max_elevon - abs(delta_pitch)), max_elevon - abs(delta_pitch)))
        # Right motor failed: zero regardless of what the controller commands.
        actuator_cmd = np.array([throttle, 0.0, delta_pitch + delta_roll, delta_pitch - delta_roll], dtype=float)
        dyn.step(actuator_cmd, dt)

    pitch, bank = body_axis_pitch_bank(dyn.attitude_quat)
    airspeed, _, _ = dyn.airspeed_alpha_beta()
    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.body_rates))
    assert np.linalg.norm(dyn.body_rates) < 0.1, "should have settled, not tumbling"
    assert abs(np.degrees(pitch)) < 20.0, "pitch should stay controlled despite one motor out"
    assert abs(np.degrees(bank)) < 20.0, "roll should stay controlled despite one motor out"
    assert abs(dyn.position[2] - 100.0) < 10.0, "altitude should stay roughly held"
    assert airspeed > min_airspeed_mps * 1.2, "should not have been driven toward stall"
