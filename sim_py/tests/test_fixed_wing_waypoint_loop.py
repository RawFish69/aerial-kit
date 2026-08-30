"""Plan 04's waypoint-loop acceptance test: a 4-waypoint circuit flown with
Dubins paths, cross-track error bounded, closed over the real 6-DOF
FixedWingDynamics + FixedWingL1TECSController.

Not driven through ``runner.py``'s single start/goal mission loop -- concatenating
several ``DubinsPlanner`` legs into one circuit and pure-pursuit-following the
result is a standalone integration of pieces `runner.py` doesn't itself support yet
(multi-waypoint circuits), matching the style of the other closed-loop tests in
``test_fixed_wing_guidance.py``. ``DubinsPlanner`` itself (registered as ``"dubins"``
in the registry, and exercised end-to-end through a real `--airframe twin_wing`
CLI/`run_simulation()` single-leg mission in
``test_dubins_planner_produces_curvature_feasible_single_leg_path`` /
``test_runner_integration.py``) is the actual "wired into planner selection" unit.
"""

from __future__ import annotations

import numpy as np

from aerial_kit.airframes.fixed_wing import TwinWingAirframe
from aerial_kit.controllers.fixed_wing import FixedWingL1TECSController
from aerial_kit.dynamics.fixed_wing import FixedWingDynamics, level_attitude_quat
from sim_py.core.types import SimState, Waypoint
from sim_py.planners.dubins import plan_dubins_path, sample_dubins_path


def _square_circuit_path(turn_radius_m: float, side_m: float, step_size_m: float) -> np.ndarray:
    """Concatenate 4 Dubins legs around a square, each leg's start heading
    matching the previous leg's arrival heading for continuity."""
    corners = [(0.0, 0.0), (side_m, 0.0), (side_m, side_m), (0.0, side_m), (0.0, 0.0)]
    legs = []
    heading = None
    for (sx, sy), (gx, gy) in zip(corners, corners[1:]):
        bearing = float(np.arctan2(gy - sy, gx - sx))
        start_pose = (sx, sy, heading if heading is not None else bearing)
        goal_pose = (gx, gy, bearing)
        dubins_path = plan_dubins_path(start_pose, goal_pose, turn_radius_m)
        samples = sample_dubins_path(start_pose, dubins_path, step_size_m)
        legs.append(samples[:, :2])
        heading = float(samples[-1, 2])
    return np.vstack(legs)


def test_dubins_waypoint_loop_bounded_cross_track_error():
    cruise = 15.0
    airframe = TwinWingAirframe(cruise_airspeed_mps=cruise)
    turn_radius_m = airframe.capabilities.min_turn_radius_m
    assert turn_radius_m is not None

    full_path = _square_circuit_path(turn_radius_m, side_m=300.0, step_size_m=5.0)

    dyn = FixedWingDynamics()
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(0.0)
    trim_cmd = dyn.compute_trim(cruise)

    controller = FixedWingL1TECSController()
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": cruise, "trim_throttle_n": float(trim_cmd[0])}}}

    dt = 0.02
    lookahead_points = 8  # ~40 m ahead at a 5 m sample spacing, matching l1_distance_m default
    max_cross_track_m = 0.0
    reached_end = False
    for _ in range(8000):  # 160 s cap
        pos_xy = dyn.position[:2]
        dist_sq = np.sum((full_path - pos_xy) ** 2, axis=1)
        nearest_idx = int(np.argmin(dist_sq))
        max_cross_track_m = max(max_cross_track_m, float(np.sqrt(dist_sq[nearest_idx])))

        if nearest_idx >= len(full_path) - 5:
            reached_end = True
            break

        target_idx = min(nearest_idx + lookahead_points, len(full_path) - 1)
        target_xy = full_path[target_idx]
        target = Waypoint(position=np.array([target_xy[0], target_xy[1], 100.0]))

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
        actuator_cmd = np.array(
            [throttle, throttle, delta_pitch + delta_roll, delta_pitch - delta_roll], dtype=float
        )
        dyn.step(actuator_cmd, dt)

    assert np.all(np.isfinite(dyn.position))
    assert reached_end, "should have completed the full 4-leg circuit within the time cap"
    assert max_cross_track_m < 30.0, f"cross-track error peaked at {max_cross_track_m:.1f} m"
