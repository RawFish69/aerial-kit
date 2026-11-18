"""Closed-loop Dubins/L1 mission over SingleWingDynamics."""

from __future__ import annotations

import math

import numpy as np

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.airframes.single_wing_airframe import SingleWingAirframe
from aerial_kit.controllers.single_wing import (
    SingleWingL1TECSController,
    single_wing_controller_config,
)
from aerial_kit.dynamics.rotations import rotmat_to_quat
from aerial_kit.dynamics.single_wing import SingleWingDynamics
from aerial_kit.types import SimState, Waypoint
from sim_py.planners.dubins import plan_dubins_path, sample_dubins_path


def _trimmed_start(
    position: np.ndarray,
    heading_rad: float,
    cruise_airspeed_mps: float,
    dyn: SingleWingDynamics,
) -> np.ndarray:
    trim = dyn.compute_trim_detailed(cruise_airspeed_mps)
    alpha = trim["alpha_rad"]
    c, s = math.cos(alpha), math.sin(alpha)
    ch, sh = math.cos(heading_rad), math.sin(heading_rad)
    rotation = np.column_stack((
        np.array([c * ch, c * sh, s]),
        np.array([sh, -ch, 0.0]),
        np.array([s * ch, s * sh, -c]),
    ))
    dyn.position = position.copy()
    dyn.velocity = np.array([
        cruise_airspeed_mps * ch,
        cruise_airspeed_mps * sh,
        0.0,
    ])
    dyn.attitude_quat = rotmat_to_quat(rotation)
    dyn.thrust_n = trim["thrust_n"]
    return dyn.compute_trim(cruise_airspeed_mps)


def test_dubins_l1_waypoint_mission_converges() -> None:
    cruise = 14.0
    spec = load_single_wing_spec()
    dyn = SingleWingDynamics(spec.dynamics_params())
    airframe = SingleWingAirframe(spec=spec, cruise_airspeed_mps=cruise)
    controller = SingleWingL1TECSController()
    cfg = single_wing_controller_config(cruise, spec)
    cfg["controller"]["l1_tecs"]["l1_distance_m"] = 32.0

    start = np.array([0.0, 0.0, 100.0])
    goal = np.array([220.0, 60.0, 100.0])
    start_heading = 0.0
    goal_heading = 0.9
    turn_radius = float(airframe.capabilities.min_turn_radius_m) * 1.10
    path = plan_dubins_path(
        (start[0], start[1], start_heading),
        (goal[0], goal[1], goal_heading),
        turn_radius,
    )
    samples = sample_dubins_path(
        (start[0], start[1], start_heading), path, 4.0)

    _trimmed_start(start, start_heading, cruise, dyn)
    max_cross_track_m = 0.0
    reached = False
    dt = 0.02
    for _ in range(1800):
        airspeed, _, _ = dyn.airspeed_alpha_beta()
        state = SimState(
            position=dyn.position.copy(),
            velocity=dyn.velocity.copy(),
            airspeed_mps=airspeed,
            attitude_quat=dyn.attitude_quat.copy(),
            body_rates=dyn.body_rates.copy(),
        )
        nearest = int(np.argmin(np.sum(
            (samples[:, :2] - dyn.position[:2]) ** 2, axis=1)))
        cross_track = float(np.linalg.norm(
            samples[nearest, :2] - dyn.position[:2]))
        max_cross_track_m = max(max_cross_track_m, cross_track)

        if (
            nearest >= len(samples) - 2
            and np.linalg.norm(dyn.position - goal) < 12.0
        ):
            reached = True
            break

        target_index = min(nearest + 8, len(samples) - 1)
        target = Waypoint(position=np.array([
            samples[target_index, 0],
            samples[target_index, 1],
            goal[2],
        ]))
        result = controller.compute(state, target, cfg)
        assert result.wrench is not None
        actuator_cmd = airframe.allocate(result.wrench, state)
        dyn.step(actuator_cmd, dt)

    assert reached, (
        f"mission did not converge: position={dyn.position}, "
        f"distance={np.linalg.norm(dyn.position - goal):.1f} m")
    assert np.linalg.norm(dyn.position - goal) < 12.0
    assert max_cross_track_m < 10.0
    assert abs(dyn.position[2] - 100.0) < 2.0
    airspeed, _, beta = dyn.airspeed_alpha_beta()
    assert 12.0 < airspeed < 17.0
    assert abs(beta) < math.radians(15.0)
    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.velocity))
    assert np.all(np.isfinite(dyn.body_rates))
