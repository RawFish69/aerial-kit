"""Standalone single-motor flying-wing Dubins/L1 mission.

This example deliberately instantiates the single-wing components directly.
They can be added to the shared registry later without making this runnable
specification depend on that merge.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.airframes.single_wing_airframe import SingleWingAirframe
from aerial_kit.controllers.single_wing import (
    SingleWingL1TECSController,
    single_wing_controller_config,
)
from aerial_kit.dynamics.rotations import rotmat_to_quat
from aerial_kit.dynamics.single_wing import SingleWingDynamics
from aerial_kit.sim import load_config
from aerial_kit.types import SimState, Waypoint
from sim_py.planners.dubins import plan_dubins_path, sample_dubins_path


def _attitude_for_alpha_heading(
    alpha_rad: float, heading_rad: float
) -> np.ndarray:
    """Body-FRD to world-ENU attitude for horizontal air-relative velocity."""
    c, s = math.cos(alpha_rad), math.sin(alpha_rad)
    ch, sh = math.cos(heading_rad), math.sin(heading_rad)
    rotation = np.column_stack((
        np.array([c * ch, c * sh, s]),
        np.array([sh, -ch, 0.0]),
        np.array([s * ch, s * sh, -c]),
    ))
    return rotmat_to_quat(rotation)


def _deep_merge(base: dict, override: dict) -> dict:
    """Overlay YAML controller values without dropping derived defaults."""
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(dict(result[key]), value)
        else:
            result[key] = value
    return result


def _plot(
    trajectory: np.ndarray,
    samples: np.ndarray,
    start: np.ndarray,
    goal: np.ndarray,
    visual: dict,
    *,
    show: bool,
    save: Path | None,
) -> None:
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(10, 7))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(
        samples[:, 0], samples[:, 1], samples[:, 2],
        color=str(visual.get("path_color", "darkorange")),
        linewidth=float(visual.get("planned_linewidth", 2.0)),
        label="Dubins path",
    )
    axis.plot(
        trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
        color="royalblue", linewidth=2.2, label="flown",
    )
    marker_size = float(visual.get("marker_size", 45))
    axis.scatter(*start, color="green", s=marker_size, label="start")
    axis.scatter(*goal, color="red", s=marker_size, label="goal")
    axis.set_xlabel("east [m]")
    axis.set_ylabel("north [m]")
    axis.set_zlabel("up [m]")
    axis.set_title("Aerial Kit single-motor flying wing")
    axis.legend()
    axis.grid(True)

    if save is not None:
        save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(save, dpi=160, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fly the aerial-kit single-motor flying-wing example.")
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--save", type=Path, help="Save the mission plot.")
    parser.add_argument(
        "--no-show", action="store_true", help="Do not open a plot window.")
    args = parser.parse_args()

    config = load_config(args.config)
    raw = config.raw_cfg
    path_cfg = dict(raw.get("path", {}) or {})
    simulation = dict(config.simulation_cfg)
    wing_cfg = dict(simulation.get("singlewing", {}) or {})
    spec_path = wing_cfg.get("airframe_spec")
    if spec_path:
        candidate = Path(spec_path)
        if not candidate.is_absolute():
            candidate = config.sim_config_path.parent / candidate
        spec_path = candidate
    spec = load_single_wing_spec(spec_path)

    cruise = float(
        raw.get("controller", {}).get("l1_tecs", {}).get(
            "cruise_airspeed_mps", 14.0))
    controller_cfg = _deep_merge(
        single_wing_controller_config(cruise, spec),
        dict(raw.get("controller", {}) or {}),
    )
    airframe = SingleWingAirframe(spec=spec, cruise_airspeed_mps=cruise)
    controller = SingleWingL1TECSController()

    start = np.asarray(
        path_cfg.get("start_position_m", [0.0, 0.0, 100.0]), dtype=float)
    goal = np.asarray(
        path_cfg.get("goal_position_m", [220.0, 60.0, 100.0]), dtype=float)
    start_heading = float(path_cfg.get(
        "start_heading_rad",
        math.atan2(goal[1] - start[1], goal[0] - start[0]),
    ))
    goal_heading = float(path_cfg.get(
        "goal_heading_rad",
        math.atan2(goal[1] - start[1], goal[0] - start[0]),
    ))

    wind = np.asarray(
        wing_cfg.get("wind_mps", [0.0, 0.0, 0.0]), dtype=float)
    dynamics = SingleWingDynamics(spec.dynamics_params(), wind_mps=wind)
    trim = dynamics.compute_trim_detailed(cruise)
    dynamics.position = start.copy()
    dynamics.velocity = np.array([
        cruise * math.cos(start_heading),
        cruise * math.sin(start_heading),
        0.0,
    ])
    dynamics.attitude_quat = _attitude_for_alpha_heading(
        trim["alpha_rad"], start_heading)
    dynamics.thrust_n = trim["thrust_n"]

    turn_radius = float(path_cfg.get(
        "turn_radius_m", airframe.capabilities.min_turn_radius_m))
    step_size = float(path_cfg.get("dubins_step_size_m", 4.0))
    dubins = plan_dubins_path(
        (start[0], start[1], start_heading),
        (goal[0], goal[1], goal_heading),
        turn_radius,
    )
    planar_samples = sample_dubins_path(
        (start[0], start[1], start_heading), dubins, step_size)
    samples = planar_samples.copy()
    samples[:, 2] = np.linspace(start[2], goal[2], len(samples))

    dt = float(simulation.get("dt", 0.02))
    duration = float(simulation.get("duration", 45.0))
    lookahead = max(0, int(path_cfg.get("lookahead_waypoints", 8)))
    acceptance = float(path_cfg.get("waypoint_acceptance_m", 12.0))
    trajectory = [dynamics.position.copy()]
    reached = False
    max_cross_track_m = 0.0

    for _ in range(int(duration / dt)):
        airspeed, _, _ = dynamics.airspeed_alpha_beta()
        state = SimState(
            position=dynamics.position.copy(),
            velocity=dynamics.velocity.copy(),
            airspeed_mps=airspeed,
            attitude_quat=dynamics.attitude_quat.copy(),
            body_rates=dynamics.body_rates.copy(),
        )
        nearest = int(np.argmin(np.sum(
            (samples[:, :2] - dynamics.position[:2]) ** 2, axis=1)))
        max_cross_track_m = max(
            max_cross_track_m,
            float(np.linalg.norm(
                samples[nearest, :2] - dynamics.position[:2])),
        )
        if (
            nearest >= len(samples) - 2
            and np.linalg.norm(dynamics.position - goal) < acceptance
        ):
            reached = True
            break

        target_index = min(nearest + lookahead, len(samples) - 1)
        target = Waypoint(position=samples[target_index].copy())
        result = controller.compute(state, target, controller_cfg)
        if result.wrench is None:
            raise RuntimeError("single-wing controller returned no wrench")
        actuator_cmd = airframe.allocate(result.wrench, state)
        dynamics.step(actuator_cmd, dt)
        trajectory.append(dynamics.position.copy())

    trajectory_array = np.asarray(trajectory)
    final_error = float(np.linalg.norm(dynamics.position - goal))
    final_airspeed, _, _ = dynamics.airspeed_alpha_beta()
    print(f"Dubins segments: {[segment.mode for segment in dubins.segments]}")
    print(f"Mission complete: {reached}")
    print(f"Final position: {dynamics.position.round(2).tolist()} m")
    print(f"Goal error: {final_error:.2f} m")
    print(f"Peak cross-track error: {max_cross_track_m:.2f} m")
    print(f"Final airspeed: {final_airspeed:.2f} m/s")

    if not reached:
        raise SystemExit("mission did not reach the goal inside the configured duration")
    if args.save is not None or not args.no_show:
        _plot(
            trajectory_array,
            samples,
            start,
            goal,
            dict(raw.get("visual", {}) or {}),
            show=not args.no_show,
            save=args.save,
        )


if __name__ == "__main__":
    main()
