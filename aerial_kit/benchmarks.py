"""Reproducible controller benchmarks on Aerial Kit's Python dynamics.

The harness deliberately uses the public controller and dynamics objects rather
than a separate performance model. Position controllers fly ``PointMassDynamics``;
wrench controllers fly the actuator-level quad-X plant; wing controllers fly the
existing fixed-wing 6-DOF model through their airframe allocator.

Run all scenarios and write a Markdown table::

    python -m aerial_kit.benchmarks --output docs/evidence/controller-benchmarks.md
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

from .airframes.fixed_wing import TwinWingAirframe
from .airframes.multirotor import MultirotorAirframe
from .airframes.single_wing_airframe import SingleWingAirframe
from .dynamics.actuator_loop import zup_state_of
from .dynamics.fixed_wing import FixedWingDynamics, FixedWingParams, level_attitude_quat
from .dynamics.multirotor_actuator import ActuatorPlant
from .dynamics.pointmass import PointMassDynamics
from .dynamics.quad_x_seam import motor_commands
from .dynamics.single_wing import SingleWingDynamics
from .registry import create_controller, register_builtin_components
from .types import CommandKind, SimState, Waypoint

ACCEL_CONTROLLERS = ("pid", "lqr", "mpc", "mppi", "constrained_mpc", "warm_mppi")
WRENCH_CONTROLLERS = ("cascade", "geometric", "nmpc")
WING_CONTROLLERS = ("l1_tecs", "single_wing_l1_tecs")


@dataclass(frozen=True)
class Scenario:
    """Reference trajectory and disturbance schedule shared by every run."""

    name: str
    duration_s: float
    reference: Callable[[float], np.ndarray]
    wind_mps: Callable[[float], np.ndarray]


@dataclass(frozen=True)
class BenchmarkResult:
    """One controller/scenario result."""

    controller: str
    family: str
    scenario: str
    rms_error_m: float
    final_error_m: float
    effort: float
    finite: bool
    error: str = ""


def benchmark_scenarios() -> tuple[Scenario, ...]:
    """Return the hover, step, figure-eight and gust scenarios."""
    origin = np.array([0.0, 0.0, 2.0])

    def hold(_: float) -> np.ndarray:
        return origin.copy()

    def step(t: float) -> np.ndarray:
        return origin + np.array([5.0 if t >= 2.0 else 0.0, 0.0, 0.0])

    def figure_eight(t: float) -> np.ndarray:
        w = 2.0 * math.pi / 12.0
        return origin + np.array([5.0 * math.sin(w * t), 2.5 * (1.0 - math.cos(w * t)), 0.0])

    calm = lambda _: np.zeros(3)  # noqa: E731
    gust = lambda t: np.array([2.5, 0.0, 0.0]) if t >= 3.0 else np.zeros(3)  # noqa: E731
    return (
        Scenario("hover_hold", 12.0, hold, calm),
        Scenario("step", 12.0, step, calm),
        Scenario("figure8", 15.0, figure_eight, calm),
        Scenario("wind_gust", 12.0, hold, gust),
    )


def _controller_config(name: str) -> dict[str, object]:
    inertia = [0.01, 0.01, 0.02]
    sections: dict[str, object] = {
        "pid": {"kp": 0.9, "kd": 1.4, "max_acc_mps2": 5.0},
        "lqr": {"q_pos": 10.0, "q_vel": 2.0, "r_acc": 1.0, "max_acc_mps2": 5.0},
        "mpc": {"q_pos": 8.0, "q_vel": 2.0, "r_acc": 10.0, "horizon": 8},
        "mppi": {"samples": 64, "horizon": 8, "seed": 7, "max_acc_mps2": 5.0},
        "constrained_mpc": {"max_speed_xy": 4.0, "max_accel_xy": 4.0},
        "warm_mppi": {"samples": 48, "horizon": 8, "seed": 7},
        "cascade": {
            "mass_kg": 1.0,
            "inertia_kgm2": inertia,
            "max_tilt_deg": 35.0,
        },
        "geometric": {"mass_kg": 1.0, "inertia_kgm2": inertia},
        "nmpc": {
            "mass_kg": 1.0,
            "inertia_kgm2": inertia,
            "horizon": 12,
            "max_iter": 5,
        },
        "l1_tecs": {},
        "single_wing_l1_tecs": {},
    }
    return {
        "controller": {name: sections.get(name, {})},
    }


def _state_of(dynamics: PointMassDynamics | FixedWingDynamics | SingleWingDynamics) -> SimState:
    attitude = getattr(dynamics, "attitude_quat", None)
    rates = getattr(dynamics, "body_rates", None)
    airspeed = None
    if hasattr(dynamics, "airspeed_alpha_beta"):
        airspeed = float(dynamics.airspeed_alpha_beta()[0])
    return SimState(
        position=np.asarray(dynamics.position, dtype=float).copy(),
        velocity=np.asarray(dynamics.velocity, dtype=float).copy(),
        t=float(getattr(dynamics, "time_s", 0.0)),
        attitude_quat=None if attitude is None else np.asarray(attitude, dtype=float).copy(),
        body_rates=None if rates is None else np.asarray(rates, dtype=float).copy(),
        airspeed_mps=airspeed,
    )


def _summarize(
    name: str,
    family: str,
    scenario: Scenario,
    errors: Sequence[float],
    efforts: Sequence[float],
    finite: bool,
    error: str = "",
) -> BenchmarkResult:
    return BenchmarkResult(
        controller=name,
        family=family,
        scenario=scenario.name,
        rms_error_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else math.inf,
        final_error_m=float(errors[-1]) if errors else math.inf,
        effort=float(np.mean(efforts)) if efforts else math.inf,
        finite=bool(finite and all(math.isfinite(v) for v in errors)),
        error=error,
    )


def _run_accel(name: str, scenario: Scenario, dt: float = 0.02) -> BenchmarkResult:
    dynamics = PointMassDynamics()
    dynamics.position[:] = [0.0, 0.0, 2.0]
    controller = create_controller(name)
    cfg = _controller_config(name)
    errors: list[float] = []
    efforts: list[float] = []
    with np.errstate(all="ignore"):
        for step in range(int(scenario.duration_s / dt) + 1):
            t = step * dt
            reference = scenario.reference(t)
            state = _state_of(dynamics)
            target = controller.compute(state, Waypoint(reference), cfg)
            disturbance = scenario.wind_mps(t)
            dynamics.step(np.asarray(target.accel_cmd) + disturbance, dt)
            errors.append(float(np.linalg.norm(_state_of(dynamics).position - reference)))
            efforts.append(float(np.linalg.norm(target.accel_cmd)))
    finite = bool(np.all(np.isfinite(dynamics.position)) and np.all(np.isfinite(dynamics.velocity)))
    return _summarize(name, "accel", scenario, errors, efforts, finite)


def _run_wrench(name: str, scenario: Scenario, dt: float = 0.01) -> BenchmarkResult:
    plant = ActuatorPlant()
    plant.position[:] = [0.0, 0.0, -2.0]
    plant.set_hover()
    airframe = MultirotorAirframe(arms=4, layout="x")
    controller = create_controller(name)
    cfg = _controller_config(name)
    errors: list[float] = []
    efforts: list[float] = []
    with np.errstate(all="ignore"):
        for step in range(int(scenario.duration_s / dt) + 1):
            t = step * dt
            reference = scenario.reference(t)
            state = zup_state_of(plant)
            state.t = t
            target = controller.compute(state, Waypoint(reference), cfg)
            if target.wrench is None:
                raise ValueError(f"{name} produced no wrench")
            motors = motor_commands(airframe, target.wrench, plant)
            plant.set_motors(motors)
            wind_ned = scenario.wind_mps(t)
            plant.params.wind_ned_mps = (-wind_ned[1], -wind_ned[0], -wind_ned[2])
            plant.step(dt)
            errors.append(float(np.linalg.norm(zup_state_of(plant).position - reference)))
            efforts.append(float(np.mean(np.abs(motors))))
    finite = bool(np.all(np.isfinite(plant.position)) and np.all(np.isfinite(plant.velocity)))
    return _summarize(name, "wrench", scenario, errors, efforts, finite)


def _wing_dynamics(name: str, scenario: Scenario) -> FixedWingDynamics | SingleWingDynamics:
    if name == "single_wing_l1_tecs":
        dynamics: FixedWingDynamics | SingleWingDynamics = SingleWingDynamics()
    else:
        dynamics = FixedWingDynamics(FixedWingParams())
    dynamics.position[:] = [0.0, 0.0, 20.0]
    dynamics.velocity[:] = [15.0, 0.0, 0.0]
    dynamics.attitude_quat = level_attitude_quat()
    dynamics.body_rates[:] = 0.0
    return dynamics


def _wing_reference(scenario: Scenario, t: float) -> np.ndarray:
    """Wing-safe references: a wing cannot hover at a fixed world point."""
    if scenario.name in {"hover_hold", "wind_gust"}:
        return np.array([15.0 * t, 0.0, 20.0])
    if scenario.name == "step":
        return np.array([15.0 * t, 5.0 if t >= 2.0 else 0.0, 20.0])
    if scenario.name == "figure8":
        w = 2.0 * math.pi / 12.0
        return np.array([30.0 * math.sin(w * t), 10.0 * math.sin(2.0 * w * t), 20.0])
    return np.array([0.0, 0.0, 20.0])


def _run_wing(name: str, scenario: Scenario, dt: float = 0.01) -> BenchmarkResult:
    dynamics = _wing_dynamics(name, scenario)
    airframe = SingleWingAirframe() if name == "single_wing_l1_tecs" else TwinWingAirframe()
    controller = create_controller(name)
    cfg = _controller_config(name)
    errors: list[float] = []
    efforts: list[float] = []
    with np.errstate(all="ignore"):
        for step in range(int(scenario.duration_s / dt) + 1):
            t = step * dt
            reference = _wing_reference(scenario, t)
            state = _state_of(dynamics)
            state.t = t
            target = controller.compute(state, Waypoint(reference), cfg)
            if target.wrench is None:
                raise ValueError(f"{name} produced no wrench")
            actuators = airframe.allocate(target.wrench, state)
            dynamics.step(actuators, dt)
            dynamics.wind_mps[:] = scenario.wind_mps(t)
            errors.append(float(np.linalg.norm(_state_of(dynamics).position - reference)))
            efforts.append(float(np.mean(np.abs(actuators))))
    finite = bool(np.all(np.isfinite(dynamics.position)) and np.all(np.isfinite(dynamics.velocity)))
    return _summarize(name, "wing", scenario, errors, efforts, finite)


def run_controller_benchmarks(
    controllers: Sequence[str] | None = None,
    scenarios: Sequence[Scenario] | None = None,
) -> list[BenchmarkResult]:
    """Run selected controllers and scenarios, preserving failures as rows."""
    register_builtin_components()
    selected = tuple(controllers) if controllers is not None else (
        ACCEL_CONTROLLERS + WRENCH_CONTROLLERS + WING_CONTROLLERS
    )
    scenario_list = tuple(scenarios) if scenarios is not None else benchmark_scenarios()
    results: list[BenchmarkResult] = []
    for name in selected:
        if name in ACCEL_CONTROLLERS:
            runner = _run_accel
        elif name in WRENCH_CONTROLLERS:
            runner = _run_wrench
        elif name in WING_CONTROLLERS:
            runner = _run_wing
        else:
            raise ValueError(f"unknown benchmark controller {name!r}")
        for scenario in scenario_list:
            try:
                results.append(runner(name, scenario))
            except Exception as exc:  # a benchmark reports a failed row, not a missing one
                results.append(BenchmarkResult(
                    controller=name,
                    family="accel" if name in ACCEL_CONTROLLERS else
                    "wrench" if name in WRENCH_CONTROLLERS else "wing",
                    scenario=scenario.name,
                    rms_error_m=math.inf,
                    final_error_m=math.inf,
                    effort=math.inf,
                    finite=False,
                    error=f"{type(exc).__name__}: {exc}",
                ))
    return results


def markdown_table(results: Sequence[BenchmarkResult]) -> str:
    """Render benchmark results as a deterministic Markdown table."""
    lines = [
        "| Controller | Family | Scenario | RMS error (m) | Final error (m) | Effort | Finite | Note |",
        "| --- | --- | --- | ---: | ---: | ---: | :---: | --- |",
    ]
    for result in results:
        lines.append(
            f"| {result.controller} | {result.family} | {result.scenario} | "
            f"{result.rms_error_m:.3f} | {result.final_error_m:.3f} | "
            f"{result.effort:.3f} | {'yes' if result.finite else 'no'} | {result.error} |"
        )
    return "\n".join(lines) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    """Run the benchmark CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write a Markdown result table")
    parser.add_argument(
        "--controller",
        action="append",
        choices=ACCEL_CONTROLLERS + WRENCH_CONTROLLERS + WING_CONTROLLERS,
    )
    args = parser.parse_args(argv)
    results = run_controller_benchmarks(args.controller)
    table = markdown_table(results)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(table, encoding="utf-8")
    print(table, end="")
    return 0 if all(result.finite for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
