"""Analytic 6-DOF backend for the single-motor flying wing."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.dynamics.fixed_wing import level_attitude_quat
from aerial_kit.dynamics.single_wing import SingleWingDynamics, SingleWingParams
from sim_py.core.interfaces import DynamicsBackend
from sim_py.core.types import CommandKind, ControlTarget, SimState


def _params_from_config(cfg: Mapping[str, Any]) -> SingleWingParams:
    simulation = dict(cfg.get("simulation", {}) or {})
    wing_cfg = dict(simulation.get("singlewing", {}) or {})
    spec = load_single_wing_spec(wing_cfg.get("airframe_spec") or None)
    values = spec.dynamics_param_overrides()
    fields = SingleWingParams.__dataclass_fields__
    for key, value in wing_cfg.items():
        if key not in fields:
            continue
        if key in ("inertia_kg_m2", "motor_position_m", "thrust_axis"):
            value = np.asarray(value, dtype=float)
        values[key] = value
    return SingleWingParams(**values)


class SingleWingBackend(DynamicsBackend):
    """Actuator-level ``[thrust, elevon_left, elevon_right]`` backend."""

    command_kind = CommandKind.AIRSPEED_NAV

    def __init__(self) -> None:
        self._dyn: SingleWingDynamics | None = None
        self._t = 0.0

    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        del world
        simulation = dict(cfg.get("simulation", {}) or {})
        wing_cfg = dict(simulation.get("singlewing", {}) or {})
        wind = np.asarray(
            wing_cfg.get("wind_mps", [0.0, 0.0, 0.0]), dtype=float)
        self._dyn = SingleWingDynamics(_params_from_config(cfg), wind_mps=wind)
        self._dyn.position = np.asarray(
            initial_state.position, dtype=float).reshape(3).copy()
        self._dyn.velocity = np.asarray(
            initial_state.velocity, dtype=float).reshape(3).copy()
        self._dyn.attitude_quat = (
            np.asarray(initial_state.attitude_quat, dtype=float).reshape(4).copy()
            if initial_state.attitude_quat is not None
            else level_attitude_quat()
        )
        self._dyn.body_rates = (
            np.asarray(initial_state.body_rates, dtype=float).reshape(3).copy()
            if initial_state.body_rates is not None
            else np.zeros(3, dtype=float)
        )
        self._dyn.thrust_n = 0.0
        self._t = float(initial_state.t)
        spec = load_single_wing_spec(wing_cfg.get("airframe_spec") or None)
        self._cruise = float(spec.flight_envelope["cruise_airspeed_mps"])

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._dyn is None:
            raise RuntimeError("SingleWingBackend.reset() must be called before step()")
        actuator_cmd = control_target.metadata.get("actuator_cmd")
        if actuator_cmd is None:
            raise ValueError(
                "SingleWingBackend requires "
                "ControlTarget.metadata['actuator_cmd'] with "
                "[thrust_n, elevon_left_rad, elevon_right_rad]")
        self._dyn.step(np.asarray(actuator_cmd, dtype=float), float(dt))
        self._t += float(dt)

    def controller_defaults(self, section: str) -> dict:
        """Spec-derived L1/TECS defaults: trim thrust and thrust limit."""
        if section != "l1_tecs" or self._dyn is None:
            return {}
        p = self._dyn.params
        envelope_cruise = getattr(self, "_cruise", None)
        trim = self._dyn.compute_trim_detailed(envelope_cruise or 14.0)
        return {"trim_throttle_n": float(trim["thrust_n"]), "max_thrust_n": float(p.max_thrust_n)}

    def state(self) -> SimState:
        if self._dyn is None:
            raise RuntimeError("SingleWingBackend.reset() must be called before state()")
        airspeed, _, _ = self._dyn.airspeed_alpha_beta()
        return SimState(
            position=self._dyn.position.copy(),
            velocity=self._dyn.velocity.copy(),
            t=self._t,
            attitude_quat=self._dyn.attitude_quat.copy(),
            body_rates=self._dyn.body_rates.copy(),
            airspeed_mps=airspeed,
        )

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._dyn is None:
            raise RuntimeError(
                "SingleWingBackend.reset() must be called before constraints")
        self._dyn.position = np.clip(
            self._dyn.position,
            np.asarray(min_bounds, dtype=float),
            np.asarray(max_bounds, dtype=float),
        )
        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(
                float(self._dyn.position[0]), float(self._dyn.position[1])))
            self._dyn.position[2] = max(
                float(self._dyn.position[2]), ground + float(terrain_clearance))
