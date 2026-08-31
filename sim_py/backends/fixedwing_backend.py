"""Twin-wing dynamics backend: 6-DOF rigid body + hand-rolled aero model.

Unlike ``PointMassBackend``, this backend needs actuator-level commands
(``[throttle_L, throttle_R, elevon_L, elevon_R]``, i.e. ``TwinWingAirframe.
allocate()``'s output) rather than an acceleration vector. The runner routes
the L1/TECS controller's Wrench through the airframe allocator and passes the
result via ``ControlTarget.metadata["actuator_cmd"]``. There is deliberately
no silent default actuator command.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..core.interfaces import DynamicsBackend
from ..core.types import ControlTarget, SimState
from aerial_kit.dynamics.fixed_wing import FixedWingDynamics, FixedWingParams, level_attitude_quat


class FixedWingBackend(DynamicsBackend):
    """6-DOF twin-wing backend driven by actuator commands, not accel_cmd."""

    def __init__(self) -> None:
        self._dyn: FixedWingDynamics | None = None
        self._t = 0.0

    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        sim_cfg = dict(cfg.get("simulation", {}) or {})
        fw_cfg = dict(sim_cfg.get("fixedwing", {}) or {})

        param_fields = set(FixedWingParams.__dataclass_fields__)
        params_kwargs = {k: v for k, v in fw_cfg.items() if k in param_fields and k != "inertia_kg_m2"}
        if "inertia_kg_m2" in fw_cfg:
            params_kwargs["inertia_kg_m2"] = np.asarray(fw_cfg["inertia_kg_m2"], dtype=float)
        params = FixedWingParams(**params_kwargs)

        wind_mps = np.asarray(fw_cfg.get("wind_mps", [0.0, 0.0, 0.0]), dtype=float)

        self._dyn = FixedWingDynamics(params=params, wind_mps=wind_mps)
        self._dyn.position = np.asarray(initial_state.position, dtype=float).copy()
        self._dyn.velocity = np.asarray(initial_state.velocity, dtype=float).copy()
        if initial_state.attitude_quat is not None:
            self._dyn.attitude_quat = np.asarray(initial_state.attitude_quat, dtype=float).copy()
        else:
            self._dyn.attitude_quat = level_attitude_quat()
        if initial_state.body_rates is not None:
            self._dyn.body_rates = np.asarray(initial_state.body_rates, dtype=float).copy()
        self._t = float(initial_state.t)

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._dyn is None:
            raise RuntimeError("FixedWingBackend.reset() must be called before step().")
        actuator_cmd = control_target.metadata.get("actuator_cmd")
        if actuator_cmd is None:
            raise ValueError(
                "FixedWingBackend requires control_target.metadata['actuator_cmd'] "
                "([throttle_L, throttle_R, elevon_L, elevon_R], e.g. from "
                "TwinWingAirframe.allocate()) -- no AIRSPEED_NAV guidance loop "
                "produces this through the generic accel_cmd path yet."
            )
        self._dyn.step(np.asarray(actuator_cmd, dtype=float), float(dt))
        self._t += float(dt)

    def state(self) -> SimState:
        if self._dyn is None:
            raise RuntimeError("FixedWingBackend.reset() must be called before state().")
        return SimState(
            position=self._dyn.position.copy(),
            velocity=self._dyn.velocity.copy(),
            t=float(self._t),
            attitude_quat=self._dyn.attitude_quat.copy(),
            body_rates=self._dyn.body_rates.copy(),
        )

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._dyn is None:
            raise RuntimeError("FixedWingBackend.reset() must be called before apply_constraints().")
        self._dyn.position = np.clip(
            self._dyn.position,
            np.asarray(min_bounds, dtype=float),
            np.asarray(max_bounds, dtype=float),
        )
