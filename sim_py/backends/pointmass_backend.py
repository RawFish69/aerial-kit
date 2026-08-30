"""Point-mass dynamics backend (default)."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..core.interfaces import DynamicsBackend
from ..core.types import ControlTarget, SimState
from aerial_kit.dynamics.pointmass import PointMassDynamics, PointMassParams


class PointMassBackend(DynamicsBackend):
    """Simple point-mass backend that matches legacy simulator behavior."""

    def __init__(self) -> None:
        self._dyn: PointMassDynamics | None = None
        self._t = 0.0

    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        sim_cfg = dict(cfg.get("simulation", {}) or {})
        pm_cfg = dict(sim_cfg.get("pointmass", {}) or {})

        params = PointMassParams(
            mass=float(pm_cfg.get("mass", 1.0)),
            kv_drag=float(pm_cfg.get("kv_drag", 0.1)),
        )
        self._dyn = PointMassDynamics(params)
        self._dyn.position = np.asarray(initial_state.position, dtype=float).copy()
        self._dyn.velocity = np.asarray(initial_state.velocity, dtype=float).copy()
        self._t = float(initial_state.t)

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._dyn is None:
            raise RuntimeError("PointMassBackend.reset() must be called before step().")
        self._dyn.step(np.asarray(control_target.accel_cmd, dtype=float), float(dt))
        self._t += float(dt)

    def state(self) -> SimState:
        if self._dyn is None:
            raise RuntimeError("PointMassBackend.reset() must be called before state().")
        return SimState(
            position=self._dyn.position.copy(),
            velocity=self._dyn.velocity.copy(),
            t=float(self._t),
        )

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._dyn is None:
            raise RuntimeError("PointMassBackend.reset() must be called before apply_constraints().")

        self._dyn.position = np.clip(
            self._dyn.position,
            np.asarray(min_bounds, dtype=float),
            np.asarray(max_bounds, dtype=float),
        )

        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(float(self._dyn.position[0]), float(self._dyn.position[1])))
            min_z = ground + float(terrain_clearance)
            if self._dyn.position[2] < min_z:
                self._dyn.position[2] = min_z
