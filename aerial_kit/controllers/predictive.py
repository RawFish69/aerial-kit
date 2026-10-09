"""Simulator-facing wrappers for :class:`~.mpc.ConstrainedMPC` and :class:`~.mppi.MPPI`.

The runner calls ``compute`` every integration step (``dt`` is typically
0.01 s), but both of these plan on their own, coarser step (0.1 s by default).
Re-solving at 100 Hz a plan that assumes the command is held for 100 ms is
wasted work at best. These wrappers therefore solve once per *controller*
step and hold the first command in between - a zero-order hold, which is
exactly the assumption the plan was made under. A target change re-solves
immediately.

Both are registered by :func:`aerial_kit.registry.register_builtin_components`
as ``"constrained_mpc"`` and ``"warm_mppi"``; their settings live under
``controller.constrained_mpc`` and ``controller.warm_mppi`` respectively.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..interfaces import Controller
from ..types import ControlTarget, SimState, Waypoint
from .mpc import ConstrainedMPC
from .mppi import MPPI, SphereObstacle

_MPC_KEYS = {
    "dt", "horizon", "q_pos", "q_vel", "r_acc", "r_delta", "terminal",
    "max_accel_xy", "max_accel_z", "max_speed_xy", "max_speed_z",
    "speed_limit_xy", "disc_sides", "rho", "max_iter", "tol", "warm_start",
}
_MPPI_KEYS = {
    "dt", "horizon", "samples", "temperature", "noise_std", "q_pos", "q_vel",
    "q_terminal", "r_acc", "max_accel_xy", "max_accel_z", "max_speed",
    "speed_penalty", "obstacle_margin", "obstacle_penalty", "min_altitude",
    "altitude_penalty", "seed",
}


class _HeldPlanController(Controller):
    """Shared zero-order-hold logic. Subclasses build ``self._planner``."""

    name = ""

    def __init__(self) -> None:
        self._planner: Any = None
        self._held: np.ndarray | None = None
        self._held_target: np.ndarray | None = None
        self._held_t = -np.inf
        self.last_solution: Any = None

    def _build(self, cfg: Mapping[str, Any]) -> Any:  # pragma: no cover - abstract
        raise NotImplementedError

    def _solve(self, state: SimState, target: np.ndarray) -> Any:
        return self._planner.solve(state.position, state.velocity, target)

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        if self._planner is None:
            ctrl_cfg = dict(cfg.get("controller", {}) or {})
            self._planner = self._build(dict(ctrl_cfg.get(self.name, {}) or {}))

        target = np.asarray(target_waypoint.position, dtype=float).reshape(3)
        t = float(state.t)
        stale = (
            self._held is None
            or t - self._held_t >= self._planner.dt - 1e-9
            or t < self._held_t  # time went backwards: a reset
            or not np.allclose(target, self._held_target)
        )
        if stale:
            self.last_solution = self._solve(state, target)
            self._held = np.asarray(self.last_solution.accel, dtype=float).copy()
            self._held_target = target.copy()
            self._held_t = t
        return ControlTarget(accel_cmd=self._held.copy(), metadata={"controller": self.name, "replanned": stale})


class ConstrainedMPCController(_HeldPlanController):
    """Box-constrained QP MPC, held between controller steps."""

    name = "constrained_mpc"

    def _build(self, cfg: Mapping[str, Any]) -> ConstrainedMPC:
        return ConstrainedMPC(**{k: v for k, v in cfg.items() if k in _MPC_KEYS})


class WarmMPPIController(_HeldPlanController):
    """Warm-started MPPI, held between controller steps.

    ``controller.warm_mppi.obstacles`` takes a list of ``{center: [x, y, z],
    radius: r}`` spheres for the sampler to avoid.
    """

    name = "warm_mppi"

    def _build(self, cfg: Mapping[str, Any]) -> MPPI:
        kwargs = {k: v for k, v in cfg.items() if k in _MPPI_KEYS}
        obstacles = [
            SphereObstacle(tuple(float(c) for c in o["center"]), float(o["radius"]))
            for o in cfg.get("obstacles", []) or []
        ]
        return MPPI(obstacles=obstacles, **kwargs)


__all__ = ["ConstrainedMPCController", "WarmMPPIController"]
