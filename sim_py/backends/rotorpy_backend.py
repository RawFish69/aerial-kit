"""RotorPy-backed dynamics backend.

This backend keeps the repo's canonical control API (acceleration targets)
and maps it to RotorPy's ``cmd_acc`` control abstraction.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..core.interfaces import DynamicsBackend
from ..core.types import ControlTarget, SimState


def _missing_rotorpy_error(exc: Exception) -> RuntimeError:
    err = RuntimeError(
        "RotorPy backend requested but RotorPy is not available. "
        "Install optional dependencies with: "
        'python -m pip install "aerial-kit[rotorpy]" '
        "(legacy source checkout: sim_py/requirements-rotorpy.txt)"
    )
    err.__cause__ = exc
    return err


class RotorPyBackend(DynamicsBackend):
    """Dynamics backend using RotorPy's Multirotor model."""

    def __init__(self) -> None:
        self._vehicle: Any | None = None
        self._state: dict[str, np.ndarray] | None = None
        self._t = 0.0

    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        try:
            from rotorpy.vehicles.crazyflie_params import quad_params as crazyflie_quad_params
            from rotorpy.vehicles.multirotor import Multirotor
        except Exception as exc:  # pragma: no cover - depends on local install
            raise _missing_rotorpy_error(exc)

        sim_cfg = dict(cfg.get("simulation", {}) or {})
        rp_cfg = dict(sim_cfg.get("rotorpy", {}) or {})

        vehicle_name = str(rp_cfg.get("vehicle", "crazyflie")).lower().strip()
        quad_params = dict(crazyflie_quad_params)

        # Optional per-parameter overrides from sim_config.yaml.
        for k, v in dict(rp_cfg.get("quad_params_override", {}) or {}).items():
            quad_params[str(k)] = v

        if vehicle_name not in {"crazyflie", "cf", "default"}:
            raise ValueError(
                f"Unsupported RotorPy vehicle '{vehicle_name}'. "
                "Supported in v1: crazyflie"
            )

        self._vehicle = Multirotor(
            quad_params=quad_params,
            control_abstraction="cmd_acc",
            aero=bool(rp_cfg.get("aero", True)),
            enable_ground=bool(rp_cfg.get("enable_ground", False)),
        )

        base_state: dict[str, np.ndarray] = {}
        for key, value in self._vehicle.initial_state.items():
            base_state[str(key)] = np.asarray(value, dtype=float).copy()

        base_state["x"] = np.asarray(initial_state.position, dtype=float).copy()
        base_state["v"] = np.asarray(initial_state.velocity, dtype=float).copy()

        if initial_state.attitude_quat is not None:
            q_wxyz = np.asarray(initial_state.attitude_quat, dtype=float).reshape(4)
            # RotorPy stores quaternions as [x, y, z, w].
            base_state["q"] = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)

        if initial_state.body_rates is not None:
            base_state["w"] = np.asarray(initial_state.body_rates, dtype=float).reshape(3)

        if "wind" not in base_state:
            base_state["wind"] = np.zeros(3, dtype=float)

        if "rotor_speeds" not in base_state:
            # RotorPy default hover speed for Crazyflie model.
            base_state["rotor_speeds"] = np.array([1788.53, 1788.53, 1788.53, 1788.53], dtype=float)

        self._state = base_state
        self._t = float(initial_state.t)

    def _map_accel_to_rotorpy_cmd(self, control_target: ControlTarget) -> dict[str, np.ndarray]:
        if self._vehicle is None:
            raise RuntimeError("RotorPyBackend.reset() must be called before step().")

        accel_cmd = np.asarray(control_target.accel_cmd, dtype=float).reshape(3)
        cmd_acc = accel_cmd + np.array([0.0, 0.0, float(self._vehicle.g)], dtype=float)

        if not np.all(np.isfinite(cmd_acc)):
            cmd_acc = np.array([0.0, 0.0, float(self._vehicle.g)], dtype=float)

        if np.linalg.norm(cmd_acc) < 1e-6:
            cmd_acc = np.array([0.0, 0.0, float(self._vehicle.g)], dtype=float)

        return {"cmd_acc": cmd_acc}

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._vehicle is None or self._state is None:
            raise RuntimeError("RotorPyBackend.reset() must be called before step().")

        control = self._map_accel_to_rotorpy_cmd(control_target)
        self._state = self._vehicle.step(self._state, control, float(dt))
        self._t += float(dt)

    def state(self) -> SimState:
        if self._state is None:
            raise RuntimeError("RotorPyBackend.reset() must be called before state().")

        q_xyzw = np.asarray(self._state.get("q", np.array([0.0, 0.0, 0.0, 1.0])), dtype=float)
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)

        return SimState(
            position=np.asarray(self._state.get("x", np.zeros(3)), dtype=float).copy(),
            velocity=np.asarray(self._state.get("v", np.zeros(3)), dtype=float).copy(),
            attitude_quat=q_wxyz,
            body_rates=np.asarray(self._state.get("w", np.zeros(3)), dtype=float).copy(),
            t=float(self._t),
        )

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._state is None:
            raise RuntimeError("RotorPyBackend.reset() must be called before apply_constraints().")

        self._state["x"] = np.clip(
            np.asarray(self._state["x"], dtype=float),
            np.asarray(min_bounds, dtype=float),
            np.asarray(max_bounds, dtype=float),
        )

        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(float(self._state["x"][0]), float(self._state["x"][1])))
            min_z = ground + float(terrain_clearance)
            if self._state["x"][2] < min_z:
                self._state["x"][2] = min_z
