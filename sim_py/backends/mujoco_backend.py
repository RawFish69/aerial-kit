"""MuJoCo point-mass dynamics backend.

This is the first MuJoCo integration: a free 6-DOF body driven by the same
``ControlTarget.accel_cmd`` contract as ``PointMassBackend``. It is intended to
grow into a full multirotor/fixed-wing MuJoCo backend, but the initial version
keeps the physics simple so ``--backend mujoco`` is usable immediately.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from aerial_kit.interfaces import DynamicsBackend
from aerial_kit.types import CommandKind, ControlTarget, SimState


_MJOCO_XML_TEMPLATE = """\
<mujoco model="aerial_kit_mujoco">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="vehicle" pos="{x} {y} {z}">
      <freejoint name="free"/>
      <geom type="sphere" size="0.12" mass="{mass}"/>
    </body>
  </worldbody>
</mujoco>
"""


class MujocoBackend(DynamicsBackend):
    """Minimal MuJoCo backend using a free body and applied world-frame force."""

    command_kind = CommandKind.ACCEL

    def __init__(self) -> None:
        self._model = None
        self._data = None
        self._body_id = -1
        self._mass = 1.0
        self._t = 0.0
        self._nominal_timestep = 0.002

    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        del world
        try:
            import mujoco
        except Exception as exc:  # pragma: no cover - optional dependency
            raise RuntimeError(
                "MuJoCo is required for the 'mujoco' backend. Install it with "
                "`pip install -r sim_py/requirements-mujoco.txt`."
            ) from exc

        sim_cfg = dict(cfg.get("simulation", {}) or {})
        mj_cfg = dict(sim_cfg.get("mujoco", {}) or {})
        self._mass = float(mj_cfg.get("mass", 1.0))
        self._nominal_timestep = float(mj_cfg.get("timestep", 0.002))
        if not np.isfinite(self._nominal_timestep) or self._nominal_timestep <= 0.0:
            raise ValueError(
                "simulation.mujoco.timestep must be finite and positive, got "
                f"{self._nominal_timestep!r}"
            )

        pos = np.asarray(initial_state.position, dtype=float).reshape(3)
        xml = _MJOCO_XML_TEMPLATE.format(x=pos[0], y=pos[1], z=pos[2], mass=self._mass)
        self._model = mujoco.MjModel.from_xml_string(xml)
        self._data = mujoco.MjData(self._model)
        self._body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "vehicle")

        self._data.qpos[0:3] = pos
        self._data.qvel[0:3] = np.asarray(initial_state.velocity, dtype=float).reshape(3)
        self._t = float(initial_state.t)

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._model is None or self._data is None:
            raise RuntimeError("MujocoBackend.reset() must be called before step().")

        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and positive, got {dt!r}")

        accel = np.asarray(control_target.accel_cmd, dtype=float).reshape(3)
        gravity_up = float(-self._model.opt.gravity[2])
        force = self._mass * (accel + np.array([0.0, 0.0, gravity_up], dtype=float))
        self._data.xfrc_applied[self._body_id, 0:3] = force
        self._data.xfrc_applied[self._body_id, 3:6] = 0.0
        self._data.ctrl[:] = 0.0

        import mujoco

        # ``mj_step`` integrates for ``model.opt.timestep``, not for the ``dt``
        # this method was handed. Stepping once and advancing ``self._t`` by
        # ``dt`` therefore runs the physics and the clock at different rates -
        # with MuJoCo's 2 ms default and a 10 ms caller, the simulation advanced
        # a fifth of the interval it reported. Sub-stepping makes the physics
        # advance exactly ``dt``: ``n`` whole steps of ``dt / n`` each, so
        # ``n * timestep == dt`` to the last bit and neither the clock nor the
        # integration is asked to approximate.
        substeps = max(1, int(np.ceil(dt / self._nominal_timestep - 1e-12)))
        self._model.opt.timestep = dt / substeps
        for _ in range(substeps):
            mujoco.mj_step(self._model, self._data)
        self._t += dt

    def state(self) -> SimState:
        if self._data is None:
            raise RuntimeError("MujocoBackend.reset() must be called before state().")
        return SimState(
            position=self._data.qpos[0:3].copy(),
            velocity=self._data.qvel[0:3].copy(),
            t=self._t,
        )

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._data is None:
            raise RuntimeError("MujocoBackend.reset() must be called before apply_constraints().")
        self._data.qpos[0:3] = np.clip(
            self._data.qpos[0:3],
            np.asarray(min_bounds, dtype=float),
            np.asarray(max_bounds, dtype=float),
        )
        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(float(self._data.qpos[0]), float(self._data.qpos[1])))
            self._data.qpos[2] = max(float(self._data.qpos[2]), ground + float(terrain_clearance))
