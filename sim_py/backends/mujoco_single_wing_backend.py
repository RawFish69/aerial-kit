"""MuJoCo-integrated single-motor flying wing.

Same split as :mod:`sim_py.backends.mujoco_fixedwing_backend`: MuJoCo
integrates the free body, two servo-driven elevon hinges (rate-limited,
damped, with stops) and the pusher prop shaft; the aerodynamic, thrust-offset,
motor-reaction and prop-wash loads come from
:class:`aerial_kit.dynamics.single_wing.SingleWingDynamics`, evaluated on the
*measured* hinge angles and the lagged thrust. The model is generated from
``single_wing.yaml`` (:mod:`aerial_kit.airframes.single_wing.mjcf`).

Actuator command: ``metadata["actuator_cmd"] = [thrust_n, elevon_left_rad,
elevon_right_rad]`` (physical order, TE-up positive), as for
``SingleWingBackend``.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.airframes.single_wing.mjcf import single_wing_mjcf
from aerial_kit.dynamics.fixed_wing import level_attitude_quat, quat_to_rotmat
from aerial_kit.dynamics.single_wing import SingleWingDynamics
from aerial_kit.interfaces import DynamicsBackend
from aerial_kit.types import CommandKind, ControlTarget, SimState

from .single_wing_backend import _params_from_config


class MujocoSingleWingBackend(DynamicsBackend):
    command_kind = CommandKind.AIRSPEED_NAV

    def __init__(self) -> None:
        self._model = self._data = self._mujoco = None
        self._spec = None
        self._aero: SingleWingDynamics | None = None
        self._body_id = -1
        self._adr: dict[str, int] = {}
        self._dof: dict[str, int] = {}
        self._act: dict[str, int] = {}
        self._servo_target = np.zeros(2)
        self._servo_rate = 8.0
        self._timestep = 0.002
        self._t = 0.0
        self._cruise = 14.0

    def reset(self, initial_state: SimState, world: Mapping[str, Any], cfg: Mapping[str, Any]) -> None:
        del world
        try:
            import mujoco
        except Exception as exc:  # pragma: no cover - optional dependency
            raise RuntimeError("MuJoCo is required for 'mujoco_singlewing' "
                               "(pip install -r sim_py/requirements-mujoco.txt)") from exc
        sim = dict(cfg.get("simulation", {}) or {})
        wing_cfg = dict(sim.get("singlewing", {}) or {})
        mj_cfg = dict(sim.get("mujoco", {}) or {})
        self._spec = load_single_wing_spec(wing_cfg.get("airframe_spec") or None)
        self._cruise = float(self._spec.flight_envelope["cruise_airspeed_mps"])
        self._timestep = float(mj_cfg.get("timestep", 0.002))
        if not math.isfinite(self._timestep) or self._timestep <= 0.0:
            raise ValueError("simulation.mujoco.timestep must be finite and positive")
        params = _params_from_config(cfg)
        self._aero = SingleWingDynamics(
            params, wind_mps=np.asarray(wing_cfg.get("wind_mps", [0.0, 0.0, 0.0]), dtype=float))
        self._servo_rate = float(self._spec.data["articulation"]["elevon_servo"]["max_rate_rad_s"])
        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_string(
            single_wing_mjcf(self._spec, self._timestep, params.gravity_mps2))
        self._data = mujoco.MjData(self._model)
        self._body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "body")
        for joint in ("left_elevon_joint", "right_elevon_joint", "prop_joint"):
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if jid < 0:
                raise ValueError(f"single-wing MJCF is missing joint {joint!r}")
            self._adr[joint] = int(self._model.jnt_qposadr[jid])
            self._dof[joint] = int(self._model.jnt_dofadr[jid])
        for act in ("left_elevon_servo", "right_elevon_servo", "prop_motor"):
            self._act[act] = int(mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, act))
        d = self._data
        d.qpos[:3] = np.asarray(initial_state.position, dtype=float).reshape(3)
        quat = (np.asarray(initial_state.attitude_quat, dtype=float).reshape(4)
                if initial_state.attitude_quat is not None else level_attitude_quat())
        d.qpos[3:7] = quat / np.linalg.norm(quat)
        d.qvel[:3] = np.asarray(initial_state.velocity, dtype=float).reshape(3)
        if initial_state.body_rates is not None:
            d.qvel[3:6] = np.asarray(initial_state.body_rates, dtype=float).reshape(3)
        self._aero.thrust_n = 0.0
        self._servo_target = np.zeros(2)
        self._t = float(initial_state.t)
        mujoco.mj_forward(self._model, d)

    # ---------------------------------------------------------------- state
    def _sync(self) -> None:
        d = self._data
        quat = np.asarray(d.qpos[3:7], dtype=float).copy()
        self._aero.position = np.asarray(d.qpos[:3], dtype=float).copy()
        self._aero.velocity = np.asarray(d.qvel[:3], dtype=float).copy()
        self._aero.attitude_quat = quat / np.linalg.norm(quat)
        self._aero.body_rates = np.asarray(d.qvel[3:6], dtype=float).copy()

    def measured_actuator_command(self) -> np.ndarray:
        d = self._data
        return np.array([self._aero.thrust_n,
                         d.qpos[self._adr["left_elevon_joint"]],
                         d.qpos[self._adr["right_elevon_joint"]]], dtype=float)

    def actuator_state(self) -> dict[str, float]:
        d = self._data
        return {
            "thrust_n": float(self._aero.thrust_n),
            "left_elevon_rad": float(d.qpos[self._adr["left_elevon_joint"]]),
            "right_elevon_rad": float(d.qpos[self._adr["right_elevon_joint"]]),
            "prop_rad_s": float(d.qvel[self._dof["prop_joint"]]),
        }

    def state(self) -> SimState:
        if self._data is None:
            raise RuntimeError("MujocoSingleWingBackend.reset() must be called before state().")
        self._sync()
        airspeed, _, _ = self._aero.airspeed_alpha_beta()
        return SimState(position=self._aero.position.copy(), velocity=self._aero.velocity.copy(),
                        t=self._t, attitude_quat=self._aero.attitude_quat.copy(),
                        body_rates=self._aero.body_rates.copy(), airspeed_mps=airspeed)

    # ----------------------------------------------------------------- step
    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._data is None:
            raise RuntimeError("MujocoSingleWingBackend.reset() must be called before step().")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and positive, got {dt!r}")
        cmd = control_target.metadata.get("actuator_cmd")
        if cmd is None:
            raise ValueError("MujocoSingleWingBackend requires actuator_cmd "
                             "[thrust_n, elevon_left_rad, elevon_right_rad]")
        cmd = np.asarray(cmd, dtype=float).reshape(3)
        if not np.all(np.isfinite(cmd)):
            raise ValueError("actuator_cmd contains a non-finite value")
        self._sync()
        self._aero.update_thrust(float(cmd[0]), dt)
        force_body, moment_body = self._aero.aero_forces_moments(
            self.measured_actuator_command(), self._aero.thrust_n)
        d = self._data
        rot = quat_to_rotmat(d.qpos[3:7])
        force_world, moment_world = rot @ force_body, rot @ moment_body
        lever = d.xpos[self._body_id] - d.xipos[self._body_id]
        d.xfrc_applied[self._body_id, :3] = force_world
        d.xfrc_applied[self._body_id, 3:6] = moment_world + np.cross(lever, force_world)

        travel = self._spec.max_elevon_rad
        step_max = self._servo_rate * dt
        for k, side in enumerate(("left", "right")):
            wanted = float(np.clip(cmd[1 + k], -travel, travel))
            self._servo_target[k] += float(np.clip(wanted - self._servo_target[k], -step_max, step_max))
            d.ctrl[self._act[f"{side}_elevon_servo"]] = self._servo_target[k]
        rotor = self._spec.data["articulation"]["rotor"]
        frac = float(np.clip(cmd[0] / self._spec.max_thrust_n, 0.0, 1.0))
        d.ctrl[self._act["prop_motor"]] = (
            self._spec.prop_spin * float(rotor["visual_max_speed_rad_s"]) * math.sqrt(frac))

        n = max(1, int(math.ceil(dt / self._timestep - 1e-12)))
        self._model.opt.timestep = dt / n
        for _ in range(n):
            self._mujoco.mj_step(self._model, d)
        self._t += dt

    def controller_defaults(self, section: str) -> dict:
        if section != "l1_tecs" or self._aero is None:
            return {}
        trim = self._aero.compute_trim_detailed(self._cruise)
        return {"trim_throttle_n": float(trim["thrust_n"]),
                "max_thrust_n": float(self._aero.params.max_thrust_n)}

    @property
    def spec(self):
        return self._spec

    @property
    def model(self):
        return self._model

    @property
    def data(self):
        return self._data

    def apply_constraints(self, min_bounds, max_bounds, terrain, terrain_clearance) -> None:
        if self._data is None:
            raise RuntimeError("MujocoSingleWingBackend.reset() must be called before constraints.")
        d = self._data
        d.qpos[:3] = np.clip(d.qpos[:3], np.asarray(min_bounds, float), np.asarray(max_bounds, float))
        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(float(d.qpos[0]), float(d.qpos[1])))
            d.qpos[2] = max(float(d.qpos[2]), ground + float(terrain_clearance))
        self._mujoco.mj_forward(self._model, d)
