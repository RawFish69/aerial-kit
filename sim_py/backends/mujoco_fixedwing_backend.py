"""MuJoCo-owned twin-wing rigid-body simulation.

The existing fixed-wing aero model supplies body-FRD force and moment loads.
MuJoCo integrates the free root body, the two elevon hinges and the two prop
shafts, and the *measured* hinge angles are fed back into the next aero
evaluation. This is a physics-backed plant rather than a pose animation.

The model itself is generated from the shared airframe spec
(``aerial_kit/airframes/twin_wing/twin_wing.yaml``) by
:func:`aerial_kit.airframes.twin_wing.mjcf.twin_wing_mjcf`; mass, inertia,
hinge lines, servo gains, prop positions and spin directions all come from
there.

What is physically integrated here:
  * root free body (aero + thrust wrench from FixedWingDynamics, gravity);
  * elevon hinges: position servos with rate limit, damping, armature,
    mechanical stops, surface mass/inertia;
  * prop shafts: velocity-controlled motors with shaft inertia whose torque
    reacts on the airframe (spin-up torque, gyroscopic coupling).
What is not: hinge aerodynamic moments, prop aerodynamic torque, and any
force from the visual geometry -- lift and drag still come from the
coefficient model, not from the meshes.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.twin_wing import TwinWingSpec, load_twin_wing_spec
from aerial_kit.airframes.twin_wing.mjcf import twin_wing_mjcf as _spec_mjcf
from aerial_kit.dynamics.fixed_wing import (
    FixedWingDynamics,
    FixedWingParams,
    level_attitude_quat,
    quat_to_rotmat,
)
from aerial_kit.interfaces import DynamicsBackend
from aerial_kit.types import CommandKind, ControlTarget, SimState

REQUIRED_JOINTS = (
    "left_elevon_joint",
    "right_elevon_joint",
    "left_prop_joint",
    "right_prop_joint",
)
REQUIRED_ACTUATORS = (
    "left_elevon_servo",
    "right_elevon_servo",
    "left_prop_motor",
    "right_prop_motor",
)


def _param_kwargs(cfg: Mapping[str, Any]) -> dict[str, Any]:
    fields = set(FixedWingParams.__dataclass_fields__)
    values = {
        key: value
        for key, value in dict(cfg).items()
        if key in fields and key != "inertia_kg_m2"
    }
    if "inertia_kg_m2" in cfg:
        values["inertia_kg_m2"] = np.asarray(cfg["inertia_kg_m2"], dtype=float)
    return values


def twin_wing_mjcf(
    params: FixedWingParams | None = None,
    spec: TwinWingSpec | None = None,
    timestep: float = 0.002,
) -> str:
    """MJCF for the twin wing, generated from the shared airframe spec.

    ``params`` is accepted for backwards compatibility and only contributes
    gravity; geometry and mass come from ``spec`` (the bundled YAML by
    default) so they cannot diverge from the Gazebo model.
    """
    spec = spec or load_twin_wing_spec()
    gravity = float(params.gravity_mps2) if params is not None else 9.81
    return _spec_mjcf(spec, timestep=timestep, gravity_mps2=gravity)


class MujocoFixedWingBackend(DynamicsBackend):
    """Actuator-level twin wing with MuJoCo rigid-body integration."""

    command_kind = CommandKind.AIRSPEED_NAV

    def __init__(self) -> None:
        self._model = None
        self._data = None
        self._mujoco = None
        self._body_id = -1
        self._qpos_adr: dict[str, int] = {}
        self._qvel_adr: dict[str, int] = {}
        self._act_id: dict[str, int] = {}
        self._spec: TwinWingSpec = load_twin_wing_spec()
        self._params = self._spec.fixed_wing_params()
        self._aero = FixedWingDynamics(self._params)
        self._thrust = np.zeros(2, dtype=float)
        self._servo_target = np.zeros(2, dtype=float)  # left, right (physical)
        self._t = 0.0
        self._nominal_timestep = 0.002
        self._thrust_tau_s = 0.03
        self._max_thrust_n = 10.0
        self._servo_rate = 8.0
        self._travel = self._spec.max_elevon_rad

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
                "MuJoCo is required for the 'mujoco_fixedwing' backend. "
                "Install it with `pip install -r sim_py/requirements-mujoco.txt`."
            ) from exc

        sim_cfg = dict(cfg.get("simulation", {}) or {})
        fw_cfg = dict(sim_cfg.get("fixedwing", {}) or {})
        mj_cfg = dict(sim_cfg.get("mujoco", {}) or {})
        self._spec = load_twin_wing_spec(mj_cfg.get("airframe_spec") or None)
        # The spec owns mass, geometry and motor layout; an explicit
        # simulation.fixedwing entry still overrides it (aero coefficients
        # live only in FixedWingParams).
        self._params = self._spec.fixed_wing_params(**_param_kwargs(fw_cfg))
        prop_cfg = self._spec.propulsion
        self._nominal_timestep = float(mj_cfg.get("timestep", 0.002))
        self._thrust_tau_s = float(mj_cfg.get("thrust_tau_s", prop_cfg["thrust_tau_s"]))
        self._max_thrust_n = float(fw_cfg.get("max_thrust_n", prop_cfg["max_thrust_n"]))
        self._servo_rate = float(self._spec.elevons["servo"]["max_rate_rad_s"])
        self._travel = self._spec.max_elevon_rad
        if not math.isfinite(self._nominal_timestep) or self._nominal_timestep <= 0.0:
            raise ValueError("simulation.mujoco.timestep must be finite and positive")
        if not math.isfinite(self._thrust_tau_s) or self._thrust_tau_s < 0.0:
            raise ValueError("simulation.mujoco.thrust_tau_s must be finite and non-negative")
        if not math.isfinite(self._max_thrust_n) or self._max_thrust_n <= 0.0:
            raise ValueError("simulation.fixedwing.max_thrust_n must be positive")

        model_path = mj_cfg.get("model_path")
        if model_path:
            self._model = mujoco.MjModel.from_xml_path(
                str(Path(str(model_path)).expanduser()))
        else:
            self._model = mujoco.MjModel.from_xml_string(twin_wing_mjcf(
                self._params, self._spec, self._nominal_timestep))
        self._model.opt.timestep = self._nominal_timestep
        self._data = mujoco.MjData(self._model)
        self._mujoco = mujoco
        self._body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "body")
        if self._body_id < 0:
            raise ValueError("twin-wing MJCF must contain a body named 'body'")
        self._qpos_adr.clear()
        self._qvel_adr.clear()
        self._act_id.clear()
        for joint in REQUIRED_JOINTS:
            joint_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if joint_id < 0:
                raise ValueError(f"twin-wing MJCF is missing joint {joint!r}")
            self._qpos_adr[joint] = int(self._model.jnt_qposadr[joint_id])
            self._qvel_adr[joint] = int(self._model.jnt_dofadr[joint_id])
        for act in REQUIRED_ACTUATORS:
            act_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, act)
            if act_id < 0:
                raise ValueError(f"twin-wing MJCF is missing actuator {act!r}")
            self._act_id[act] = int(act_id)

        position = np.asarray(initial_state.position, dtype=float).reshape(3)
        velocity = np.asarray(initial_state.velocity, dtype=float).reshape(3)
        quat = (
            np.asarray(initial_state.attitude_quat, dtype=float).reshape(4)
            if initial_state.attitude_quat is not None
            else level_attitude_quat()
        )
        quat = quat / np.linalg.norm(quat)
        rates = (
            np.asarray(initial_state.body_rates, dtype=float).reshape(3)
            if initial_state.body_rates is not None
            else np.zeros(3, dtype=float)
        )
        self._data.qpos[:3] = position
        self._data.qpos[3:7] = quat
        self._data.qvel[:3] = velocity
        self._data.qvel[3:6] = rates
        self._aero = FixedWingDynamics(
            self._params,
            wind_mps=np.asarray(
                fw_cfg.get("wind_mps", [0.0, 0.0, 0.0]), dtype=float),
        )
        self._thrust = np.zeros(2, dtype=float)
        self._servo_target = np.zeros(2, dtype=float)
        self._t = float(initial_state.t)
        mujoco.mj_forward(self._model, self._data)

    def _sync_aero_state(self) -> SimState:
        quat = np.asarray(self._data.qpos[3:7], dtype=float).copy()
        quat = quat / np.linalg.norm(quat)
        rates = np.asarray(self._data.qvel[3:6], dtype=float).copy()
        self._aero.position = np.asarray(self._data.qpos[:3], dtype=float).copy()
        self._aero.velocity = np.asarray(self._data.qvel[:3], dtype=float).copy()
        self._aero.attitude_quat = quat
        self._aero.body_rates = rates
        airspeed, _, _ = self._aero.airspeed_alpha_beta()
        return SimState(
            position=self._aero.position.copy(),
            velocity=self._aero.velocity.copy(),
            t=self._t,
            attitude_quat=quat,
            body_rates=rates,
            airspeed_mps=airspeed,
        )

    def _hinge(self, side: str) -> float:
        return float(self._data.qpos[self._qpos_adr[f"{side}_elevon_joint"]])

    def measured_actuator_command(self) -> np.ndarray:
        """[thrust_L, thrust_R, elevon_L, elevon_R] as the plant realises it.

        Elevon entries are the measured hinge angles placed at the command
        index the spec assigns to each physical surface.
        """
        out = np.zeros(4, dtype=float)
        out[:2] = self._thrust
        for side in ("left", "right"):
            out[self._spec.elevon_command_index(side)] = self._hinge(side)
        return out

    def actuator_state(self) -> dict[str, float]:
        """Hinge angles (rad, TE-up positive), shaft speeds (rad/s), thrust (N)."""
        if self._data is None:
            raise RuntimeError("MujocoFixedWingBackend.reset() must be called first.")
        return {
            "left_elevon_rad": self._hinge("left"),
            "right_elevon_rad": self._hinge("right"),
            "left_prop_rad_s": float(self._data.qvel[self._qvel_adr["left_prop_joint"]]),
            "right_prop_rad_s": float(self._data.qvel[self._qvel_adr["right_prop_joint"]]),
            "left_thrust_n": float(self._thrust[0]),
            "right_thrust_n": float(self._thrust[1]),
        }

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._model is None or self._data is None or self._mujoco is None:
            raise RuntimeError(
                "MujocoFixedWingBackend.reset() must be called before step().")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and positive, got {dt!r}")
        command = control_target.metadata.get("actuator_cmd")
        if command is None:
            raise ValueError(
                "MujocoFixedWingBackend requires actuator_cmd "
                "[thrust_L, thrust_R, elevon_L, elevon_R]")
        command = np.asarray(command, dtype=float).reshape(4)
        if not np.all(np.isfinite(command)):
            raise ValueError("actuator_cmd contains a non-finite value")

        self._sync_aero_state()
        alpha = 1.0 if self._thrust_tau_s == 0.0 else (
            1.0 - math.exp(-dt / self._thrust_tau_s))
        self._thrust += alpha * (command[:2] - self._thrust)

        # Aero sees the hinge angles MuJoCo actually reached, not the command.
        force_body, moment_body = self._aero.aero_forces_moments(
            self.measured_actuator_command())
        rot = quat_to_rotmat(self._data.qpos[3:7])
        force_world = rot @ force_body
        moment_world = rot @ moment_body
        # The aero wrench acts at the reference CG (the body origin).
        # xfrc_applied acts at the root body's own COM, which sits slightly
        # off the origin because the elevons and rotors are separate bodies;
        # shift the moment so the resultant is the same.
        lever = self._data.xpos[self._body_id] - self._data.xipos[self._body_id]
        self._data.xfrc_applied[self._body_id, :3] = force_world
        self._data.xfrc_applied[self._body_id, 3:6] = (
            moment_world + np.cross(lever, force_world))

        # Servo: rate-limited position target, then MuJoCo's position actuator.
        max_step = self._servo_rate * dt
        for k, side in enumerate(("left", "right")):
            wanted = float(np.clip(
                command[self._spec.elevon_command_index(side)],
                -self._travel, self._travel))
            delta = float(np.clip(wanted - self._servo_target[k], -max_step, max_step))
            self._servo_target[k] += delta
            self._data.ctrl[self._act_id[f"{side}_elevon_servo"]] = self._servo_target[k]
            # Props are driven from the commanded thrust; the motor's own
            # time constant matches the thrust lag.
            self._data.ctrl[self._act_id[f"{side}_prop_motor"]] = (
                self._spec.shaft_speed_for_thrust(
                    float(np.clip(command[k], 0.0, self._max_thrust_n))
                    * float(self._spec.propulsion["max_thrust_n"]) / self._max_thrust_n,
                    side))

        substeps = max(1, int(math.ceil(dt / self._nominal_timestep - 1e-12)))
        self._model.opt.timestep = dt / substeps
        for _ in range(substeps):
            self._mujoco.mj_step(self._model, self._data)
        self._t += dt

    def state(self) -> SimState:
        if self._data is None:
            raise RuntimeError(
                "MujocoFixedWingBackend.reset() must be called before state().")
        return self._sync_aero_state()

    @property
    def spec(self) -> TwinWingSpec:
        return self._spec

    @property
    def model(self):
        return self._model

    @property
    def data(self):
        return self._data

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._data is None or self._mujoco is None:
            raise RuntimeError(
                "MujocoFixedWingBackend.reset() must be called before constraints.")
        self._data.qpos[:3] = np.clip(
            self._data.qpos[:3],
            np.asarray(min_bounds, dtype=float),
            np.asarray(max_bounds, dtype=float),
        )
        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(
                float(self._data.qpos[0]), float(self._data.qpos[1])))
            self._data.qpos[2] = max(
                float(self._data.qpos[2]), ground + float(terrain_clearance))
        self._mujoco.mj_forward(self._model, self._data)
