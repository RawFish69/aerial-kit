"""MuJoCo multirotor plant: quad / hex / octo from the shared airframe specs.

Rotor thrust and yaw reaction torque are MuJoCo actuators (site transmission
with a first-order motor-lag filter, see
:mod:`aerial_kit.airframes.multirotor_specs.mjcf`), so MuJoCo integrates the
rotor forces, the free body, the visual rotor shafts and ground contact. Only
translational drag (and wind, through that drag) is applied as an external
force, matching :class:`aerial_kit.dynamics.ActuatorPlant`.

Commands, per step, either:

* ``ControlTarget.wrench`` (z-up body: ``force_body[2]`` collective thrust,
  ``moment_body`` roll/pitch/yaw) - allocated with the spec's
  geometry-derived mixer and moment-first desaturation; or
* ``metadata["rotor_thrust_n"]`` - per-rotor thrust in newtons, in the spec's
  rotor order (validated and clamped).

Propulsion models (``propulsion.model`` in the spec):

* ``lagged_thrust`` - thrust follows the command through MuJoCo's first-order
  actuator filter. The model the firmware ``ActuatorPlant`` cross-check pins.
* ``rpm`` - the command is a throttle (``u = sqrt(T_cmd / T_max)``); motor
  speed follows ``u * V / V_full`` with the motor time constant, thrust is
  ``T_max * omega^2`` (so spin-up and spin-down are asymmetric in thrust), and
  the pack voltage sags with current and state of charge (``battery`` in the
  spec). The backend integrates this every MuJoCo substep and sets the thrust.

``propulsion.ground_effect`` multiplies each rotor's thrust by the
Cheeseman-Bennett factor for its hub height above the ground (z = 0).

Settings under ``simulation.mujoco_multirotor``: ``airframe`` (bundled name
or YAML path, default ``fpv_quad_5in``), ``timestep`` (0.002), ``wind_mps``
(ENU), ``start_motors`` (``hover`` or ``idle``), and optional overrides
``propulsion_model`` (``rpm`` | ``lagged_thrust``), ``ground_effect`` (bool),
``battery_soc`` (0..1) and ``voltage_compensation`` (bool, default true: the
flight controller's battery-sag compensation, so sag appears as a lower
thrust ceiling rather than a steady droop).
"""

from __future__ import annotations

import copy
import math
from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.multirotor_specs import MultirotorSpec, load_multirotor_spec
from aerial_kit.airframes.multirotor_specs.spec import ground_effect_factor
from aerial_kit.airframes.multirotor_specs.mjcf import multirotor_mjcf
from aerial_kit.interfaces import DynamicsBackend
from .mujoco_guard import SimulationDiverged, step_guarded  # noqa: F401  (re-exported)
from aerial_kit.types import CommandKind, ControlTarget, SimState


def _import_mujoco():
    try:
        import mujoco
    except Exception as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "MuJoCo is required for the 'mujoco_multirotor' backend. "
            "Install it with `pip install -r sim_py/requirements-mujoco.txt`.") from exc
    return mujoco


class MujocoMultirotorBackend(DynamicsBackend):
    command_kind = CommandKind.ACCEL
    consumes_wrench = True

    def __init__(self) -> None:
        self._model = None
        self._data = None
        self._mujoco = None
        self._spec: MultirotorSpec | None = None
        self._body_id = -1
        self._thrust_act: list[int] = []
        self._spin_act: list[int] = []
        self._spin_dof: list[int] = []
        self._timestep = 0.002
        self._wind = np.zeros(3)
        self._drag = np.zeros(3)
        self._t = 0.0
        self.last_moment_scale = 1.0
        self.last_command = None
        self._rpm = False
        self._ground_effect = False
        self._omega = np.zeros(0)       # motor speed as a fraction of full-voltage max
        self._thrust = np.zeros(0)      # thrust actually produced (rpm model)
        self._site = []
        self._battery = None
        self._vcomp = True
        # Where this vehicle lives in the MuJoCo model: a name prefix and the
        # freejoint's qpos/qvel addresses. A single vehicle owns the model
        # ("" / 0 / 0); a swarm attaches several to one shared scene.
        self._prefix = ""
        self._qa = 0
        self._va = 0
        self._start = "hover"

    # ------------------------------------------------------------- setup
    @staticmethod
    def settings(cfg: Mapping[str, Any]) -> dict:
        sim_cfg = dict(cfg.get("simulation", {}) or {})
        return dict(sim_cfg.get("mujoco_multirotor", {}) or {})

    def reset(self, initial_state: SimState, world: Mapping[str, Any], cfg: Mapping[str, Any]) -> None:
        del world
        mujoco = _import_mujoco()
        s = self.settings(cfg)
        self.configure(s)
        model = mujoco.MjModel.from_xml_string(multirotor_mjcf(self._spec, self._timestep))
        self.bind(mujoco, model, mujoco.MjData(model))
        self.init_state(initial_state)

    def configure(self, s: Mapping[str, Any]) -> None:
        """Load the spec and per-vehicle settings (no model yet)."""
        self._spec = load_multirotor_spec(s.get("airframe", "fpv_quad_5in"))
        if "propulsion_model" in s or "ground_effect" in s:
            # Experiment override without editing the spec file.
            data = copy.deepcopy(self._spec.data)
            if "propulsion_model" in s:
                data["propulsion"]["model"] = str(s["propulsion_model"])
            if "ground_effect" in s:
                data["propulsion"]["ground_effect"] = bool(s["ground_effect"])
            self._spec = MultirotorSpec(data, source=f"{self._spec.source} (overridden)")
        self._timestep = float(s.get("timestep", 0.002))
        if not math.isfinite(self._timestep) or self._timestep <= 0.0:
            raise ValueError("simulation.mujoco_multirotor.timestep must be finite and positive")
        wind = np.asarray(s.get("wind_mps", [0.0, 0.0, 0.0]), dtype=float)
        if wind.shape != (3,) or not np.all(np.isfinite(wind)):
            raise ValueError("simulation.mujoco_multirotor.wind_mps must be 3 finite values (ENU)")
        self._wind = wind
        self._drag = self._spec.linear_drag()
        start = str(s.get("start_motors", "hover")).lower()
        if start not in ("hover", "idle"):
            raise ValueError("simulation.mujoco_multirotor.start_motors must be 'hover' or 'idle'")
        self._start = start
        self._rpm = self._spec.propulsion_model == "rpm"
        self._ground_effect = self._spec.ground_effect
        self._battery = self._spec.battery_model() if self._rpm else None
        # Flight-controller battery-sag compensation (Betaflight vbat
        # compensation / PX4 thrust scaling): commanded thrust is delivered
        # until the motor saturates. Off: the same throttle droops with sag.
        self._vcomp = bool(s.get("voltage_compensation", True))
        if self._battery is not None and "battery_soc" in s:
            self._battery.reset(float(s["battery_soc"]))

    def bind(self, mujoco, model, data, prefix: str = "") -> None:
        """Attach to a compiled model holding this vehicle under ``prefix``."""
        self._mujoco, self._model, self._data, self._prefix = mujoco, model, data, prefix
        self._body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "body")
        root = self._id(mujoco.mjtObj.mjOBJ_JOINT, "root")
        self._qa = int(model.jnt_qposadr[root])
        self._va = int(model.jnt_dofadr[root])
        self._site = [self._id(mujoco.mjtObj.mjOBJ_SITE, f"{r.name}_thrust_site") for r in self._spec.rotors]
        self._thrust_act, self._spin_act, self._spin_dof = [], [], []
        for rotor in self._spec.rotors:
            self._thrust_act.append(self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rotor.name}_thrust"))
            self._spin_act.append(self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rotor.name}_spin"))
            jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, f"{rotor.name}_rotor_joint")
            self._spin_dof.append(int(self._model.jnt_dofadr[jid]))

    def init_state(self, initial_state: SimState) -> None:
        """Place the vehicle and spin its motors up (hover) or leave them idle."""
        mujoco, d, qa, va = self._mujoco, self._data, self._qa, self._va
        d.qpos[qa:qa + 3] = np.asarray(initial_state.position, dtype=float).reshape(3)
        quat = (np.asarray(initial_state.attitude_quat, dtype=float).reshape(4)
                if initial_state.attitude_quat is not None else np.array([1.0, 0.0, 0.0, 0.0]))
        d.qpos[qa + 3:qa + 7] = quat / np.linalg.norm(quat)
        d.qvel[va:va + 3] = np.asarray(initial_state.velocity, dtype=float).reshape(3)
        if initial_state.body_rates is not None:
            d.qvel[va + 3:va + 6] = np.asarray(initial_state.body_rates, dtype=float).reshape(3)
        thrusts = self._spec.hover_thrusts() if self._start == "hover" else np.zeros(self._spec.n_rotors)
        mujoco.mj_forward(self._model, d)
        if self._rpm:
            v_ratio = self._voltage_ratio(thrusts)
            ge = self._ge_factors()
            # Spun up so that the produced thrust is exactly the start thrust.
            self._omega = np.sqrt(np.clip(thrusts / (self._spec.allocator.t_max * ge), 0.0, None))
            self._thrust = self._spec.allocator.t_max * self._omega ** 2 * ge
            if not self._vcomp:
                # Throttle that holds this speed at the present voltage.
                thrusts = self._spec.allocator.t_max * (self._omega / v_ratio) ** 2
        self._apply_thrusts(thrusts)
        # Start spun up (thrust state and shafts) so a hover start is a hover.
        for k, rotor in enumerate(self._spec.rotors):
            if self._rpm:
                d.ctrl[self._thrust_act[k]] = self._thrust[k]
            else:
                d.act[self._model.actuator_actadr[self._thrust_act[k]]] = thrusts[k]
            d.qvel[self._spin_dof[k]] = self._spec.shaft_speed_for_thrust(
                self._thrust[k] if self._rpm else thrusts[k], rotor)
        self._t = float(initial_state.t)
        mujoco.mj_forward(self._model, d)

    def _id(self, kind, name: str) -> int:
        idx = self._mujoco.mj_name2id(self._model, kind, self._prefix + name)
        if idx < 0:
            raise ValueError(f"multirotor MJCF is missing {self._prefix + name!r}")
        return int(idx)

    # ------------------------------------------------------------ commands
    def _apply_thrusts(self, thrusts: np.ndarray) -> None:
        self.last_command = np.asarray(thrusts, dtype=float).copy()
        if self._rpm:
            return       # the substep loop owns the actuator controls
        ge = self._ge_factors() if self._ground_effect else 1.0
        for k, rotor in enumerate(self._spec.rotors):
            self._data.ctrl[self._thrust_act[k]] = thrusts[k] * (ge[k] if self._ground_effect else 1.0)
            self._data.ctrl[self._spin_act[k]] = self._spec.shaft_speed_for_thrust(thrusts[k], rotor)

    def _ge_factors(self) -> np.ndarray:
        if not self._ground_effect:
            return np.ones(self._spec.n_rotors)
        radius = self._spec.prop_radius_m
        return np.array([ground_effect_factor(float(self._data.site_xpos[s][2]), radius)
                         for s in self._site])

    def _voltage_ratio(self, thrusts) -> float:
        if self._battery is None:
            return 1.0
        self._battery.step(thrusts, 0.0)
        return self._battery.voltage / self._battery.full_voltage

    def _rpm_substep(self, h: float) -> None:
        """Motor speed, battery and ground effect for one MuJoCo substep."""
        t_max = self._spec.allocator.t_max
        v_ratio = 1.0 if self._battery is None else self._battery.voltage / self._battery.full_voltage
        # Speed the command asks for at full voltage; with compensation the
        # throttle is raised by 1/v_ratio, capped at full throttle.
        wanted = np.sqrt(np.clip(self.last_command / t_max, 0.0, None))
        throttle = np.clip(wanted / v_ratio if self._vcomp else wanted, 0.0, 1.0)
        tau = np.array([r.motor_tau_s for r in self._spec.rotors])
        self._omega += (throttle * v_ratio - self._omega) * (1.0 - np.exp(-h / tau))
        self._thrust = t_max * self._omega ** 2 * self._ge_factors()
        if self._battery is not None:
            self._battery.step(self._thrust, h)
        d = self._data
        for k, rotor in enumerate(self._spec.rotors):
            d.ctrl[self._thrust_act[k]] = self._thrust[k]
            # Shafts turn at the motor's actual speed, not the command.
            d.ctrl[self._spin_act[k]] = rotor.spin * float(
                self._spec.rotor_cfg["visual_max_speed_rad_s"]) * self._omega[k]

    def thrusts_for(self, control_target: ControlTarget) -> np.ndarray:
        direct = control_target.metadata.get("rotor_thrust_n") if control_target.metadata else None
        if direct is not None:
            thrusts, _ = self._spec.clamp_rotor_thrusts(direct)
            self.last_moment_scale = 1.0
            return thrusts
        wrench = control_target.wrench
        if wrench is None:
            raise ValueError(
                "mujoco_multirotor flies rotors: give a ControlTarget.wrench (from a "
                "geometric/nmpc/cascade controller) or metadata['rotor_thrust_n'].")
        force = np.asarray(wrench.force_body, dtype=float)
        moment = np.asarray(wrench.moment_body, dtype=float)
        if not (np.all(np.isfinite(force)) and np.all(np.isfinite(moment))):
            raise ValueError("wrench contains a non-finite value")
        thrusts, scale = self._spec.allocator.allocate(float(force[2]), moment)
        self.last_moment_scale = scale
        return thrusts

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._data is None:
            raise RuntimeError("MujocoMultirotorBackend.reset() must be called before step().")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and positive, got {dt!r}")
        self._apply_thrusts(self.thrusts_for(control_target))
        n = max(1, int(math.ceil(dt / self._timestep - 1e-12)))
        self._model.opt.timestep = dt / n
        for _ in range(n):
            self.pre_substep(dt / n)
            step_guarded(self._mujoco, self._model, self._data)
        self._t += dt

    def pre_substep(self, h: float) -> None:
        """Everything this vehicle does before one MuJoCo substep of length ``h``."""
        self._apply_drag()
        if self._rpm:
            self._rpm_substep(h)
        elif self._ground_effect:
            self._apply_thrusts(self.last_command)

    def _apply_drag(self) -> None:
        d = self._data
        rot = d.xmat[self._body_id].reshape(3, 3)
        v_air_body = rot.T @ (d.qvel[self._va:self._va + 3] - self._wind)
        force_world = rot @ (-self._drag * v_air_body)
        d.xfrc_applied[self._body_id, :3] = force_world
        # xfrc acts at the root body's COM; the composite CG is the origin.
        lever = d.xpos[self._body_id] - d.xipos[self._body_id]
        d.xfrc_applied[self._body_id, 3:6] = np.cross(lever, force_world)

    # --------------------------------------------------------------- state
    def state(self) -> SimState:
        if self._data is None:
            raise RuntimeError("MujocoMultirotorBackend.reset() must be called before state().")
        d = self._data
        qa, va = self._qa, self._va
        quat = np.asarray(d.qpos[qa + 3:qa + 7], dtype=float).copy()
        quat /= np.linalg.norm(quat)
        velocity = np.asarray(d.qvel[va:va + 3], dtype=float).copy()
        return SimState(
            position=np.asarray(d.qpos[qa:qa + 3], dtype=float).copy(),
            velocity=velocity,
            t=self._t,
            attitude_quat=quat,
            body_rates=np.asarray(d.qvel[va + 3:va + 6], dtype=float).copy(),
            airspeed_mps=float(np.linalg.norm(velocity - self._wind)),
        )

    def rotor_state(self) -> dict[str, np.ndarray]:
        """Commanded and lagged (actual) thrust, and shaft speeds, per rotor."""
        d, m = self._data, self._model
        if self._rpm:
            actual = self._thrust.copy()
        else:
            actual = np.array([d.act[m.actuator_actadr[a]] for a in self._thrust_act])
        out = {
            "commanded_n": np.asarray(self.last_command, dtype=float).copy(),
            "actual_n": actual,
            "shaft_rad_s": np.array([d.qvel[k] for k in self._spin_dof]),
        }
        if self._rpm:
            out["motor_speed_frac"] = self._omega.copy()
        if self._battery is not None:
            out.update(battery_v=self._battery.voltage, battery_a=self._battery.current,
                       battery_soc=self._battery.soc)
        return out

    def on_ground(self, speed_tol: float = 0.3) -> bool:
        """True when the airframe's collision geometry touches the world and it is nearly still."""
        d, m = self._data, self._model
        touching = False
        for i in range(d.ncon):
            c = d.contact[i]
            bodies = (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])
            if 0 in bodies and self._body_id in bodies:
                touching = True
                break
        return touching and float(np.linalg.norm(d.qvel[self._va:self._va + 3])) < speed_tol

    def mass_kg(self) -> float:
        return self._spec.total_mass_kg if self._spec is not None else 1.0

    def controller_defaults(self, section: str) -> dict:
        """Physical defaults for wrench controllers, from the airframe spec."""
        if self._spec is None:
            return {}
        out = {"mass_kg": self._spec.total_mass_kg,
               "inertia_kgm2": [float(v) for v in self._spec.inertia_diag]}
        if section == "cascade":
            c = self._spec.control
            out.update(k_att=float(c["k_att"]), k_rate=float(c["k_rate"]),
                       max_tilt_deg=float(c["max_tilt_deg"]))
        return out

    @property
    def spec(self) -> MultirotorSpec | None:
        return self._spec

    @property
    def model(self):
        return self._model

    @property
    def data(self):
        return self._data

    def apply_constraints(self, min_bounds, max_bounds, terrain, terrain_clearance) -> None:
        if self._data is None:
            raise RuntimeError("MujocoMultirotorBackend.reset() must be called before constraints.")
        d, qa = self._data, self._qa
        d.qpos[qa:qa + 2] = np.clip(d.qpos[qa:qa + 2], np.asarray(min_bounds, float)[:2],
                                    np.asarray(max_bounds, float)[:2])
        d.qpos[qa + 2] = min(float(d.qpos[qa + 2]), float(np.asarray(max_bounds, float)[2]))
        self._mujoco.mj_forward(self._model, d)
