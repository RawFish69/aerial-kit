"""Motor-level quad-X backend: the actuator plant behind the simulator runner.

Every other multirotor backend here integrates an *acceleration*: the
controller's ``accel_cmd`` is the aircraft's acceleration, with no motors, no
attitude dynamics and nothing to saturate (the runner labels that mode
``IDEAL_ACCEL`` and explains at length why it is kept). This backend is the
other mode, ``ACTUATOR``: it takes the controller's **wrench**, mixes it into
four motor commands through the airframe's allocator and
``aerial_kit.dynamics.quad_x_seam``, and integrates
``aerial_kit.dynamics.ActuatorPlant`` - first-order motor lag, per-motor
saturation, rigid-body attitude dynamics, ground contact - at its own fine
step, holding the motor commands across the runner's step.

So it flies only controllers that close an attitude loop and return a wrench:
``geometric``, ``nmpc`` and ``cascade``. An acceleration-only controller is
refused with that list, rather than flown into the ground.

Settings live under ``simulation.actuator`` (mass, geometry, motor limits and
lag, ``plant_dt``). The runner fills the controller's ``mass_kg`` from the same
section when the controller's own config does not give one, so the controller
and the plant cannot silently disagree about the mass.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.multirotor import MultirotorAirframe
from aerial_kit.dynamics import ActuatorPlant, ActuatorPlantParams
from aerial_kit.dynamics.actuator_loop import FRD_TO_ZUP_BODY, NED_TO_ENU, zup_state_of
from aerial_kit.dynamics.multirotor_actuator import QUAD_X_SPIN, quad_x_positions
from aerial_kit.dynamics.quad_x_seam import motor_commands
from aerial_kit.dynamics.rotations import quat_to_rotmat, rotmat_to_quat

from ..core.interfaces import DynamicsBackend
from ..core.types import CommandKind, ControlTarget, SimState

WRENCH_CONTROLLERS = ("geometric", "nmpc", "cascade")

_PLANT_KEYS = set(ActuatorPlantParams.__dataclass_fields__)


class ActuatorBackend(DynamicsBackend):
    """Quad-X actuator plant driven by ``ControlTarget.wrench``."""

    # The airframe contract is still the multirotor's; what differs is that
    # this backend reads the wrench instead of the acceleration.
    command_kind = CommandKind.ACCEL
    consumes_wrench = True

    def __init__(self) -> None:
        self.plant: ActuatorPlant | None = None
        self.airframe: MultirotorAirframe | None = None
        self.plant_dt = 0.002

    @staticmethod
    def settings(cfg: Mapping[str, Any]) -> dict:
        sim_cfg = dict(cfg.get("simulation", {}) or {})
        return dict(sim_cfg.get("actuator", {}) or {})

    def reset(self, initial_state: SimState, world: Mapping[str, Any], cfg: Mapping[str, Any]) -> None:
        s = self.settings(cfg)
        arm = float(s.get("arm_length_m", 0.2))
        self.plant_dt = float(s.get("plant_dt", 0.002))
        if self.plant_dt <= 0.0:
            raise ValueError("simulation.actuator.plant_dt must be > 0")
        params = ActuatorPlantParams(**{k: v for k, v in s.items() if k in _PLANT_KEYS})
        self.plant = ActuatorPlant(params=params, motor_positions=quad_x_positions(arm), spin=QUAD_X_SPIN.copy())
        self.airframe = MultirotorAirframe(
            arms=4, layout="x", arm_length_m=arm, mass_kg=params.mass_kg,
            yaw_torque_coeff=params.yaw_torque_coeff,
        )

        # The runner speaks ENU / z-up body; the plant is NED / FRD. Both maps
        # are their own inverse.
        p = np.asarray(initial_state.position, dtype=float)
        v = np.asarray(initial_state.velocity, dtype=float)
        self.plant.position[...] = NED_TO_ENU @ p
        self.plant.velocity[...] = NED_TO_ENU @ v
        if initial_state.attitude_quat is not None:
            r_enu = quat_to_rotmat(np.asarray(initial_state.attitude_quat, dtype=float))
            self.plant.attitude_quat[...] = rotmat_to_quat(NED_TO_ENU @ r_enu @ FRD_TO_ZUP_BODY)
        if initial_state.body_rates is not None:
            self.plant.body_rates[...] = FRD_TO_ZUP_BODY @ np.asarray(initial_state.body_rates, dtype=float)
        # Start in steady hover: commanded *and* spun up. set_hover() only
        # sets the command, and motors starting from rest behind their lag
        # drop the aircraft for the first tens of milliseconds - a sink rate
        # nothing asked for, at the very start of every mission.
        self.plant.set_hover()
        self.plant.motor_actual[...] = self.plant.motor_command

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self.plant is None:
            raise RuntimeError("ActuatorBackend.reset() must be called before step().")
        if control_target.wrench is None:
            raise ValueError(
                "the actuator backend flies motors, and needs a controller that returns a "
                f"wrench: one of {', '.join(WRENCH_CONTROLLERS)}. An acceleration-only "
                "controller has no attitude loop to turn its demand into a moment."
            )
        self.plant.set_motors(motor_commands(self.airframe, control_target.wrench, self.plant))
        n = max(1, math.ceil(float(dt) / self.plant_dt - 1e-9))
        h = float(dt) / n
        for _ in range(n):
            self.plant.step(h)

    def state(self) -> SimState:
        if self.plant is None:
            raise RuntimeError("ActuatorBackend.reset() must be called before state().")
        return zup_state_of(self.plant)

    def mass_kg(self) -> float:
        return float(self.plant.params.mass_kg) if self.plant is not None else 1.0
