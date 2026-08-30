"""Multirotor airframe: quad/hex/octo differ only in mixer geometry."""

from __future__ import annotations

import numpy as np

from ..types import Capabilities, CommandKind, SimState, Wrench
from .base import Airframe


class MultirotorAirframe(Airframe):
    """An omnidirectional multirotor: quad, hex, or octo, X or + layout.

    The allocation matrix maps [total_thrust, Mx, My, Mz] to per-rotor thrust
    via its pseudo-inverse. Rotors are placed on a regular polygon and spin
    direction alternates so that all-equal thrust yields zero net yaw torque
    at trim.
    """

    def __init__(
        self,
        arms: int = 4,
        layout: str = "x",
        arm_length_m: float = 0.2,
        mass_kg: float = 1.0,
        yaw_torque_coeff: float = 0.02,
        gravity_mps2: float = 9.81,
        max_airspeed_mps: float = 20.0,
        max_climb_rate_mps: float = 5.0,
        max_bank_deg: float = 45.0,
    ) -> None:
        if arms < 3:
            raise ValueError(f"arms must be >= 3, got {arms}")
        if layout not in {"x", "+"}:
            raise ValueError(f"layout must be 'x' or '+', got {layout!r}")

        self.name = f"multirotor_{arms}_{layout}"
        self.arms = arms
        self.layout = layout
        self.arm_length_m = float(arm_length_m)
        self.mass_kg = float(mass_kg)
        self.yaw_torque_coeff = float(yaw_torque_coeff)
        self.gravity_mps2 = float(gravity_mps2)

        self.capabilities = Capabilities(
            can_hover=True,
            min_airspeed_mps=None,
            max_airspeed_mps=float(max_airspeed_mps),
            max_climb_rate_mps=float(max_climb_rate_mps),
            max_bank_deg=float(max_bank_deg),
            min_turn_radius_m=None,
            n_actuators=arms,
            command_kind=CommandKind.ACCEL,
        )

        self._mixer = self._build_mixer()
        self._mixer_pinv = np.linalg.pinv(self._mixer)

    def _build_mixer(self) -> np.ndarray:
        offset = np.pi / self.arms if self.layout == "x" else 0.0
        mixer = np.zeros((4, self.arms), dtype=float)
        for i in range(self.arms):
            theta = 2.0 * np.pi * i / self.arms + offset
            x_i = self.arm_length_m * np.cos(theta)
            y_i = self.arm_length_m * np.sin(theta)
            spin_dir = 1.0 if i % 2 == 0 else -1.0
            mixer[0, i] = 1.0
            mixer[1, i] = y_i
            mixer[2, i] = -x_i
            mixer[3, i] = spin_dir * self.yaw_torque_coeff
        return mixer

    def allocate(self, wrench: Wrench, state: SimState) -> np.ndarray:
        thrust_total = float(np.asarray(wrench.force_body, dtype=float)[2])
        mx, my, mz = np.asarray(wrench.moment_body, dtype=float)
        demand = np.array([thrust_total, mx, my, mz], dtype=float)
        return self._mixer_pinv @ demand

    def trim(self, state: SimState) -> np.ndarray:
        demand = np.array([self.mass_kg * self.gravity_mps2, 0.0, 0.0, 0.0], dtype=float)
        return self._mixer_pinv @ demand
