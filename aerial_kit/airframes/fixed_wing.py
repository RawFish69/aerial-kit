"""Twin-motor flying wing: two motors, two elevons, no rudder.

See plans/04-twin-wing-control/PLAN.md for the design. Yaw is differential
thrust only -- authority is proportional to throttle and collapses at idle
or in a glide, so a controller must coordinate turns with bank rather than
relying on yaw. Aero coefficients for a specific airframe are unknown at
this stage (no FixedWingBackend/aero model exists yet), so allocate() below
is a deliberately simple, documented placeholder: moment_body is
interpreted directly in actuator-native units (elevon radians for
pitch/roll) rather than through a dynamic-pressure-scaled aero model. It
will be superseded once a real aero model can convert aerodynamic moments
into these commands.
"""

from __future__ import annotations

import numpy as np

from ..types import Capabilities, CommandKind, SimState, Wrench
from .base import Airframe


class TwinWingAirframe(Airframe):
    """Flying wing: throttle_L/R + elevon_L/R actuators, AIRSPEED_NAV command kind."""

    def __init__(
        self,
        mass_kg: float = 1.5,
        cruise_airspeed_mps: float = 15.0,
        min_airspeed_mps: float = 8.0,
        max_airspeed_mps: float = 25.0,
        max_climb_rate_mps: float = 3.0,
        max_bank_deg: float = 45.0,
        max_thrust_n: float = 10.0,
        max_elevon_rad: float = np.radians(25.0),
        motor_separation_m: float = 0.6,
        trim_throttle_n: float = 3.0,
        trim_elevon_rad: float = np.radians(2.0),
        gravity_mps2: float = 9.81,
    ) -> None:
        if mass_kg <= 0.0:
            raise ValueError(f"mass_kg must be positive, got {mass_kg}")
        if motor_separation_m <= 0.0:
            raise ValueError(f"motor_separation_m must be positive, got {motor_separation_m}")
        if max_thrust_n <= 0.0:
            raise ValueError(f"max_thrust_n must be positive, got {max_thrust_n}")
        if not (0.0 < max_bank_deg < 90.0):
            raise ValueError(f"max_bank_deg must be in (0, 90), got {max_bank_deg}")

        self.name = "twin_wing"
        self.mass_kg = float(mass_kg)
        self.motor_separation_m = float(motor_separation_m)
        self.max_thrust_n = float(max_thrust_n)
        self.max_elevon_rad = float(max_elevon_rad)
        self.trim_throttle_n = float(trim_throttle_n)
        self.trim_elevon_rad = float(trim_elevon_rad)
        self.gravity_mps2 = float(gravity_mps2)

        phi_max_rad = np.radians(max_bank_deg)
        min_turn_radius_m = (cruise_airspeed_mps**2) / (gravity_mps2 * np.tan(phi_max_rad))

        self.capabilities = Capabilities(
            can_hover=False,
            min_airspeed_mps=float(min_airspeed_mps),
            max_airspeed_mps=float(max_airspeed_mps),
            max_climb_rate_mps=float(max_climb_rate_mps),
            max_bank_deg=float(max_bank_deg),
            min_turn_radius_m=float(min_turn_radius_m),
            n_actuators=4,
            command_kind=CommandKind.AIRSPEED_NAV,
        )

    def allocate(self, wrench: Wrench, state: SimState) -> np.ndarray:
        """[throttle_L, throttle_R, elevon_L, elevon_R], saturating by priority.

        Interprets ``wrench.force_body[0]`` as collective thrust demand (T_c) and
        ``wrench.moment_body`` as [roll, pitch, yaw] demand. Pitch is preserved
        first (it is what keeps the wing out of a stall), roll second, and yaw
        (via differential thrust) last -- matching the plan's saturation-by-
        priority rule rather than naive clipping.
        """
        t_c = float(np.asarray(wrench.force_body, dtype=float)[0])
        roll_m, pitch_m, yaw_m = np.asarray(wrench.moment_body, dtype=float)

        delta_pitch = float(np.clip(pitch_m, -self.max_elevon_rad, self.max_elevon_rad))
        roll_headroom = self.max_elevon_rad - abs(delta_pitch)
        delta_roll = float(np.clip(roll_m, -roll_headroom, roll_headroom))
        elevon_l = delta_pitch + delta_roll
        elevon_r = delta_pitch - delta_roll

        t_c = float(np.clip(t_c, 0.0, self.max_thrust_n))
        q_c_demand = yaw_m / self.motor_separation_m
        yaw_headroom = min(self.max_thrust_n - t_c, t_c)
        q_c = float(np.clip(q_c_demand, -yaw_headroom, yaw_headroom))
        throttle_l = float(np.clip(t_c + q_c, 0.0, self.max_thrust_n))
        throttle_r = float(np.clip(t_c - q_c, 0.0, self.max_thrust_n))

        return np.array([throttle_l, throttle_r, elevon_l, elevon_r], dtype=float)

    def trim(self, state: SimState) -> np.ndarray:
        return np.array(
            [self.trim_throttle_n, self.trim_throttle_n, self.trim_elevon_rad, self.trim_elevon_rad],
            dtype=float,
        )
