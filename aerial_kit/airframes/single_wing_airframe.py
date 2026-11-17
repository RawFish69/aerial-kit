"""Single-motor, two-elevon flying-wing airframe allocation."""

from __future__ import annotations

import math

import numpy as np

from aerial_kit.airframes.single_wing import SingleWingSpec, load_single_wing_spec
from aerial_kit.dynamics.single_wing import SingleWingDynamics
from aerial_kit.types import Capabilities, CommandKind, SimState, Wrench
from aerial_kit.airframes.base import Airframe


class SingleWingAirframe(Airframe):
    """Centreline pusher, two elevons and no dedicated yaw actuator.

    Yaw demand is intentionally dropped. This airframe must be flown as a
    coordinated banked-turn aircraft; its limited yaw authority comes only
    from aerodynamic weathervaning.
    """

    name = "single_wing"

    def __init__(
        self,
        spec: SingleWingSpec | None = None,
        cruise_airspeed_mps: float | None = None,
        min_airspeed_mps: float | None = None,
        max_airspeed_mps: float | None = None,
        max_climb_rate_mps: float | None = None,
        max_bank_deg: float | None = None,
    ) -> None:
        self.spec = spec or load_single_wing_spec()
        self.params = self.spec.dynamics_params()
        self.dynamics = SingleWingDynamics(self.params)
        envelope = self.spec.flight_envelope
        cruise_airspeed_mps = (
            float(envelope["cruise_airspeed_mps"])
            if cruise_airspeed_mps is None else cruise_airspeed_mps
        )
        min_airspeed_mps = (
            float(envelope["min_airspeed_mps"])
            if min_airspeed_mps is None else min_airspeed_mps
        )
        max_airspeed_mps = (
            float(envelope["max_airspeed_mps"])
            if max_airspeed_mps is None else max_airspeed_mps
        )
        max_climb_rate_mps = (
            float(envelope["max_climb_rate_mps"])
            if max_climb_rate_mps is None else max_climb_rate_mps
        )
        max_bank_deg = (
            float(envelope["max_bank_deg"])
            if max_bank_deg is None else max_bank_deg
        )
        self.cruise_airspeed_mps = float(cruise_airspeed_mps)
        self.min_airspeed_mps = float(min_airspeed_mps)
        self.max_airspeed_mps = float(max_airspeed_mps)
        self.max_thrust_n = self.spec.max_thrust_n
        self.max_elevon_rad = self.spec.max_elevon_rad
        if not (0.0 < max_bank_deg < 90.0):
            raise ValueError("max_bank_deg must be in (0, 90)")
        phi = math.radians(max_bank_deg)
        min_radius = self.cruise_airspeed_mps**2 / (
            self.params.gravity_mps2 * math.tan(phi))
        self.capabilities = Capabilities(
            can_hover=False,
            min_airspeed_mps=float(min_airspeed_mps),
            max_airspeed_mps=float(max_airspeed_mps),
            max_climb_rate_mps=float(max_climb_rate_mps),
            max_bank_deg=float(max_bank_deg),
            min_turn_radius_m=float(min_radius),
            n_actuators=3,
            command_kind=CommandKind.AIRSPEED_NAV,
        )

    def _airspeed(self, state: SimState) -> float:
        if state.airspeed_mps is not None and state.airspeed_mps > 0.0:
            measured = float(state.airspeed_mps)
        else:
            measured = float(np.linalg.norm(
                np.asarray(state.velocity, dtype=float)[:2]))
        if measured <= 1e-6:
            measured = self.cruise_airspeed_mps
        return float(np.clip(
            measured, self.min_airspeed_mps, self.max_airspeed_mps))

    def allocate(self, wrench: Wrench, state: SimState) -> np.ndarray:
        """[thrust_n, elevon_left_rad, elevon_right_rad], pitch first.

        Pitch has priority over roll. The pitch base includes the thrust-line
        offset; the roll base includes opposite motor-reaction torque. Yaw
        demand has no actuator and is discarded.
        """
        force = np.asarray(wrench.force_body, dtype=float)
        moment = np.asarray(wrench.moment_body, dtype=float)
        thrust = float(np.clip(force[0], 0.0, self.max_thrust_n))
        pitch_request = float(moment[1])
        roll_request = float(moment[0])
        airspeed = self._airspeed(state)
        common_trim, reaction_compensation, _ = self.dynamics.trim_moment_bases(
            airspeed, thrust)

        delta_pitch = float(np.clip(
            common_trim + pitch_request,
            -self.max_elevon_rad, self.max_elevon_rad,
        ))
        roll_headroom = self.max_elevon_rad - abs(delta_pitch)
        delta_roll = float(np.clip(
            reaction_compensation + roll_request,
            -roll_headroom, roll_headroom,
        ))
        return np.array([
            thrust,
            delta_pitch - delta_roll,
            delta_pitch + delta_roll,
        ], dtype=float)

    def trim(self, state: SimState) -> np.ndarray:
        return self.dynamics.compute_trim(self._airspeed(state))
