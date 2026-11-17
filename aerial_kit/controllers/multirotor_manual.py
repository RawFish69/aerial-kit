"""Pilot-facing multirotor control modes, built from the cascade's stages.

Three modes, the usual flight-controller set, each producing a z-up
:class:`~aerial_kit.types.Wrench` for the airframe's allocator:

``velocity`` (position-assist, research default)
    Sticks command a horizontal velocity in the heading frame, a climb rate
    and a yaw rate. Centred sticks hold position. Same law as the ROS
    ``cmd_twist`` path, so a mission and a pilot fly the same loop.
``angle`` (self-level + vertical-speed hold, FPV default)
    Roll/pitch sticks command a tilt angle, yaw a yaw rate, and throttle a
    climb rate (centre holds altitude). On a high thrust-to-weight FPV quad a
    raw-throttle angle mode is not flyable from digital keys: half stick above
    hover is several g of climb with nothing to stop it.
``acro`` (rate)
    Roll/pitch/yaw sticks command body rates, throttle the raw collective
    (centre = hover thrust). Nothing self-levels and nothing holds altitude.

Stick conventions (all in ``[-1, 1]``, throttle in ``[0, 1]``):
``pitch`` +1 is stick forward (nose down, fly forward); ``roll`` +1 is right;
``yaw`` +1 is clockwise seen from above (nose right); ``throttle`` 0.5 is
hover thrust (acro) or zero climb rate (velocity, angle).

Everything is ENU world / FLU body, like :mod:`.cascade`, whose stage
functions do the attitude and rate work here; nothing re-derives them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..dynamics.rotations import quat_to_rotmat, yaw_of
from ..types import SimState, Wrench
from .cascade import (
    CascadeGains,
    attitude_error,
    limit_tilt,
    rotation_from_thrust_and_yaw,
    thrust_vector,
)

MODES = ("velocity", "angle", "acro")


@dataclass(frozen=True)
class StickInput:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.5

    def clipped(self) -> "StickInput":
        vals = (self.roll, self.pitch, self.yaw, self.throttle)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError("stick input contains a non-finite value")
        return StickInput(
            roll=float(np.clip(self.roll, -1.0, 1.0)),
            pitch=float(np.clip(self.pitch, -1.0, 1.0)),
            yaw=float(np.clip(self.yaw, -1.0, 1.0)),
            throttle=float(np.clip(self.throttle, 0.0, 1.0)),
        )


@dataclass(frozen=True)
class ManualLimits:
    max_angle_deg: float = 30.0
    acro_rate_deg_s: float = 360.0
    yaw_rate_deg_s: float = 180.0
    max_speed_mps: float = 8.0
    max_climb_mps: float = 3.0
    velocity_kp: float = 2.0
    max_accel_mps2: float = 6.0
    max_collective_n: float = 40.0

    @classmethod
    def from_spec(cls, spec) -> "ManualLimits":
        c, m = spec.control, spec.control.get("manual", {})
        return cls(
            max_angle_deg=float(m.get("max_angle_deg", 30.0)),
            acro_rate_deg_s=float(m.get("acro_rate_deg_s", 360.0)),
            yaw_rate_deg_s=float(m.get("yaw_rate_deg_s", c.get("max_yaw_rate_deg_s", 180.0))),
            max_speed_mps=float(c.get("max_speed_mps", 8.0)),
            max_climb_mps=float(c.get("max_climb_mps", 3.0)),
            velocity_kp=float(c.get("velocity_kp", 2.0)),
            max_accel_mps2=float(c.get("max_accel_mps2", 6.0)),
            max_collective_n=float(spec.n_rotors * spec.max_thrust_n),
        )


class MultirotorPilot:
    """Holds a yaw setpoint and turns stick or velocity demands into a wrench."""

    def __init__(self, gains: CascadeGains, limits: ManualLimits) -> None:
        self.gains = gains
        self.limits = limits
        self.yaw_sp: float | None = None
        self.last: dict = {}

    def reset_yaw(self, state: SimState | None = None) -> None:
        self.yaw_sp = None if state is None else yaw_of(self._rotation(state))

    # ------------------------------------------------------------ helpers
    @staticmethod
    def _rotation(state: SimState) -> np.ndarray:
        quat = np.asarray(state.attitude_quat, dtype=float)
        return quat_to_rotmat(quat / np.linalg.norm(quat))

    def _advance_yaw(self, rotation: np.ndarray, yaw_rate: float, dt: float) -> float:
        if self.yaw_sp is None:
            self.yaw_sp = yaw_of(rotation)
        self.yaw_sp = float(self.yaw_sp + yaw_rate * dt)
        # Keep the setpoint within reach so a held yaw stick cannot wind up.
        err = math.atan2(math.sin(self.yaw_sp - yaw_of(rotation)), math.cos(self.yaw_sp - yaw_of(rotation)))
        if abs(err) > 0.5:
            self.yaw_sp = yaw_of(rotation) + math.copysign(0.5, err)
        return self.yaw_sp

    def _attitude_wrench(self, state: SimState, desired: np.ndarray, thrust_n: float,
                         yaw_rate_ff: float = 0.0) -> Wrench:
        rotation = self._rotation(state)
        rates = np.asarray(state.body_rates, dtype=float)
        error = attitude_error(desired, rotation)
        omega_des = -self.gains.k_att * error
        omega_des[2] += yaw_rate_ff
        return self._rate_wrench(rates, omega_des, thrust_n)

    def _rate_wrench(self, rates: np.ndarray, omega_des: np.ndarray, thrust_n: float) -> Wrench:
        moment = np.asarray(self.gains.inertia_kgm2, float) * self.gains.k_rate * (omega_des - rates)
        thrust_n = float(np.clip(thrust_n, 0.0, self.limits.max_collective_n))
        self.last.update(omega_des=omega_des, thrust_n=thrust_n)
        return Wrench(force_body=np.array([0.0, 0.0, thrust_n]), moment_body=moment)

    def _collective(self, throttle: float, tilt_cos: float) -> float:
        hover = self.gains.mass_kg * self.gains.gravity_mps2
        if throttle <= 0.5:
            base = hover * throttle / 0.5
        else:
            base = hover + (self.limits.max_collective_n - hover) * (throttle - 0.5) / 0.5
        return base / max(tilt_cos, 0.5)

    # --------------------------------------------------------------- modes
    def velocity_wrench(self, state: SimState, v_cmd_world, yaw_rate: float, dt: float) -> Wrench:
        """World-frame (ENU) velocity + yaw-rate demand -> wrench (the cmd_twist law)."""
        lim = self.limits
        rotation = self._rotation(state)
        v_cmd = np.asarray(v_cmd_world, dtype=float).reshape(3).copy()
        horiz = float(np.linalg.norm(v_cmd[:2]))
        if horiz > lim.max_speed_mps:
            v_cmd[:2] *= lim.max_speed_mps / horiz
        v_cmd[2] = float(np.clip(v_cmd[2], -lim.max_climb_mps, lim.max_climb_mps))
        accel = lim.velocity_kp * (v_cmd - np.asarray(state.velocity, dtype=float))
        norm = float(np.linalg.norm(accel))
        if norm > lim.max_accel_mps2:
            accel *= lim.max_accel_mps2 / norm
        thrust, z_body = thrust_vector(accel, self.gains)
        z_body, scale = limit_tilt(z_body, self.gains.max_tilt_deg)
        yaw = self._advance_yaw(rotation, yaw_rate, dt)
        desired = rotation_from_thrust_and_yaw(z_body, yaw)
        self.last = {"mode": "velocity", "accel_cmd": accel, "yaw_sp": yaw}
        return self._attitude_wrench(state, desired, thrust * scale, yaw_rate)

    def stick_wrench(self, state: SimState, mode: str, sticks: StickInput, dt: float) -> Wrench:
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
        s = sticks.clipped()
        lim = self.limits
        yaw_rate = -s.yaw * math.radians(lim.yaw_rate_deg_s)
        rotation = self._rotation(state)
        if mode == "velocity":
            heading = yaw_of(rotation)
            fwd = np.array([math.cos(heading), math.sin(heading), 0.0])
            right = np.array([math.sin(heading), -math.cos(heading), 0.0])
            v = lim.max_speed_mps * (s.pitch * fwd + s.roll * right)
            v[2] = lim.max_climb_mps * (s.throttle - 0.5) / 0.5
            return self.velocity_wrench(state, v, yaw_rate, dt)
        if mode == "angle":
            max_angle = math.radians(lim.max_angle_deg)
            roll, pitch = s.roll * max_angle, s.pitch * max_angle
            yaw = self._advance_yaw(rotation, yaw_rate, dt)
            # FLU: +roll about x lowers the right side; +pitch about y lowers the nose.
            desired = _rz(yaw) @ _ry(pitch) @ _rx(roll)
            vz_cmd = lim.max_climb_mps * (s.throttle - 0.5) / 0.5
            vz = float(np.asarray(state.velocity, dtype=float)[2])
            a_z = float(np.clip(lim.velocity_kp * (vz_cmd - vz), -0.8 * self.gains.gravity_mps2,
                                lim.max_accel_mps2))
            tilt_cos = max(math.cos(roll) * math.cos(pitch), 0.5)
            thrust = self.gains.mass_kg * (self.gains.gravity_mps2 + a_z) / tilt_cos
            self.last = {"mode": "angle", "yaw_sp": yaw, "roll": roll, "pitch": pitch}
            return self._attitude_wrench(state, desired, thrust, yaw_rate)
        rate = math.radians(lim.acro_rate_deg_s)
        omega_des = np.array([s.roll * rate, s.pitch * rate, yaw_rate])
        self.yaw_sp = None
        self.last = {"mode": "acro"}
        return self._rate_wrench(np.asarray(state.body_rates, float), omega_des, self._collective(s.throttle, 1.0))

    def hold_wrench(self, state: SimState, dt: float) -> Wrench:
        """Brake to a hover and hold heading (the safe fallback in the air)."""
        return self.velocity_wrench(state, np.zeros(3), 0.0, dt)


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
