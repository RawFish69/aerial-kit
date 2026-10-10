"""Geometric tracking control on SE(3) (Lee, Leok & McClamroch, 2010).

What it adds over :mod:`.cascade`
---------------------------------
The cascade is an acceleration controller with three loops bolted underneath:
it asks the inner controller for an acceleration, tilts the thrust axis towards
it, and closes attitude and rate loops on the *small-angle* attitude error with
a tilt limit to keep itself inside the region where that error means something.

This controller is one law from position to moment, written on the rotation
group itself:

* **Large attitudes are in scope.** The attitude error is the same
  ``e_R = 0.5 vee(R_d^T R - R^T R_d)`` - :func:`.cascade.attitude_error`, reused
  rather than restated - but the loop is designed around it rather than around
  its linearisation, and the result (Lee et al., Prop. 1) is exponential
  stability for any initial attitude error under 90 degrees and almost-global
  attractivity beyond it. A quadrotor tipped to 150 degrees comes back.
* **It tracks a trajectory, not a point.** Desired velocity and acceleration
  feed forward into the thrust, and with the desired *jerk* the desired body
  rate follows from differential flatness (Mellinger & Kumar, 2011), so a
  moving reference is followed without the lag a point stabiliser has.
* **The moment cancels the rigid-body coupling.** ``M`` includes
  ``Omega x J Omega``: the plant's Euler equation is
  ``J Omega_dot = M - Omega x J Omega``, so adding the term here leaves
  ``J Omega_dot = -k_R e_R - k_Omega e_Omega``, the linear error dynamics the
  stability proof is about. (The cascade omits it and is right to for its own
  design; here the proof needs it.)

Conventions are the repository's: world ENU, body x forward / y left / z up,
thrust ``+f`` along body z, quaternions w-first body-to-world, SI throughout.

Gains are given as bandwidths and scaled by mass and inertia, so they mean the
same thing on any airframe: ``omega_pos`` and ``omega_att`` in rad/s with
damping ratios ``zeta_pos`` and ``zeta_att``. The defaults keep the attitude
loop (12 rad/s) a factor under the actuator plant's 33 rad/s motor corner, and
the position loop a factor under that.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np

from ..dynamics.rotations import quat_to_rotmat, yaw_of
from ..interfaces import Controller
from ..types import ControlTarget, SimState, Waypoint, Wrench
from .cascade import attitude_error

E3 = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class FlatReference:
    """A point on a trajectory of the flat outputs: position, its derivatives, yaw.

    A quadrotor's state and inputs are algebraic functions of position, yaw and
    their derivatives (differential flatness); this is the slice of them a
    tracking controller needs. ``jerk`` and ``yaw_rate`` are optional: without
    them the desired body rate is taken as zero, which is exact at hover and a
    lag on an aggressive trajectory.
    """

    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    jerk: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float | None = None  # None: hold the current heading
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class GeometricGains:
    """Physical parameters and loop bandwidths. ``mass_kg`` has no default.

    A wrong mass is a wrong hover thrust (see :class:`.cascade.CascadeGains`
    for the longer version of that argument); the inertia scales the moment,
    so a wrong inertia is a wrong attitude bandwidth, which is gentler.
    """

    mass_kg: float
    inertia_kgm2: tuple[float, float, float] = (0.01, 0.01, 0.02)
    omega_pos: float = 2.5
    zeta_pos: float = 0.9
    omega_att: float = 12.0
    zeta_att: float = 0.8
    max_accel_mps2: float = 8.0  # bound on the feedback acceleration (keeps tilt moderate)
    gravity_mps2: float = 9.81
    gyroscopic_compensation: bool = True

    def __post_init__(self) -> None:
        if self.mass_kg <= 0.0:
            raise ValueError(f"mass_kg must be > 0, got {self.mass_kg}")
        if np.any(np.asarray(self.inertia_kgm2, dtype=float) <= 0.0):
            raise ValueError("every inertia component must be > 0")
        for name in ("omega_pos", "zeta_pos", "omega_att", "zeta_att", "max_accel_mps2"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be > 0, got {getattr(self, name)}")

    @property
    def inertia(self) -> np.ndarray:
        return np.diag(np.asarray(self.inertia_kgm2, dtype=float))

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any], section: str = "geometric") -> "GeometricGains":
        sub = dict((cfg.get("controller", {}) or {}).get(section, {}) or {})
        if "mass_kg" not in sub:
            raise ValueError(
                f"the {section} controller needs `controller.{section}.mass_kg` - "
                "it builds the hover thrust from it and will not guess it"
            )
        known = set(cls.__dataclass_fields__)
        unknown = sorted(set(sub) - known)
        if unknown:
            raise ValueError(f"unknown {section} setting(s) {unknown}; known are {sorted(known)}")
        if "inertia_kgm2" in sub:
            sub["inertia_kgm2"] = tuple(float(i) for i in sub["inertia_kgm2"])
        return cls(**sub)


def vee(skew: np.ndarray) -> np.ndarray:
    return np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)


def hat(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def desired_attitude(thrust_dir: np.ndarray, yaw: float) -> np.ndarray:
    """``R_d = [b1, b2, b3]`` with ``b3`` the thrust direction and ``b1`` as close
    to the heading ``yaw`` as ``b3`` allows (Lee et al., eq. 16-17).

    Unlike the cascade's version this is defined for any ``b3`` that is not
    parallel to the heading - including ones pointing below the horizon - which
    is what a large-angle controller needs from it.
    """
    b3 = np.asarray(thrust_dir, dtype=float).reshape(3)
    b3 = b3 / np.linalg.norm(b3)
    b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    b2 = np.cross(b3, b1c)
    n = float(np.linalg.norm(b2))
    if n < 1e-9:
        # Thrust along the heading: any perpendicular is as good as another.
        b2 = np.cross(b3, np.array([-np.sin(yaw), np.cos(yaw), 0.0]))
        n = float(np.linalg.norm(b2))
    b2 = b2 / n
    return np.column_stack([np.cross(b2, b3), b2, b3])


def flat_body_rates(
    rotation_d: np.ndarray, thrust_accel: float, jerk: np.ndarray, yaw: float, yaw_rate: float
) -> np.ndarray:
    """Desired body rate from jerk, yaw and yaw rate, by differential flatness.

    Roll and pitch rate (Mellinger & Kumar, 2011): with ``f/m`` the
    mass-normalised thrust along ``b3``, differentiating ``a + g e3 = (f/m) b3``
    gives ``j = (f/m)' b3 + (f/m) b3'``, and ``b3' = q b1 - p b2``, so the part
    of ``j`` perpendicular to ``b3``, divided by ``f/m``, yields
    ``p = -h.b2`` and ``q = h.b1``.

    Yaw rate, for *this* attitude construction - not Mellinger's
    ``r = yaw_rate * (b3 . e3)``, which belongs to his ZYX one and is wrong
    here by up to half the yaw rate (a finite-difference test caught it).
    :func:`desired_attitude` makes ``b1 = (c - (b3.c) b3) / n`` with ``c`` the
    heading and ``n`` that vector's norm; ``r = b2 . b1'``, and differentiating
    gives ``r = (yaw_rate * b2.c_perp + (b3.c) p) / n`` with
    ``c_perp = [-sin(yaw), cos(yaw), 0]``.
    """
    b1, b2, b3 = rotation_d[:, 0], rotation_d[:, 1], rotation_d[:, 2]
    if thrust_accel <= 1e-6:
        return np.zeros(3)
    j = np.asarray(jerk, dtype=float).reshape(3)
    h = (j - float(np.dot(b3, j)) * b3) / thrust_accel
    p = -float(np.dot(h, b2))
    q = float(np.dot(h, b1))
    c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    c_perp = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    n = float(np.linalg.norm(c - float(np.dot(b3, c)) * b3))
    r = 0.0 if n < 1e-9 else (yaw_rate * float(np.dot(b2, c_perp)) + float(np.dot(b3, c)) * p) / n
    return np.array([p, q, r])


class GeometricController(Controller):
    """SE(3) tracking controller producing a body-frame wrench.

    ``reference`` is an optional callable ``t -> FlatReference``. Without one,
    the target waypoint is held at rest with the current heading, which makes
    it a drop-in for any waypoint-following runner; with one, the waypoint is
    ignored and the trajectory is tracked with full feedforward.

    The returned ``ControlTarget.accel_cmd`` is the acceleration the commanded
    thrust produces along the *current* body axis, ``(f/m) R e3 - g e3``, so the
    ideal-acceleration backends can fly this controller too.
    """

    def __init__(
        self,
        gains: GeometricGains | None = None,
        reference: Callable[[float], FlatReference] | None = None,
    ) -> None:
        self.gains = gains
        self.reference = reference
        self.last_stages: dict[str, Any] = {}

    def compute(self, state: SimState, target_waypoint: Waypoint, cfg: Mapping[str, Any]) -> ControlTarget:
        g = self.gains if self.gains is not None else GeometricGains.from_config(cfg)
        R = _rotation_of(state)
        omega = _rates_of(state)
        p = np.asarray(state.position, dtype=float).reshape(3)
        v = np.asarray(state.velocity, dtype=float).reshape(3)

        if self.reference is not None:
            ref = self.reference(float(state.t))
        else:
            ref = FlatReference(position=np.asarray(target_waypoint.position, dtype=float))
        yaw_d = yaw_of(R) if ref.yaw is None else float(ref.yaw)

        # Translational loop: the force the thrust has to supply.
        kx = g.omega_pos ** 2
        kv = 2.0 * g.zeta_pos * g.omega_pos
        e_p = p - np.asarray(ref.position, dtype=float)
        e_v = v - np.asarray(ref.velocity, dtype=float)
        feedback = -kx * e_p - kv * e_v
        n = float(np.linalg.norm(feedback))
        if n > g.max_accel_mps2:
            feedback *= g.max_accel_mps2 / n
        accel_des = feedback + np.asarray(ref.acceleration, dtype=float)
        force_des = g.mass_kg * (accel_des + g.gravity_mps2 * E3)

        # Thrust is the projection on the *current* axis, so while the aircraft
        # is turning towards R_d it does not push hard in the wrong direction;
        # a rotor cannot pull, hence the floor at zero.
        thrust = max(float(np.dot(force_des, R @ E3)), 0.0)
        R_d = desired_attitude(force_des, yaw_d)

        thrust_accel_d = float(np.linalg.norm(force_des)) / g.mass_kg
        omega_d = flat_body_rates(R_d, thrust_accel_d, ref.jerk, yaw_d, ref.yaw_rate)

        # Rotational loop on SO(3).
        J = g.inertia
        k_R = g.omega_att ** 2
        k_W = 2.0 * g.zeta_att * g.omega_att
        e_R = attitude_error(R_d, R)
        RtRd = R.T @ R_d
        e_W = omega - RtRd @ omega_d
        alpha = -k_R * e_R - k_W * e_W - hat(omega) @ RtRd @ omega_d  # Omega_d_dot taken as 0
        moment = J @ alpha
        if g.gyroscopic_compensation:
            moment = moment + np.cross(omega, J @ omega)

        accel_cmd = (thrust / g.mass_kg) * (R @ E3) - g.gravity_mps2 * E3
        angle = float(np.degrees(np.arccos(np.clip((np.trace(RtRd) - 1.0) / 2.0, -1.0, 1.0))))
        self.last_stages = {
            "force_des": force_des,
            "thrust_n": thrust,
            "rotation_des": R_d,
            "attitude_error_rad": e_R,
            "attitude_error_deg": angle,
            "omega_des_rps": omega_d,
            "moment_body": moment,
        }
        return ControlTarget(
            accel_cmd=accel_cmd,
            wrench=Wrench(force_body=np.array([0.0, 0.0, thrust]), moment_body=moment),
            metadata={"controller": "geometric", "thrust_n": thrust, "attitude_error_deg": angle},
        )


def _rotation_of(state: SimState) -> np.ndarray:
    if state.attitude_quat is None:
        raise ValueError("an SE(3) controller needs an attitude; this state has attitude_quat=None")
    q = np.asarray(state.attitude_quat, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if not np.isfinite(n) or n < 1e-6:
        raise ValueError(f"attitude_quat {state.attitude_quat} is not a rotation")
    return quat_to_rotmat(q / n)


def _rates_of(state: SimState) -> np.ndarray:
    if state.body_rates is None:
        raise ValueError("an SE(3) controller needs body rates; pass zeros if at rest, not None")
    return np.asarray(state.body_rates, dtype=float).reshape(3)


__all__ = [
    "FlatReference",
    "GeometricController",
    "GeometricGains",
    "desired_attitude",
    "flat_body_rates",
    "hat",
    "vee",
]
