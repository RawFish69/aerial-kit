"""6-DOF rigid-body dynamics with a hand-rolled aero model for the twin wing.

Body axes are FRD (x-forward, y-right, z-down) -- the standard convention for
the alpha/beta/lift/drag formulas below, which follow the small-UAV model in
Beard & McLain, *Small Unmanned Aircraft* (the flat-plate stall blend, the
alpha-only wind-to-body force transform, and the non-dimensional rate terms
are all from that source). World position/velocity stay ENU (z-up), matching
every other backend and ``SimState``; ``attitude_quat`` is the rotation that
maps body-FRD vectors into world-ENU vectors, so it is *not* the identity at
level attitude (level, heading +world-x, is a 180-degree rotation about the
shared x-axis -- see ``level_attitude_quat``).

Aero/inertia coefficients are a plausible default set for a ~1.5 kg foam
flying wing, not a fitted model of any real airframe -- plan 04's own Risks
section flags this. They are constructor arguments (``FixedWingParams``), not
hardcoded, so they can be replaced with fitted values later without touching
this module.

Wind is a world-frame term applied only when computing the air-relative
velocity used for aero forces (workstream 02's open decision: wind-in-world,
not a new ``SimState`` field) -- ``velocity`` itself stays groundspeed.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

#: Body-rate magnitude bound applied every step -- see the comment at its use
#: in FixedWingDynamics.step(). ~4x anything seen in a hard sustained
#: maneuver during normal (even aggressive) flight; a real airframe never
#: approaches it, it only stops forward Euler's gyroscopic cross-coupling
#: term from diverging to inf/nan under a sustained extreme command.
MAX_BODY_RATE_RAD_S = 15.0


@dataclass
class FixedWingParams:
    mass_kg: float = 1.5
    wing_area_m2: float = 0.35
    wingspan_m: float = 1.4
    mean_chord_m: float = 0.25
    inertia_kg_m2: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.03, 0.06]))
    air_density_kg_m3: float = 1.225
    gravity_mps2: float = 9.81

    # Lift: linear region + flat-plate blend past stall.
    cl0: float = 0.15
    cl_alpha: float = 5.5  # per rad
    alpha_stall_rad: float = np.radians(15.0)
    stall_blend_rate: float = 25.0  # steepness of the sigmoid blend

    # Drag polar.
    cd0: float = 0.03
    k_induced: float = 0.0711  # 1/(pi * e * AR), e=0.8, AR=b^2/S=5.6

    # Pitch moment. cm_delta_e is positive: positive elevon deflection is
    # defined as trailing-edge-up / nose-up (matching the sign convention
    # TwinWingAirframe.allocate() already documents), so a positive commanded
    # moment must map to a positive elevon-driven pitch moment here too.
    cm0: float = 0.0
    cm_alpha: float = -0.5
    cm_q: float = -8.0
    cm_delta_e: float = 0.6

    # Roll moment.
    cl_beta: float = -0.05
    cl_p: float = -0.4
    cl_delta_a: float = 0.12

    # Yaw moment (aero only -- differential thrust is handled separately).
    cn_beta: float = 0.06
    cn_r: float = -0.05

    motor_separation_m: float = 0.6


def level_attitude_quat(heading_rad: float = 0.0) -> np.ndarray:
    """Quaternion for level flight at the given world heading (radians from +x).

    Body-FRD needs a 180-degree rotation about the shared x-axis just to align
    body-down with world -z (ENU is z-up); heading is then a rotation about
    world-z composed on top of that base roll. Composing
    ``q_z(heading) (x) q_x(180)`` collapses to ``(0, cos(h/2), sin(h/2), 0)``.
    """
    half = 0.5 * heading_rad
    return np.array([0.0, np.cos(half), np.sin(half), 0.0], dtype=float)


def quat_to_rotmat(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )


def _integrate_quaternion(quat: np.ndarray, body_rates: np.ndarray, dt: float) -> np.ndarray:
    w, x, y, z = quat
    p, q, r = body_rates
    dq = 0.5 * np.array(
        [
            -x * p - y * q - z * r,
            w * p + y * r - z * q,
            w * q - x * r + z * p,
            w * r + x * q - y * p,
        ]
    )
    quat = quat + dt * dq
    return quat / np.linalg.norm(quat)


def lift_coefficient(alpha: float, params: FixedWingParams) -> float:
    """Linear lift curve blended into a flat-plate model past stall.

    A pure linear ``CL0 + CLalpha*alpha`` keeps climbing without bound at high
    alpha, which is nonsense; a pure flat-plate model is wrong for the small
    attached-flow angles that dominate normal flight. The sigmoid blend
    (Beard & McLain) gives attached-flow lift near trim and a bounded,
    physically-sane falloff once alpha passes the stall angle.
    """
    a0 = params.alpha_stall_rad
    m = params.stall_blend_rate
    sigma = (1.0 + np.exp(-m * (alpha - a0)) + np.exp(m * (alpha + a0))) / (
        (1.0 + np.exp(-m * (alpha - a0))) * (1.0 + np.exp(m * (alpha + a0)))
    )
    cl_linear = params.cl0 + params.cl_alpha * alpha
    cl_flat_plate = 2.0 * np.sign(alpha) * np.sin(alpha) ** 2 * np.cos(alpha)
    return float((1.0 - sigma) * cl_linear + sigma * cl_flat_plate)


class FixedWingDynamics:
    """6-DOF rigid body + linear/flat-plate-blend aero for the twin wing.

    State (mutable attributes, matching ``UAVDynamics``/``PointMassDynamics``
    style): ``position`` (world ENU, m), ``velocity`` (world ENU, m/s),
    ``attitude_quat`` ([w, x, y, z], body-FRD -> world-ENU), ``body_rates``
    ([p, q, r], body-FRD rad/s).
    """

    def __init__(self, params: FixedWingParams | None = None, wind_mps: np.ndarray | None = None):
        self.params = params or FixedWingParams()
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.attitude_quat = level_attitude_quat()
        self.body_rates = np.zeros(3, dtype=float)
        self.wind_mps = np.zeros(3, dtype=float) if wind_mps is None else np.asarray(wind_mps, dtype=float)

    def airspeed_alpha_beta(self) -> tuple[float, float, float]:
        """Airspeed (m/s), angle of attack, sideslip -- all body-FRD, air-relative."""
        rot = quat_to_rotmat(self.attitude_quat)
        v_air_body = rot.T @ (self.velocity - self.wind_mps)
        u, v, w = v_air_body
        airspeed = float(np.linalg.norm(v_air_body))
        alpha = float(np.arctan2(w, u)) if airspeed > 1e-6 else 0.0
        beta = float(np.arcsin(np.clip(v / airspeed, -1.0, 1.0))) if airspeed > 1e-6 else 0.0
        return airspeed, alpha, beta

    def _aero_forces_moments(self, actuator_cmd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        p = self.params
        throttle_l, throttle_r, elevon_l, elevon_r = np.asarray(actuator_cmd, dtype=float)
        delta_e = 0.5 * (elevon_l + elevon_r)
        delta_a = 0.5 * (elevon_l - elevon_r)

        airspeed, alpha, beta = self.airspeed_alpha_beta()
        qbar = 0.5 * p.air_density_kg_m3 * airspeed**2
        p_rate, q_rate, r_rate = self.body_rates

        cl = lift_coefficient(alpha, p)
        cd = p.cd0 + p.k_induced * cl**2
        lift = qbar * p.wing_area_m2 * cl
        drag = qbar * p.wing_area_m2 * cd

        # Standard alpha-only wind-to-body transform (FRD); side force ignores
        # alpha coupling, which is accurate enough for the small-beta regime
        # this model targets.
        fx_aero = -drag * np.cos(alpha) + lift * np.sin(alpha)
        fz_aero = -drag * np.sin(alpha) - lift * np.cos(alpha)
        cy_beta = 0.5 * p.cn_beta  # weak side force, shares sign with yaw weathervaning
        fy_aero = qbar * p.wing_area_m2 * cy_beta * beta

        qhat = q_rate * p.mean_chord_m / (2.0 * airspeed) if airspeed > 1e-6 else 0.0
        phat = p_rate * p.wingspan_m / (2.0 * airspeed) if airspeed > 1e-6 else 0.0
        rhat = r_rate * p.wingspan_m / (2.0 * airspeed) if airspeed > 1e-6 else 0.0

        cl_roll = p.cl_beta * beta + p.cl_p * phat + p.cl_delta_a * delta_a
        cm_pitch = p.cm0 + p.cm_alpha * alpha + p.cm_q * qhat + p.cm_delta_e * delta_e
        cn_yaw = p.cn_beta * beta + p.cn_r * rhat

        roll_moment = qbar * p.wing_area_m2 * p.wingspan_m * cl_roll
        pitch_moment = qbar * p.wing_area_m2 * p.mean_chord_m * cm_pitch
        yaw_moment_aero = qbar * p.wing_area_m2 * p.wingspan_m * cn_yaw

        # Exact inverse of TwinWingAirframe.allocate()'s differential-thrust
        # formula (Q_c = yaw_m / motor_separation_m), so an unsaturated wrench
        # commanded through the airframe reproduces the same body yaw moment
        # here, rather than re-deriving a thrust moment-arm sign from scratch.
        yaw_moment_thrust = (throttle_l - throttle_r) * (p.motor_separation_m / 2.0)

        thrust_x = throttle_l + throttle_r
        force_body = np.array([fx_aero + thrust_x, fy_aero, fz_aero], dtype=float)
        moment_body = np.array([roll_moment, pitch_moment, yaw_moment_aero + yaw_moment_thrust], dtype=float)
        return force_body, moment_body

    def step(self, actuator_cmd: np.ndarray, dt: float) -> None:
        """Advance state by ``dt`` using forward Euler.

        Euler (not RK4) matches the rest of the codebase's dynamics classes
        and keeps the model inspectable; at the small dt sim configs already
        use this is adequate; revisit if energy drift shows up at larger dt.
        """
        p = self.params
        force_body, moment_body = self._aero_forces_moments(actuator_cmd)

        rot = quat_to_rotmat(self.attitude_quat)
        gravity_world = np.array([0.0, 0.0, -p.gravity_mps2])
        accel_world = (rot @ force_body) / p.mass_kg + gravity_world

        ixx, iyy, izz = p.inertia_kg_m2
        p_rate, q_rate, r_rate = self.body_rates
        l_m, m_m, n_m = moment_body
        p_dot = (iyy - izz) / ixx * q_rate * r_rate + l_m / ixx
        q_dot = (izz - ixx) / iyy * p_rate * r_rate + m_m / iyy
        r_dot = (ixx - iyy) / izz * p_rate * q_rate + n_m / izz

        self.position = self.position + self.velocity * dt
        self.velocity = self.velocity + accel_world * dt
        self.attitude_quat = _integrate_quaternion(self.attitude_quat, self.body_rates, dt)
        body_rates = self.body_rates + np.array([p_dot, q_dot, r_dot]) * dt
        # p_dot/q_dot/r_dot's gyroscopic cross-coupling terms (q*r, p*r, p*q)
        # have aero damping (cl_p, cm_q, cn_r) but nothing damps the coupling
        # itself, so a sustained extreme input -- full aileron held for
        # several seconds, e.g. -- can grow it faster than forward Euler's
        # fixed dt can track, diverging exponentially rather than saturating.
        # A real airframe's rates don't exceed this regardless of input; the
        # clamp exists purely to keep the explicit integrator from overflowing
        # to inf/nan once something well outside the model's flyable envelope
        # is commanded, not because rates approaching it are meant to be flown.
        self.body_rates = np.clip(body_rates, -MAX_BODY_RATE_RAD_S, MAX_BODY_RATE_RAD_S)

    def compute_trim(self, cruise_airspeed_mps: float) -> np.ndarray:
        """Closed-form level-cruise trim: ``[throttle_L, throttle_R, elevon_L, elevon_R]``.

        Assumes small-angle, pre-stall trim (linear lift region, zero body
        rates, zero bank/sideslip) -- valid for cruise, not for stall-adjacent
        or high-AoA conditions.
        """
        p = self.params
        qbar = 0.5 * p.air_density_kg_m3 * cruise_airspeed_mps**2
        weight = p.mass_kg * p.gravity_mps2
        cl_trim = weight / (qbar * p.wing_area_m2)
        alpha_trim = (cl_trim - p.cl0) / p.cl_alpha
        cd_trim = p.cd0 + p.k_induced * cl_trim**2
        drag_trim = qbar * p.wing_area_m2 * cd_trim
        delta_e_trim = -(p.cm0 + p.cm_alpha * alpha_trim) / p.cm_delta_e

        throttle_each = drag_trim / 2.0
        return np.array(
            [throttle_each, throttle_each, delta_e_trim, delta_e_trim],
            dtype=float,
        )
