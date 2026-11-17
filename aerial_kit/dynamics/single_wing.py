"""Single-motor flying-wing dynamics with switchable propulsion effects.

The rigid-body conventions are shared with ``fixed_wing.py``: body FRD, world
ENU, and ``attitude_quat`` maps body vectors into world. Lift, drag and stall
are reused from :mod:`aerial_kit.dynamics.fixed_wing` rather than copied.

All default coefficients and propulsion parameters originate in
``airframes/single_wing/single_wing.yaml`` and are research approximations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .fixed_wing import (
    FixedWingParams,
    MAX_BODY_RATE_RAD_S,
    _integrate_quaternion,
    level_attitude_quat,
    lift_coefficient,
    quat_to_rotmat,
)


@dataclass
class SingleWingParams(FixedWingParams):
    """Aero plus propulsion parameters for a centreline-pusher flying wing.

    The defaults duplicate ``single_wing.yaml`` intentionally so direct
    construction remains useful; ``test_single_wing_model.py`` fails if the two
    drift. Runtime consumers load the YAML through :class:`SingleWingSpec`.
    """

    mass_kg: float = 1.10
    wing_area_m2: float = 0.24
    wingspan_m: float = 1.20
    mean_chord_m: float = 0.20
    inertia_kg_m2: np.ndarray = field(
        default_factory=lambda: np.array([0.035, 0.022, 0.045]))
    air_density_kg_m3: float = 1.225
    gravity_mps2: float = 9.81
    cl0: float = 0.17
    cl_alpha: float = 5.30
    alpha_stall_rad: float = math.radians(14.0)
    stall_blend_rate: float = 25.0
    cd0: float = 0.030
    k_induced: float = 0.0663
    cm0: float = 0.0
    cm_alpha: float = -0.45
    cm_q: float = -7.5
    cm_delta_e: float = 0.62
    cl_beta: float = -0.05
    cl_p: float = -0.45
    cl_delta_a: float = 0.11
    cn_beta: float = 0.06
    cn_r: float = -0.06
    max_thrust_n: float = 8.0
    thrust_tau_s: float = 0.05
    max_elevon_rad: float = math.radians(25.0)
    thrust_axis: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    motor_position_m: np.ndarray = field(
        default_factory=lambda: np.array([-0.135, 0.0, -0.035]))
    prop_spin: int = 1
    reaction_torque_per_n_nm: float = 0.006
    prop_inertia_kg_m2: float = 8.0e-6
    motor_inertia_kg_m2: float = 4.0e-5
    prop_wash_gain: float = 0.08
    enable_motor_reaction: bool = True
    enable_thrust_lag: bool = True
    enable_prop_wash: bool = True
    enable_thrust_offset: bool = True

    def __post_init__(self) -> None:
        self.thrust_axis = np.asarray(self.thrust_axis, dtype=float)
        if self.thrust_axis.shape != (3,):
            raise ValueError("thrust_axis must have 3 components")
        norm = float(np.linalg.norm(self.thrust_axis))
        if norm < 1e-9:
            raise ValueError("thrust_axis must be nonzero")
        self.thrust_axis = self.thrust_axis / norm
        if self.thrust_axis[0] <= 1e-9:
            raise ValueError("thrust_axis must point forward")
        self.motor_position_m = np.asarray(self.motor_position_m, dtype=float)
        if self.motor_position_m.shape != (3,):
            raise ValueError("motor_position_m must have 3 components")
        if self.max_thrust_n <= 0.0:
            raise ValueError("max_thrust_n must be positive")
        if self.thrust_tau_s < 0.0:
            raise ValueError("thrust_tau_s must be non-negative")
        if self.prop_spin not in (-1, 1):
            raise ValueError("prop_spin must be +1 or -1")
        for name in (
            "reaction_torque_per_n_nm", "prop_inertia_kg_m2",
            "motor_inertia_kg_m2", "prop_wash_gain",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")


class SingleWingDynamics:
    """6-DOF wing with a centreline motor and two elevons.

    Actuator order is ``[thrust_n, elevon_left_rad, elevon_right_rad]``.
    Positive elevon is trailing-edge-up. The prop-wash model scales only the
    elevon moment terms; it intentionally does not add a second lift model.
    """

    def __init__(
        self,
        params: SingleWingParams | None = None,
        wind_mps: np.ndarray | None = None,
        thrust_axis: np.ndarray | None = None,
    ):
        self.params = params or SingleWingParams()
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.attitude_quat = level_attitude_quat()
        self.body_rates = np.zeros(3, dtype=float)
        self.wind_mps = (
            np.zeros(3, dtype=float)
            if wind_mps is None else np.asarray(wind_mps, dtype=float)
        )
        axis = self.params.thrust_axis if thrust_axis is None else thrust_axis
        self.thrust_axis = np.asarray(axis, dtype=float)
        self.thrust_axis /= np.linalg.norm(self.thrust_axis)
        if self.thrust_axis[0] <= 1e-9:
            raise ValueError("thrust axis needs a forward x component")
        self.thrust_n = 0.0

    def airspeed_alpha_beta(self) -> tuple[float, float, float]:
        rot = quat_to_rotmat(self.attitude_quat)
        v_air_body = rot.T @ (self.velocity - self.wind_mps)
        u, v, w = v_air_body
        airspeed = float(np.linalg.norm(v_air_body))
        alpha = float(np.arctan2(w, u)) if airspeed > 1e-6 else 0.0
        beta = (
            float(np.arcsin(np.clip(v / airspeed, -1.0, 1.0)))
            if airspeed > 1e-6 else 0.0
        )
        return airspeed, alpha, beta

    @staticmethod
    def _command(cmd: np.ndarray) -> np.ndarray:
        out = np.asarray(cmd, dtype=float).reshape(-1)
        if out.shape != (3,):
            raise ValueError("single-wing actuator command must have 3 values")
        if not np.all(np.isfinite(out)):
            raise ValueError("single-wing actuator command is non-finite")
        return out

    def wash_authority_scale(self, airspeed_mps: float, thrust_n: float) -> float:
        """Elevon authority multiplier from pusher-prop wash.

        The deliberately simple scale is
        ``1 + gain * thrust / (qbar * S)``. It is small because the propeller
        sits behind the wing and elevons; it becomes useful only at low
        dynamic pressure and high thrust.
        """
        if not self.params.enable_prop_wash:
            return 1.0
        qbar_s = (
            0.5 * self.params.air_density_kg_m3 * max(airspeed_mps, 1e-6) ** 2
            * self.params.wing_area_m2
        )
        raw = 1.0 + self.params.prop_wash_gain * max(float(thrust_n), 0.0) / max(qbar_s, 1e-6)
        return float(np.clip(raw, 1.0, 4.0))

    def _aero_forces_moments(
        self, cmd: np.ndarray, thrust_n: float
    ) -> tuple[np.ndarray, np.ndarray]:
        command = self._command(cmd)
        p = self.params
        elevon_left = float(command[1])
        elevon_right = float(command[2])
        delta_e = 0.5 * (elevon_left + elevon_right)
        delta_a = 0.5 * (elevon_right - elevon_left)

        airspeed, alpha, beta = self.airspeed_alpha_beta()
        qbar = 0.5 * p.air_density_kg_m3 * airspeed**2
        wash = self.wash_authority_scale(airspeed, thrust_n)
        p_rate, q_rate, r_rate = self.body_rates

        cl = lift_coefficient(alpha, p)
        cd = p.cd0 + p.k_induced * cl**2
        lift = qbar * p.wing_area_m2 * cl
        drag = qbar * p.wing_area_m2 * cd
        fx_aero = -drag * math.cos(alpha) + lift * math.sin(alpha)
        fz_aero = -drag * math.sin(alpha) - lift * math.cos(alpha)
        fy_aero = (
            qbar * p.wing_area_m2 * (0.5 * p.cn_beta) * beta
        )

        qhat = q_rate * p.mean_chord_m / (2.0 * airspeed) if airspeed > 1e-6 else 0.0
        phat = p_rate * p.wingspan_m / (2.0 * airspeed) if airspeed > 1e-6 else 0.0
        rhat = r_rate * p.wingspan_m / (2.0 * airspeed) if airspeed > 1e-6 else 0.0

        cl_roll = p.cl_beta * beta + p.cl_p * phat + wash * p.cl_delta_a * delta_a
        cm_pitch = (
            p.cm0 + p.cm_alpha * alpha + p.cm_q * qhat
            + wash * p.cm_delta_e * delta_e
        )
        cn_yaw = p.cn_beta * beta + p.cn_r * rhat
        roll_aero = qbar * p.wing_area_m2 * p.wingspan_m * cl_roll
        pitch_aero = qbar * p.wing_area_m2 * p.mean_chord_m * cm_pitch
        yaw_aero = qbar * p.wing_area_m2 * p.wingspan_m * cn_yaw

        thrust_force = self.thrust_axis * float(thrust_n)
        force_body = np.array(
            [fx_aero + thrust_force[0], fy_aero + thrust_force[1],
             fz_aero + thrust_force[2]], dtype=float)
        moment = np.array([roll_aero, pitch_aero, yaw_aero], dtype=float)

        if p.enable_thrust_offset:
            moment += np.cross(p.motor_position_m, thrust_force)
        reaction = np.zeros(3, dtype=float)
        if p.enable_motor_reaction:
            reaction = (
                -float(p.prop_spin) * p.reaction_torque_per_n_nm
                * float(thrust_n) * self.thrust_axis
            )
            moment += reaction
        return force_body, moment

    def aero_forces_moments(
        self, cmd: np.ndarray, thrust_n: float | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """Body-FRD force/moment for an actuator command.

        With no ``thrust_n`` argument this evaluates the commanded thrust
        instantaneously. :meth:`step` passes the lagged thrust instead.
        """
        command = self._command(cmd)
        thrust = float(command[0]) if thrust_n is None else float(thrust_n)
        return self._aero_forces_moments(command, thrust)

    def update_thrust(self, commanded_thrust_n: float, dt: float) -> float:
        p = self.params
        target = float(np.clip(commanded_thrust_n, 0.0, p.max_thrust_n))
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        if not p.enable_thrust_lag or p.thrust_tau_s == 0.0:
            self.thrust_n = target
        else:
            alpha = 1.0 - math.exp(-dt / p.thrust_tau_s)
            self.thrust_n += alpha * (target - self.thrust_n)
        return self.thrust_n

    def _level_force_residual(
        self, alpha_rad: float, speed_mps: float, thrust_n: float
    ) -> float:
        """World-z force at a candidate pitch angle and fixed thrust.

        The aircraft is assumed to have zero body rates, zero sideslip and its
        air-relative velocity along world +x. This helper is used only by trim;
        it deliberately contains no control, gust or rate-loop dynamics.
        """
        p = self.params
        cl = lift_coefficient(alpha_rad, p)
        cd = p.cd0 + p.k_induced * cl**2
        qbar = 0.5 * p.air_density_kg_m3 * speed_mps**2
        lift = qbar * p.wing_area_m2 * cl
        drag = qbar * p.wing_area_m2 * cd
        force_x = -drag * math.cos(alpha_rad) + lift * math.sin(alpha_rad)
        force_z = -drag * math.sin(alpha_rad) - lift * math.cos(alpha_rad)
        thrust_body = self.thrust_axis * thrust_n
        total_x = force_x + thrust_body[0]
        total_z = force_z + thrust_body[2]
        c = math.cos(alpha_rad)
        s = math.sin(alpha_rad)
        return (
            s * total_x - c * total_z
            - p.mass_kg * p.gravity_mps2
        )

    def _trim_alpha_for_thrust(
        self, speed_mps: float, thrust_n: float
    ) -> float:
        """Solve the level-flight force balance for alpha at fixed thrust."""
        p = self.params
        limit = 0.95 * p.alpha_stall_rad
        grid = np.linspace(-limit, limit, 161)
        residuals = np.array([
            self._level_force_residual(float(alpha), speed_mps, thrust_n)
            for alpha in grid
        ])
        crossings = np.flatnonzero(residuals[:-1] * residuals[1:] <= 0.0)
        if crossings.size == 0:
            raise ValueError(
                "no pre-stall level-flight trim for this airspeed and thrust")
        index = int(crossings[0])
        lower = float(grid[index])
        upper = float(grid[index + 1])
        lower_value = float(residuals[index])
        if abs(lower_value) < 1e-12:
            return lower
        for _ in range(80):
            middle = 0.5 * (lower + upper)
            middle_value = self._level_force_residual(
                middle, speed_mps, thrust_n)
            if lower_value * middle_value <= 0.0:
                upper = middle
            else:
                lower = middle
                lower_value = middle_value
        return 0.5 * (lower + upper)

    def step(self, actuator_cmd: np.ndarray, dt: float) -> None:
        command = self._command(actuator_cmd)
        self.update_thrust(float(command[0]), dt)
        force_body, moment_body = self._aero_forces_moments(command, self.thrust_n)

        rot = quat_to_rotmat(self.attitude_quat)
        accel_world = (
            rot @ force_body / self.params.mass_kg
            + np.array([0.0, 0.0, -self.params.gravity_mps2])
        )
        ixx, iyy, izz = self.params.inertia_kg_m2
        p_rate, q_rate, r_rate = self.body_rates
        l_m, m_m, n_m = moment_body
        p_dot = (iyy - izz) / ixx * q_rate * r_rate + l_m / ixx
        q_dot = (izz - ixx) / iyy * p_rate * r_rate + m_m / iyy
        r_dot = (ixx - iyy) / izz * p_rate * q_rate + n_m / izz

        self.position += self.velocity * dt
        self.velocity += accel_world * dt
        self.attitude_quat = _integrate_quaternion(
            self.attitude_quat, self.body_rates, dt)
        self.body_rates = np.clip(
            self.body_rates + np.array([p_dot, q_dot, r_dot]) * dt,
            -MAX_BODY_RATE_RAD_S, MAX_BODY_RATE_RAD_S)

    def _trim_at_thrust(
        self,
        cruise_airspeed_mps: float,
        thrust_n: float,
        alpha_rad: float | None = None,
    ) -> dict[str, float]:
        p = self.params
        speed = float(cruise_airspeed_mps)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("cruise airspeed must be finite and positive")
        qbar = 0.5 * p.air_density_kg_m3 * speed**2
        if qbar <= 0.0:
            raise ValueError("cruise airspeed produces no dynamic pressure")
        alpha = (
            self._trim_alpha_for_thrust(speed, float(thrust_n))
            if alpha_rad is None else float(alpha_rad)
        )
        cl_trim = lift_coefficient(alpha, p)
        if alpha >= p.alpha_stall_rad:
            raise ValueError("requested cruise airspeed is below the modelled stall")
        wash = self.wash_authority_scale(speed, thrust_n)
        thrust_force = self.thrust_axis * thrust_n
        offset_moment = (
            np.cross(p.motor_position_m, thrust_force)
            if p.enable_thrust_offset else np.zeros(3)
        )
        reaction = (
            -float(p.prop_spin) * p.reaction_torque_per_n_nm * thrust_n
            * self.thrust_axis
            if p.enable_motor_reaction else np.zeros(3)
        )

        pitch_required = -(
            p.cm0 + p.cm_alpha * alpha
            + offset_moment[1] / (qbar * p.wing_area_m2 * p.mean_chord_m)
        )
        denominator_pitch = wash * p.cm_delta_e
        if abs(denominator_pitch) < 1e-9:
            raise ValueError("elevon pitch effectiveness is zero")
        delta_e = pitch_required / denominator_pitch

        roll_required = -(reaction[0] + offset_moment[0])
        denominator_roll = (
            qbar * p.wing_area_m2 * p.wingspan_m
            * wash * p.cl_delta_a
        )
        if abs(denominator_roll) < 1e-9:
            raise ValueError("elevon roll effectiveness is zero")
        delta_a = roll_required / denominator_roll
        left = delta_e - delta_a
        right = delta_e + delta_a
        if max(abs(left), abs(right)) > p.max_elevon_rad:
            raise ValueError("computed trim requires more than commanded elevon travel")
        return {
            "airspeed_mps": speed,
            "alpha_rad": float(alpha),
            "cl": float(cl_trim),
            "thrust_n": float(thrust_n),
            "elevon_common_rad": float(delta_e),
            "elevon_differential_rad": float(delta_a),
            "elevon_left_rad": float(left),
            "elevon_right_rad": float(right),
            "wash_authority_scale": float(wash),
            "reaction_roll_moment_nm": float(reaction[0]),
            "thrust_offset_pitch_moment_nm": float(offset_moment[1]),
        }

    def trim_moment_bases(
        self, cruise_airspeed_mps: float, thrust_n: float
    ) -> tuple[float, float, float]:
        """(common elevon, reaction-compensation differential, wash scale)."""
        trim = self._trim_at_thrust(cruise_airspeed_mps, thrust_n)
        return (
            trim["elevon_common_rad"],
            trim["elevon_differential_rad"],
            trim["wash_authority_scale"],
        )

    def compute_trim_detailed(self, cruise_airspeed_mps: float) -> dict[str, float]:
        speed = float(cruise_airspeed_mps)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("cruise airspeed must be finite and positive")
        p = self.params
        limit = 0.95 * p.alpha_stall_rad

        def solve_thrust(alpha: float) -> tuple[float, float]:
            qbar = 0.5 * p.air_density_kg_m3 * speed**2
            cl = lift_coefficient(alpha, p)
            cd = p.cd0 + p.k_induced * cl**2
            lift = qbar * p.wing_area_m2 * cl
            drag = qbar * p.wing_area_m2 * cd
            fx_aero = -drag * math.cos(alpha) + lift * math.sin(alpha)
            fz_aero = -drag * math.sin(alpha) - lift * math.cos(alpha)
            c, s = math.cos(alpha), math.sin(alpha)
            thrust_denominator = (
                c * self.thrust_axis[0] + s * self.thrust_axis[2])
            if abs(thrust_denominator) < 1e-9:
                raise ValueError("thrust axis is tangent to the trim force direction")
            thrust = -(c * fx_aero + s * fz_aero) / thrust_denominator
            return float(thrust), self._level_force_residual(alpha, speed, thrust)

        grid = np.linspace(-limit, limit, 161)
        residuals = np.array([solve_thrust(float(alpha))[1] for alpha in grid])
        crossings = np.flatnonzero(residuals[:-1] * residuals[1:] <= 0.0)
        if crossings.size == 0:
            raise ValueError("no pre-stall level-flight trim for this airspeed")
        index = int(crossings[0])
        lower = float(grid[index])
        upper = float(grid[index + 1])
        lower_value = float(residuals[index])
        for _ in range(80):
            middle = 0.5 * (lower + upper)
            middle_value = solve_thrust(middle)[1]
            if lower_value * middle_value <= 0.0:
                upper = middle
            else:
                lower = middle
                lower_value = middle_value
        alpha = 0.5 * (lower + upper)
        thrust = solve_thrust(alpha)[0]
        if thrust < 0.0 or thrust > p.max_thrust_n:
            raise ValueError("cruise drag requires thrust outside the motor's range")
        return self._trim_at_thrust(speed, thrust, alpha_rad=alpha)

    def compute_trim(self, cruise_airspeed_mps: float) -> np.ndarray:
        trim = self.compute_trim_detailed(cruise_airspeed_mps)
        return np.array([
            trim["thrust_n"], trim["elevon_left_rad"],
            trim["elevon_right_rad"]], dtype=float)
