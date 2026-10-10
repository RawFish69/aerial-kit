"""Nonlinear MPC on the quadrotor's full translational and attitude dynamics.

:mod:`.mpc` plans accelerations for a point mass and leaves attitude to
whatever sits underneath. That is the right model while the aircraft can tilt
much faster than its plan changes - and the wrong one exactly when the plan is
aggressive, because then the time it takes to *rotate* the thrust is the
limiting factor and a point-mass plan does not know it exists.

This one plans in the inputs a multirotor's attitude controller really
accepts: **mass-normalised collective thrust and body rates**,

    state  x = [p (3), v (3), q (4, w-first), w (3)]  world ENU, z up; w body rates
    input  u = [c, wx_cmd, wy_cmd, wz_cmd]            c in m/s^2, rates in rad/s, body z-up
    p' = v,  v' = c R(q) e3 - g e3,  q' = 0.5 q (x) [0, w],  w' = (w_cmd - w) / tau

The last line is the body-rate loop's response, first order with time
constant ``tau``. Without it the plan assumes a commanded rate takes effect at
once; the real rate loop and motors lag it, and the first version of this
controller planned a 45-degree tilt and flew 74. ``tau`` is ``1 / k_rate``, the
bandwidth of the rate loop this controller itself closes.

This is the model that betaflight-style rate controllers, PX4's offboard rate
setpoints and most agile-flight NMPC papers (Kamel 2017, Foehn 2021) share. The
thrust can only be rotated as fast as the rate bounds allow, and the plan
respects that.

Solver: iterative LQR (Li & Todorov 2004) with box-clamped inputs, written for
numpy alone. Each iteration linearises the RK4-discretised dynamics along the
current plan (by finite differences, batched over the whole horizon in one
vectorised call), solves the time-varying LQ problem backwards, and line-
searches a forward rollout. Between control steps the plan is shifted and
reused, so steady flight needs one or two iterations.

What reaches the aircraft is the first input of the plan, through a rate loop:
``M = J k_rate (w_des - w) + w x J w`` and ``f = m c`` - the same wrench
contract as :class:`.cascade.CascadeController` and
:class:`.geometric.GeometricController`, so the actuator loop flies all three.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np

from ..dynamics.rotations import quat_to_rotmat, yaw_of
from ..interfaces import Controller
from ..types import ControlTarget, SimState, Waypoint, Wrench
from .cascade import attitude_error
from .geometric import FlatReference

NX, NU = 13, 4
G = 9.81


# ---------------------------------------------------------------------------
# Model (batched over any leading dimensions)
# ---------------------------------------------------------------------------


def _rot_e3(q: np.ndarray) -> np.ndarray:
    """``R(q) e3`` - the body z axis in world - for quaternions ``(..., 4)``."""
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)], axis=-1)


def quad_dynamics(x: np.ndarray, u: np.ndarray, gravity: float = G, rate_tau: float = 1.0 / 15.0) -> np.ndarray:
    """Continuous-time ``x' = f(x, u)``, batched over leading dimensions."""
    v = x[..., 3:6]
    q = x[..., 6:10]
    w = x[..., 10:13]
    c = u[..., 0:1]
    wx, wy, wz = w[..., 0], w[..., 1], w[..., 2]
    acc = c * _rot_e3(q)
    acc[..., 2] -= gravity
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    # q' = 0.5 * q (x) (0, w): the body-rate form, because w is in body axes.
    dq = 0.5 * np.stack(
        [
            -qx * wx - qy * wy - qz * wz,
            qw * wx + qy * wz - qz * wy,
            qw * wy - qx * wz + qz * wx,
            qw * wz + qx * wy - qy * wx,
        ],
        axis=-1,
    )
    dw = (u[..., 1:4] - w) / rate_tau
    return np.concatenate([v, acc, dq, dw], axis=-1)


def rk4_step(x: np.ndarray, u: np.ndarray, dt: float, gravity: float = G, rate_tau: float = 1.0 / 15.0) -> np.ndarray:
    """One RK4 step with the input held, and the quaternion renormalised."""
    k1 = quad_dynamics(x, u, gravity, rate_tau)
    k2 = quad_dynamics(x + 0.5 * dt * k1, u, gravity, rate_tau)
    k3 = quad_dynamics(x + 0.5 * dt * k2, u, gravity, rate_tau)
    k4 = quad_dynamics(x + dt * k3, u, gravity, rate_tau)
    out = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    q = out[..., 6:10]
    out[..., 6:10] = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return out


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


# ---------------------------------------------------------------------------
# Solver
# ---------------------------------------------------------------------------


@dataclass
class NMPCSolution:
    u0: np.ndarray  # (4,) [c, wx, wy, wz] to apply now
    inputs: np.ndarray  # (N, 4)
    states: np.ndarray  # (N + 1, 13), states[0] is the current state
    cost: float
    iterations: int
    converged: bool
    metadata: dict = field(default_factory=dict)


class QuadrotorNMPC:
    """iLQR on the thrust/body-rate model with box-bounded inputs."""

    def __init__(
        self,
        *,
        dt: float = 0.05,
        horizon: int = 20,
        q_pos: float = 10.0,
        q_vel: float = 1.0,
        q_tilt: float = 2.0,
        q_yaw: float = 2.0,
        r_thrust: float = 0.05,
        r_rate: float = 0.3,
        r_yaw_rate: float = 1.0,
        q_rate: float = 0.05,
        rate_tau: float = 1.0 / 15.0,
        terminal_scale: float = 10.0,
        max_tilt_deg: float | None = 45.0,
        tilt_penalty: float = 2.0e4,
        thrust_min: float = 0.5,
        thrust_max: float = 20.0,
        rate_max_xy: float = 4.0,
        rate_max_z: float = 1.5,
        gravity: float = G,
        max_iter: int = 8,
        tol: float = 1e-4,
    ) -> None:
        if dt <= 0.0 or horizon < 2:
            raise ValueError("dt must be > 0 and horizon >= 2")
        if not 0.0 <= thrust_min < gravity < thrust_max:
            raise ValueError("need 0 <= thrust_min < gravity < thrust_max, or it cannot hover")
        if rate_max_xy <= 0.0 or rate_max_z <= 0.0:
            raise ValueError("rate limits must be > 0")
        self.dt, self.horizon, self.gravity = float(dt), int(horizon), float(gravity)
        self.u_lo = np.array([thrust_min, -rate_max_xy, -rate_max_xy, -rate_max_z])
        self.u_hi = np.array([thrust_max, rate_max_xy, rate_max_xy, rate_max_z])
        self.u_hover = np.array([gravity, 0.0, 0.0, 0.0])
        # Quaternion weights: (qx, qy) carry the tilt - qx^2 + qy^2 is
        # sin^2(tilt/2) for a yaw-only reference - and (qw, qz) the heading.
        if rate_tau <= 0.0:
            raise ValueError("rate_tau must be > 0")
        self.rate_tau = float(rate_tau)
        self.Q = np.diag([q_pos] * 3 + [q_vel] * 3 + [q_yaw, q_tilt, q_tilt, q_yaw] + [q_rate] * 3)
        self.Qf = terminal_scale * self.Q
        self.R = np.diag([r_thrust, r_rate, r_rate, r_yaw_rate])
        self.max_iter, self.tol = int(max_iter), float(tol)
        # Soft tilt limit: tilt > max_tilt_deg costs tilt_penalty * excess^2,
        # with excess measured in s = qx^2 + qy^2 = sin^2(tilt / 2). Quadratic
        # weights alone cannot hold it: a few metres of position error outweigh
        # any sensible tilt weight, and the first plans tilted past 90 degrees.
        if max_tilt_deg is not None and not 0.0 < max_tilt_deg < 180.0:
            raise ValueError("max_tilt_deg must be in (0, 180) or None")
        self.s_max = None if max_tilt_deg is None else float(np.sin(np.radians(max_tilt_deg) / 2.0) ** 2)
        self.tilt_penalty = float(tilt_penalty)
        self._U: np.ndarray | None = None

    def reset(self) -> None:
        self._U = None

    # -- pieces --------------------------------------------------------------
    def rollout(self, x0: np.ndarray, U: np.ndarray) -> np.ndarray:
        X = np.empty((self.horizon + 1, NX))
        X[0] = x0
        for k in range(self.horizon):
            X[k + 1] = rk4_step(X[k], U[k], self.dt, self.gravity, self.rate_tau)
        return X

    def _align(self, X: np.ndarray, Xref: np.ndarray) -> np.ndarray:
        """Flip reference quaternions into the plan's hemisphere (q and -q agree)."""
        Xr = Xref.copy()
        flip = np.sum(X[:, 6:10] * Xr[:, 6:10], axis=1) < 0.0
        Xr[flip, 6:10] *= -1.0
        return Xr

    def cost(self, X: np.ndarray, U: np.ndarray, Xref: np.ndarray) -> float:
        Xr = self._align(X, Xref)
        dx = X - Xr
        du = U - self.u_hover
        stage = np.einsum("ki,ij,kj->", dx[:-1], self.Q, dx[:-1]) + np.einsum("ki,ij,kj->", du, self.R, du)
        total = float(stage + dx[-1] @ self.Qf @ dx[-1])
        if self.s_max is not None:
            excess = np.maximum(X[:, 7] ** 2 + X[:, 8] ** 2 - self.s_max, 0.0)
            total += self.tilt_penalty * float(np.sum(excess ** 2))
        return total

    def _tilt_terms(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Gradient and Gauss-Newton Hessian of the tilt penalty at one state."""
        g = np.zeros(NX)
        H = np.zeros((NX, NX))
        if self.s_max is None:
            return g, H
        excess = x[7] ** 2 + x[8] ** 2 - self.s_max
        if excess <= 0.0:
            return g, H
        ds = np.zeros(NX)
        ds[7], ds[8] = 2.0 * x[7], 2.0 * x[8]
        g = 2.0 * self.tilt_penalty * excess * ds
        H = 2.0 * self.tilt_penalty * np.outer(ds, ds)
        return g, H

    def linearize(self, X: np.ndarray, U: np.ndarray, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
        """``A_k, B_k`` for every step at once: one batched RK4 call of size N x (NX + NU + 1)."""
        N = self.horizon
        xs = np.repeat(X[:-1, None, :], NX + NU + 1, axis=1)
        us = np.repeat(U[:, None, :], NX + NU + 1, axis=1)
        idx = np.arange(NX)
        xs[:, 1 + idx, idx] += eps
        idu = np.arange(NU)
        us[:, 1 + NX + idu, idu] += eps
        out = rk4_step(xs.reshape(-1, NX), us.reshape(-1, NU), self.dt, self.gravity, self.rate_tau).reshape(N, -1, NX)
        diff = (out[:, 1:, :] - out[:, :1, :]) / eps  # (N, NX + NU, NX)
        A = np.transpose(diff[:, :NX, :], (0, 2, 1))
        B = np.transpose(diff[:, NX:, :], (0, 2, 1))
        return A, B

    # -- iLQR ------------------------------------------------------------------
    def solve(self, x0: np.ndarray, Xref: np.ndarray) -> NMPCSolution:
        x0 = np.asarray(x0, dtype=float).reshape(NX)
        if not np.all(np.isfinite(x0)):
            raise ValueError("NMPC received a non-finite state")
        N = self.horizon
        Xref = np.asarray(Xref, dtype=float).reshape(N + 1, NX)

        if self._U is None:
            U = np.tile(self.u_hover, (N, 1))
        else:
            U = np.vstack([self._U[1:], self._U[-1:]])
        U = np.clip(U, self.u_lo, self.u_hi)
        X = self.rollout(x0, U)
        J = self.cost(X, U, Xref)
        mu = 1e-6
        converged = False
        it = 0
        for it in range(1, self.max_iter + 1):
            A, B = self.linearize(X, U)
            Xr = self._align(X, Xref)
            k_ff, K_fb = self._backward(X, U, Xr, A, B, mu)
            if k_ff is None:
                mu = max(mu * 10.0, 1e-4)
                continue
            accepted = False
            for alpha in (1.0, 0.5, 0.25, 0.1, 0.03):
                Xn = np.empty_like(X)
                Un = np.empty_like(U)
                Xn[0] = x0
                for k in range(N):
                    du = alpha * k_ff[k] + K_fb[k] @ (Xn[k] - X[k])
                    Un[k] = np.clip(U[k] + du, self.u_lo, self.u_hi)
                    Xn[k + 1] = rk4_step(Xn[k], Un[k], self.dt, self.gravity, self.rate_tau)
                Jn = self.cost(Xn, Un, Xref)
                if Jn < J:
                    accepted = True
                    break
            if not accepted:
                mu = max(mu * 10.0, 1e-4)
                if mu > 1e6:
                    break
                continue
            rel = (J - Jn) / max(abs(J), 1e-9)
            X, U, J = Xn, Un, Jn
            mu = max(mu / 10.0, 1e-8)
            if rel < self.tol:
                converged = True
                break

        self._U = U.copy()
        return NMPCSolution(
            u0=U[0].copy(), inputs=U.copy(), states=X.copy(), cost=J,
            iterations=it, converged=converged, metadata={"mu": mu},
        )

    def _backward(self, X, U, Xr, A, B, mu):
        N = self.horizon
        Q, R, Qf = self.Q, self.R, self.Qf
        gN, HN = self._tilt_terms(X[-1])
        Vx = 2.0 * Qf @ (X[-1] - Xr[-1]) + gN
        Vxx = 2.0 * Qf + HN
        k_ff = np.zeros((N, NU))
        K_fb = np.zeros((N, NU, NX))
        for k in range(N - 1, -1, -1):
            gk, Hk = self._tilt_terms(X[k])
            lx = 2.0 * Q @ (X[k] - Xr[k]) + gk
            lu = 2.0 * R @ (U[k] - self.u_hover)
            Ak, Bk = A[k], B[k]
            Qx = lx + Ak.T @ Vx
            Qu = lu + Bk.T @ Vx
            Qxx = 2.0 * Q + Hk + Ak.T @ Vxx @ Ak
            Vreg = Vxx + mu * np.eye(NX)
            Quu = 2.0 * R + Bk.T @ Vreg @ Bk
            Qux = Bk.T @ Vreg @ Ak
            Quu = 0.5 * (Quu + Quu.T)
            try:
                kk, KK = _clamped_newton(Quu, Qu, Qux, U[k], self.u_lo, self.u_hi)
            except np.linalg.LinAlgError:
                return None, None
            k_ff[k], K_fb[k] = kk, KK
            Vx = Qx + KK.T @ Quu @ kk + KK.T @ Qu + Qux.T @ kk
            Vxx = Qxx + KK.T @ Quu @ KK + KK.T @ Qux + Qux.T @ KK
            Vxx = 0.5 * (Vxx + Vxx.T)
        return k_ff, K_fb


def _clamped_newton(Quu, Qu, Qux, u, lo, hi):
    """Feedforward and gain with inputs that would leave their box held at it.

    One active-set pass: solve unconstrained, clamp what leaves the box, then
    re-solve for the free inputs with the clamped ones fixed. Clamped inputs
    get no feedback gain - the forward pass cannot move them anyway.
    """
    np.linalg.cholesky(Quu)  # raises if not positive definite
    k = -np.linalg.solve(Quu, Qu)
    clamped = (u + k < lo) | (u + k > hi)
    if not clamped.any():
        return k, -np.linalg.solve(Quu, Qux)
    free = ~clamped
    k_c = np.clip(u + k, lo, hi) - u
    kk = np.zeros(NU)
    KK = np.zeros((NU, NX))
    kk[clamped] = k_c[clamped]
    if free.any():
        Qff = Quu[np.ix_(free, free)]
        rhs = Qu[free] + Quu[np.ix_(free, clamped)] @ k_c[clamped]
        kk[free] = -np.linalg.solve(Qff, rhs)
        KK[free] = -np.linalg.solve(Qff, Qux[free])
    return kk, KK


# ---------------------------------------------------------------------------
# Controller: plan + rate loop -> wrench
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class NMPCGains:
    mass_kg: float
    inertia_kgm2: tuple[float, float, float] = (0.01, 0.01, 0.02)
    k_rate: float = 15.0  # rate-loop bandwidth, 1/s; under the motors' corner
    k_att: float = 6.0  # attitude feedback onto the plan's attitude, 1/s
    dt: float = 0.05
    horizon: int = 20
    q_pos: float = 10.0
    q_vel: float = 1.0
    q_tilt: float = 2.0
    q_yaw: float = 2.0
    r_thrust: float = 0.05
    r_rate: float = 0.3
    max_tilt_deg: float | None = 45.0
    thrust_max: float | None = None  # m/s^2; default: 2 g
    rate_max_xy: float = 4.0
    rate_max_z: float = 1.5
    max_iter: int = 8

    def __post_init__(self) -> None:
        if self.mass_kg <= 0.0:
            raise ValueError(f"mass_kg must be > 0, got {self.mass_kg}")
        if self.k_rate <= 0.0:
            raise ValueError("k_rate must be > 0")

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "NMPCGains":
        sub = dict((cfg.get("controller", {}) or {}).get("nmpc", {}) or {})
        if "mass_kg" not in sub:
            raise ValueError("the nmpc controller needs `controller.nmpc.mass_kg` and will not guess it")
        unknown = sorted(set(sub) - set(cls.__dataclass_fields__))
        if unknown:
            raise ValueError(f"unknown nmpc setting(s) {unknown}")
        if "inertia_kgm2" in sub:
            sub["inertia_kgm2"] = tuple(float(i) for i in sub["inertia_kgm2"])
        return cls(**sub)


class NMPCController(Controller):
    """Re-plans every ``gains.dt`` and runs the rate loop on every call.

    ``reference`` is an optional ``t -> FlatReference``; without it the target
    waypoint is held at rest, at the current heading.
    """

    def __init__(self, gains: NMPCGains | None = None, reference: Callable[[float], FlatReference] | None = None) -> None:
        self.gains = gains
        self.reference = reference
        self.planner: QuadrotorNMPC | None = None
        self.last_solution: NMPCSolution | None = None
        self._u = None
        self._t_plan = -np.inf
        self._target = None

    def _build(self, g: NMPCGains) -> QuadrotorNMPC:
        return QuadrotorNMPC(
            dt=g.dt, horizon=g.horizon, q_pos=g.q_pos, q_vel=g.q_vel, q_tilt=g.q_tilt,
            q_yaw=g.q_yaw, r_thrust=g.r_thrust, r_rate=g.r_rate, max_tilt_deg=g.max_tilt_deg,
            thrust_max=2.0 * G if g.thrust_max is None else g.thrust_max,
            rate_tau=1.0 / g.k_rate,
            rate_max_xy=g.rate_max_xy, rate_max_z=g.rate_max_z, max_iter=g.max_iter,
        )

    def reference_horizon(self, t: float, target: np.ndarray, yaw: float) -> np.ndarray:
        planner = self.planner
        Xref = np.zeros((planner.horizon + 1, NX))
        for k in range(planner.horizon + 1):
            if self.reference is not None:
                r = self.reference(t + k * planner.dt)
                Xref[k, 0:3] = r.position
                Xref[k, 3:6] = r.velocity
                Xref[k, 6:10] = yaw_quat(yaw if r.yaw is None else r.yaw)
            else:
                Xref[k, 0:3] = target
                Xref[k, 6:10] = yaw_quat(yaw)
        return Xref

    def compute(self, state: SimState, target_waypoint: Waypoint, cfg: Mapping[str, Any]) -> ControlTarget:
        g = self.gains if self.gains is not None else NMPCGains.from_config(cfg)
        if self.planner is None:
            self.planner = self._build(g)
        if state.attitude_quat is None or state.body_rates is None:
            raise ValueError("NMPC needs attitude_quat and body_rates")
        q = np.asarray(state.attitude_quat, dtype=float).reshape(4)
        q = q / np.linalg.norm(q)
        omega = np.asarray(state.body_rates, dtype=float).reshape(3)
        R = quat_to_rotmat(q)
        t = float(state.t)
        target = np.asarray(target_waypoint.position, dtype=float).reshape(3)

        stale = (
            self._u is None
            or t - self._t_plan >= self.planner.dt - 1e-9
            or t < self._t_plan
            or (self.reference is None and not np.allclose(target, self._target))
        )
        if stale:
            x0 = np.concatenate([state.position, state.velocity, q, omega])
            Xref = self.reference_horizon(t, target, yaw_of(R))
            self.last_solution = self.planner.solve(x0, Xref)
            self._u = self.last_solution.u0.copy()
            self._t_plan, self._target = t, target.copy()

        # Track the plan's attitude, not only its rates. The plan assumes a body
        # rate takes effect at once; the real rate loop and motors lag it, so a
        # rate-only command lets the attitude drift off the plan between
        # re-solves - the first version tilted to 74 degrees against a planned
        # 45. The plan's attitude at this instant (normalised linear
        # interpolation between its first two states) is fed back as a rate.
        c, omega_ff = float(self._u[0]), self._u[1:]
        sol = self.last_solution
        frac = float(np.clip((t - self._t_plan) / self.planner.dt, 0.0, 1.0))
        q_a, q_b = sol.states[0, 6:10], sol.states[1, 6:10]
        if np.dot(q_a, q_b) < 0.0:
            q_b = -q_b
        q_ref = (1.0 - frac) * q_a + frac * q_b
        R_ref = quat_to_rotmat(q_ref / np.linalg.norm(q_ref))
        omega_des = omega_ff - g.k_att * attitude_error(R_ref, R)
        J = np.diag(np.asarray(g.inertia_kgm2, dtype=float))
        moment = J @ (g.k_rate * (omega_des - omega)) + np.cross(omega, J @ omega)
        thrust = g.mass_kg * c
        accel_cmd = c * (R @ np.array([0.0, 0.0, 1.0])) - np.array([0.0, 0.0, G])
        return ControlTarget(
            accel_cmd=accel_cmd,
            wrench=Wrench(force_body=np.array([0.0, 0.0, thrust]), moment_body=moment),
            metadata={"controller": "nmpc", "replanned": stale, "thrust_n": thrust,
                      "iterations": self.last_solution.iterations if self.last_solution else 0},
        )


__all__ = [
    "NMPCController", "NMPCGains", "NMPCSolution", "QuadrotorNMPC",
    "quad_dynamics", "rk4_step", "yaw_quat",
]
