"""Constrained linear MPC for a point-mass (double-integrator) model.

``mpc_position_control`` in :mod:`.position` is an unconstrained receding
horizon LQ: a gain, computed by a Riccati recursion, and a ``clip`` afterwards.
Clipping the output of an unconstrained optimiser is not the same as optimising
under the constraint - the clipped command is not the best one that respects
the limit, and the plan behind it assumed accelerations it will never get.

:class:`ConstrainedMPC` puts the limits inside the optimisation:

* acceleration bounds per axis (``max_accel_xy``, ``max_accel_z``),
* optional velocity bounds per axis (``max_speed_xy``, ``max_speed_z``),
* a tracking cost against a *horizon reference* (positions and velocities for
  every step, see :mod:`.reference`) rather than a single target point,
* an optional penalty on the change in acceleration between steps
  (``r_delta``), which is what smooths the command a real airframe receives.

The model is per-axis and decoupled: ``p' = v, v' = a`` on each of x, y, z. The
three axes therefore share one Hessian and one constraint matrix, and are solved
together by :class:`~.qp.BoxQP` as a batch of three. In that default
(``speed_limit_xy="box"``) the xy speed limit is a box, ``|vx|, |vy| <=
max_speed_xy``, so a diagonal can reach ``sqrt(2)`` times the limit.

``speed_limit_xy="disc"`` bounds ``|v_xy|`` itself, by a regular polygon
(``disc_sides``, default 8) inscribed in the circle of radius
``max_speed_xy``: inside the polygon is inside the circle, and the polygon
gives away at most ``1 - cos(pi / sides)`` (8 % for an octagon) along its
flats. A norm bound couples x and y, so in this mode they are solved as one
2N-variable QP and z on its own.

Units and frame are this repository's: metres, seconds, world frame, z up. The
acceleration is the *demand* the point-mass and multirotor backends integrate,
so gravity is not part of the model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_discrete_are

from .qp import BoxQP
from .reference import HorizonReference, constant_reference


@dataclass
class MPCSolution:
    """Result of one MPC solve."""

    accel: np.ndarray  # (3,) first acceleration of the plan - the one to apply
    accel_sequence: np.ndarray  # (N, 3)
    predicted_positions: np.ndarray  # (N, 3), steps 1..N
    predicted_velocities: np.ndarray  # (N, 3), steps 1..N
    iterations: int
    converged: bool
    metadata: dict = field(default_factory=dict)


def double_integrator(dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact zero-order-hold discretisation of ``p' = v, v' = a``."""
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt * dt], [dt]])
    return A, B


def prediction_matrices(A: np.ndarray, B: np.ndarray, horizon: int) -> tuple[np.ndarray, np.ndarray]:
    """``X = Phi x0 + Gamma U`` with ``X = [x_1; ...; x_N]``."""
    nx, nu = A.shape[0], B.shape[1]
    Phi = np.zeros((nx * horizon, nx))
    Gamma = np.zeros((nx * horizon, nu * horizon))
    Ak = np.eye(nx)
    powers = []
    for k in range(horizon):
        powers.append(Ak)  # A^k
        Ak = A @ Ak
        Phi[k * nx:(k + 1) * nx] = Ak  # A^(k+1)
    for k in range(horizon):
        for j in range(k + 1):
            Gamma[k * nx:(k + 1) * nx, j * nu:(j + 1) * nu] = powers[k - j] @ B
    return Phi, Gamma


class ConstrainedMPC:
    """Box-constrained tracking MPC on a 3-axis double integrator."""

    def __init__(
        self,
        *,
        dt: float = 0.1,
        horizon: int = 20,
        q_pos: float = 8.0,
        q_vel: float = 1.0,
        r_acc: float = 0.5,
        r_delta: float = 0.0,
        terminal: str = "dare",
        max_accel_xy: float = 6.0,
        max_accel_z: float = 4.0,
        max_speed_xy: float | None = None,
        max_speed_z: float | None = None,
        speed_limit_xy: str = "box",
        disc_sides: int = 8,
        rho: float | None = None,
        max_iter: int = 400,
        tol: float = 1e-5,
        warm_start: bool = True,
    ) -> None:
        self.dt = max(float(dt), 1e-4)
        self.horizon = max(int(horizon), 1)
        if q_pos < 0.0 or q_vel < 0.0 or r_delta < 0.0:
            raise ValueError("weights must be non-negative")
        if r_acc <= 0.0:
            raise ValueError("r_acc must be positive")
        if max_accel_xy <= 0.0 or max_accel_z <= 0.0:
            raise ValueError("acceleration limits must be positive")
        for name, value in (("max_speed_xy", max_speed_xy), ("max_speed_z", max_speed_z)):
            if value is not None and value <= 0.0:
                raise ValueError(f"{name} must be positive or None")
        terminal = str(terminal).lower()
        if terminal not in {"dare", "stage"}:
            raise ValueError("terminal must be 'dare' or 'stage'")
        speed_limit_xy = str(speed_limit_xy).lower()
        if speed_limit_xy not in {"box", "disc"}:
            raise ValueError("speed_limit_xy must be 'box' or 'disc'")
        if speed_limit_xy == "disc" and int(disc_sides) < 4:
            raise ValueError("disc_sides must be at least 4")

        self.q_pos, self.q_vel = float(q_pos), float(q_vel)
        self.r_acc, self.r_delta = float(r_acc), float(r_delta)
        self.terminal = terminal
        self.accel_limit = np.array([max_accel_xy, max_accel_xy, max_accel_z], dtype=float)
        self.speed_limit = (
            None
            if max_speed_xy is None and max_speed_z is None
            else np.array(
                [
                    np.inf if max_speed_xy is None else max_speed_xy,
                    np.inf if max_speed_xy is None else max_speed_xy,
                    np.inf if max_speed_z is None else max_speed_z,
                ],
                dtype=float,
            )
        )
        self.warm_start = bool(warm_start)

        N = self.horizon
        A, B = double_integrator(self.dt)
        Q = np.diag([self.q_pos, self.q_vel])
        Qf = solve_discrete_are(A, B, Q, np.array([[self.r_acc]])) if terminal == "dare" else Q
        self._Phi, self._Gamma = prediction_matrices(A, B, N)
        Qbar = np.kron(np.eye(N), Q)
        Qbar[-2:, -2:] = Qf
        self._Qbar = Qbar

        # First-difference operator: (D U)_0 = u_0, (D U)_k = u_k - u_{k-1}.
        D = np.eye(N) - np.eye(N, k=-1)
        self._D = D
        H = 2.0 * (self._Gamma.T @ Qbar @ self._Gamma + self.r_acc * np.eye(N) + self.r_delta * D.T @ D)
        self._H = H

        # ADMM's step wants to be on the scale of the Hessian; 0.6 x its mean
        # diagonal is a measured sweet spot across the default weights.
        rho = 0.6 * float(np.mean(np.diag(H))) if rho is None else float(rho)
        qp_kw = dict(rho=rho, max_iter=max_iter, eps_abs=tol, eps_rel=tol)
        Gv = self._Gamma[1::2]  # velocity rows: v_k = v_free_k + Gv[k] U
        self.disc = speed_limit_xy == "disc" and max_speed_xy is not None
        if not self.disc:
            C = [np.eye(N)]
            if self.speed_limit is not None:
                C.append(Gv)
            self._C = np.vstack(C)
            self._qp = BoxQP(H, self._C, **qp_kw)
        else:
            sides = int(disc_sides)
            theta = 2.0 * np.pi * np.arange(sides) / sides
            self._normals = np.stack([np.cos(theta), np.sin(theta)], axis=1)  # (sides, 2)
            self._apothem = float(max_speed_xy) * float(np.cos(np.pi / sides))
            # Rows ordered side-major: row j*N + k is n_j . v_k <= apothem.
            poly = np.vstack([np.hstack([c * Gv, s_ * Gv]) for c, s_ in self._normals])
            self._qp_xy = BoxQP(np.kron(np.eye(2), H), np.vstack([np.eye(2 * N), poly]), **qp_kw)
            Cz = [np.eye(N)] + ([Gv] if max_speed_z is not None else [])
            self._qp_z = BoxQP(H, np.vstack(Cz), **qp_kw)

        self._last_u: np.ndarray | None = None  # (N, 3)
        self._last_y = None  # duals; a tuple (xy, z) in disc mode
        self._last_applied = np.zeros(3)

    def reset(self) -> None:
        """Forget the warm start and the previously applied command."""
        self._last_u = None
        self._last_y = None
        self._last_applied = np.zeros(3)

    def solve(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        reference: HorizonReference | np.ndarray,
    ) -> MPCSolution:
        """Plan from ``(position, velocity)`` against ``reference``.

        ``reference`` is a :class:`HorizonReference`, or a single 3-vector
        target, which is treated as "be there and at rest" for every step.
        """
        p0 = np.asarray(position, dtype=float).reshape(3)
        v0 = np.asarray(velocity, dtype=float).reshape(3)
        if not (np.all(np.isfinite(p0)) and np.all(np.isfinite(v0))):
            raise ValueError("MPC received a non-finite state")
        N = self.horizon
        if not isinstance(reference, HorizonReference):
            reference = constant_reference(np.asarray(reference, dtype=float), N)
        ref_p = np.asarray(reference.positions, dtype=float).reshape(-1, 3)
        ref_v = np.asarray(reference.velocities, dtype=float).reshape(-1, 3)
        if ref_p.shape[0] != N or ref_v.shape[0] != N:
            raise ValueError(f"reference must have {N} steps, got {ref_p.shape[0]}")

        # Stack per-axis: x0 is (2, 3), r is (2N, 3) interleaved [p1, v1, p2, v2, ...].
        x0 = np.vstack([p0, v0])
        r = np.empty((2 * N, 3))
        r[0::2] = ref_p
        r[1::2] = ref_v
        free = self._Phi @ x0  # (2N, 3)
        q = 2.0 * self._Gamma.T @ self._Qbar @ (free - r)
        if self.r_delta > 0.0:
            e0 = np.zeros((N, 1))
            e0[0, 0] = 1.0
            q -= 2.0 * self.r_delta * (self._D.T @ e0) @ self._last_applied.reshape(1, 3)

        a_lim = self.accel_limit.reshape(1, 3)
        steps = np.arange(1, N + 1, dtype=float).reshape(-1, 1) * self.dt
        v_free = free[1::2]  # (N, 3)

        x_init = None
        if self.warm_start and self._last_u is not None:
            x_init = np.vstack([self._last_u[1:], self._last_u[-1:]])
        y_init = self._last_y if self.warm_start else None

        if not self.disc:
            lower = [np.repeat(-a_lim, N, axis=0)]
            upper = [np.repeat(a_lim, N, axis=0)]
            if self.speed_limit is not None:
                v_lo, v_hi = self._reachable_box(v0, a_lim, steps, self.speed_limit.reshape(1, 3))
                lower.append(v_lo - v_free)
                upper.append(v_hi - v_free)
            res = self._qp.solve(q, np.vstack(lower), np.vstack(upper), x0=x_init, y0=y_init)
            U_raw, y_new = res.x, res.y
            iterations, converged = res.iterations, res.converged
            residuals = (res.primal_residual, res.dual_residual)
        else:
            U_raw, y_new, iterations, converged, residuals = self._solve_disc(
                q, v0, v_free, a_lim, steps, x_init, y_init
            )

        U = np.clip(U_raw, -a_lim, a_lim)  # ADMM is feasible only to tolerance
        X = free + self._Gamma @ U
        self._last_u = U.copy()
        self._last_y = y_new
        self._last_applied = U[0].copy()

        return MPCSolution(
            accel=U[0].copy(),
            accel_sequence=U.copy(),
            predicted_positions=X[0::2].copy(),
            predicted_velocities=X[1::2].copy(),
            iterations=iterations,
            converged=converged,
            metadata={"primal_residual": residuals[0], "dual_residual": residuals[1]},
        )

    @staticmethod
    def _reachable_box(v0, a_lim, steps, limit):
        """Per-axis velocity bounds the plan can actually meet.

        If the aircraft is already faster than the limit, the bound relaxes to
        what 90 % braking can reach by step k. Full braking would also be
        feasible, but it would leave exactly one feasible plan, and ADMM crawls
        on a feasible set that has no interior.
        """
        reach_hi = v0.reshape(1, -1) + 0.9 * a_lim * steps
        reach_lo = v0.reshape(1, -1) - 0.9 * a_lim * steps
        return np.minimum(-limit, reach_hi), np.maximum(limit, reach_lo)

    def _solve_disc(self, q, v0, v_free, a_lim, steps, x_init, y_init):
        """xy as one QP under the polygon speed bound, z on its own."""
        N = self.horizon
        y_xy, y_z = y_init if isinstance(y_init, tuple) else (None, None)

        # xy: [Ux; Uy] with accel boxes, then n_j . v_k <= apothem (relaxed,
        # as in the box case, to what 90 % braking along n_j reaches).
        n = self._normals
        along_free = v_free[:, :2] @ n.T  # (N, sides): n_j . v_free_k
        along_now = n @ v0[:2]  # (sides,)
        bound = np.maximum(self._apothem, along_now[None, :] - 0.9 * a_lim[0, 0] * steps)
        poly_hi = (bound - along_free).T.reshape(-1)  # side-major, matching the rows
        a_xy = np.full(2 * N, a_lim[0, 0])
        lo_xy = np.concatenate([-a_xy, np.full(poly_hi.size, -np.inf)])
        hi_xy = np.concatenate([a_xy, poly_hi])
        x0_xy = None if x_init is None else np.concatenate([x_init[:, 0], x_init[:, 1]])
        rxy = self._qp_xy.solve(
            np.concatenate([q[:, 0], q[:, 1]]), lo_xy, hi_xy, x0=x0_xy, y0=y_xy
        )

        lo_z = [np.full(N, -a_lim[0, 2])]
        hi_z = [np.full(N, a_lim[0, 2])]
        if np.isfinite(self.speed_limit[2]):
            v_lo, v_hi = self._reachable_box(
                v0[2:], a_lim[:, 2:], steps, self.speed_limit[2:].reshape(1, 1)
            )
            lo_z.append((v_lo - v_free[:, 2:]).reshape(-1))
            hi_z.append((v_hi - v_free[:, 2:]).reshape(-1))
        rz = self._qp_z.solve(
            q[:, 2], np.concatenate(lo_z), np.concatenate(hi_z),
            x0=None if x_init is None else x_init[:, 2], y0=y_z,
        )

        U = np.stack([rxy.x[:N], rxy.x[N:], rz.x], axis=1)
        return (
            U,
            (rxy.y, rz.y),
            max(rxy.iterations, rz.iterations),
            rxy.converged and rz.converged,
            (max(rxy.primal_residual, rz.primal_residual), max(rxy.dual_residual, rz.dual_residual)),
        )


__all__ = ["ConstrainedMPC", "MPCSolution", "double_integrator", "prediction_matrices"]
