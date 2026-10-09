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
together by :class:`~.qp.BoxQP` as a batch of three. That is also why the
velocity limit in xy is a box (``|vx|, |vy| <= max_speed_xy``) and not a disc:
a norm bound couples the axes and makes it a different problem. The box is a
conservative-in-one-direction approximation and is documented as such.

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

        C = [np.eye(N)]
        if self.speed_limit is not None:
            C.append(self._Gamma[1::2])  # velocity rows
        self._C = np.vstack(C)
        # ADMM's step wants to be on the scale of the Hessian; 0.6 x its mean
        # diagonal is a measured sweet spot across the default weights.
        rho = 0.6 * float(np.mean(np.diag(H))) if rho is None else float(rho)
        self._qp = BoxQP(H, self._C, rho=rho, max_iter=max_iter, eps_abs=tol, eps_rel=tol)

        self._last_u: np.ndarray | None = None  # (N, 3)
        self._last_y: np.ndarray | None = None
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
        lower = [np.repeat(-a_lim, N, axis=0)]
        upper = [np.repeat(a_lim, N, axis=0)]
        if self.speed_limit is not None:
            steps = np.arange(1, N + 1, dtype=float).reshape(-1, 1) * self.dt
            v_free = free[1::2]
            # Bounds the plan can actually meet: if the aircraft is already
            # faster than the limit, the bound relaxes to what 90 % braking can
            # reach by step k. Full braking would also be feasible, but it would
            # leave exactly one feasible plan, and ADMM crawls on a feasible set
            # that has no interior.
            reach_hi = v0.reshape(1, 3) + 0.9 * a_lim * steps
            reach_lo = v0.reshape(1, 3) - 0.9 * a_lim * steps
            v_hi = np.maximum(self.speed_limit.reshape(1, 3), reach_lo)
            v_lo = np.minimum(-self.speed_limit.reshape(1, 3), reach_hi)
            lower.append(v_lo - v_free)
            upper.append(v_hi - v_free)
        lower = np.vstack(lower)
        upper = np.vstack(upper)

        x_init = y_init = None
        if self.warm_start and self._last_u is not None:
            x_init = np.vstack([self._last_u[1:], self._last_u[-1:]])
            y_init = self._last_y

        res = self._qp.solve(q, lower, upper, x0=x_init, y0=y_init)
        U = np.clip(res.x, -a_lim, a_lim)  # ADMM is feasible only to tolerance
        X = free + self._Gamma @ U
        self._last_u = U.copy()
        self._last_y = res.y.copy()
        self._last_applied = U[0].copy()

        return MPCSolution(
            accel=U[0].copy(),
            accel_sequence=U.copy(),
            predicted_positions=X[0::2].copy(),
            predicted_velocities=X[1::2].copy(),
            iterations=res.iterations,
            converged=res.converged,
            metadata={"primal_residual": res.primal_residual, "dual_residual": res.dual_residual},
        )


__all__ = ["ConstrainedMPC", "MPCSolution", "double_integrator", "prediction_matrices"]
