"""A small box-constrained QP solver (ADMM, OSQP-style), numpy/scipy only.

Solves, for one or several right-hand sides at once,

    minimise    0.5 x' H x + q' x
    subject to  l <= C x <= u

``H`` and ``C`` are fixed when the solver is built, so the one linear system
ADMM needs is factorised once and every solve after that is a pair of
triangular solves per iteration. That is the shape a linear MPC has: the
Hessian and the constraint matrix depend on the model and the horizon, and only
the linear term and the bounds move with the state.

Several problems sharing ``H`` and ``C`` are solved together by passing ``q``,
``l`` and ``u`` with one column per problem. The per-axis MPC uses that to solve
x, y and z in one call - the three axes of a double integrator are the same
problem with different numbers.

Each row of ``C`` is scaled to unit norm before factorising (the bounds are
scaled with it and the duals unscaled on the way out). Without that, a
constraint set that mixes rows of very different size - the MPC's acceleration
rows are ones, its velocity rows are ``dt`` - converges an order of magnitude
slower for any single ``rho``.

It is deliberately not a general solver: there is no equality handling beyond
``l == u`` rows, no adaptive step and no infeasibility certificate. An
infeasible problem simply runs to ``max_iter`` and reports ``converged=False``,
and the caller is expected to keep its problems feasible (the MPC does, by
construction of its velocity bounds).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import cho_factor, cho_solve


@dataclass
class QPResult:
    """Solution of one batched solve. Arrays carry one column per problem."""

    x: np.ndarray
    z: np.ndarray
    y: np.ndarray
    iterations: int
    converged: bool
    primal_residual: float
    dual_residual: float


class BoxQP:
    """ADMM solver for ``min 0.5 x'Hx + q'x  s.t.  l <= Cx <= u``."""

    def __init__(
        self,
        H: np.ndarray,
        C: np.ndarray,
        *,
        rho: float = 1.0,
        sigma: float = 1e-6,
        alpha: float = 1.6,
        max_iter: int = 400,
        eps_abs: float = 1e-5,
        eps_rel: float = 1e-5,
    ) -> None:
        H = np.asarray(H, dtype=float)
        C = np.asarray(C, dtype=float)
        if H.ndim != 2 or H.shape[0] != H.shape[1]:
            raise ValueError("H must be square")
        if C.ndim != 2 or C.shape[1] != H.shape[0]:
            raise ValueError("C must have as many columns as H")
        if rho <= 0.0 or sigma <= 0.0:
            raise ValueError("rho and sigma must be positive")
        if not 0.0 < alpha < 2.0:
            raise ValueError("alpha must be in (0, 2)")

        self.H = 0.5 * (H + H.T)
        norms = np.linalg.norm(C, axis=1)
        if np.any(norms <= 0.0):
            raise ValueError("C has an all-zero row")
        self._row_scale = (1.0 / norms).reshape(-1, 1)
        self.C = C * self._row_scale  # equilibrated; bounds are scaled to match
        self.n = H.shape[0]
        self.m = C.shape[0]
        self.rho = float(rho)
        self.sigma = float(sigma)
        self.alpha = float(alpha)
        self.max_iter = int(max_iter)
        self.eps_abs = float(eps_abs)
        self.eps_rel = float(eps_rel)

        kkt = self.H + self.sigma * np.eye(self.n) + self.rho * (self.C.T @ self.C)
        self._factor = cho_factor(kkt)

    def solve(
        self,
        q: np.ndarray,
        l: np.ndarray,
        u: np.ndarray,
        *,
        x0: np.ndarray | None = None,
        y0: np.ndarray | None = None,
    ) -> QPResult:
        """Solve one problem (1-D arrays) or a batch (one column per problem)."""
        q = np.asarray(q, dtype=float)
        single = q.ndim == 1
        q2 = q.reshape(self.n, -1)
        batch = q2.shape[1]
        l2 = np.broadcast_to(np.asarray(l, dtype=float).reshape(self.m, -1), (self.m, batch))
        u2 = np.broadcast_to(np.asarray(u, dtype=float).reshape(self.m, -1), (self.m, batch))
        if np.any(l2 > u2):
            raise ValueError("lower bound above upper bound")
        l2 = l2 * self._row_scale
        u2 = u2 * self._row_scale

        H, C, rho, sigma, alpha = self.H, self.C, self.rho, self.sigma, self.alpha
        x = np.zeros((self.n, batch)) if x0 is None else np.array(x0, dtype=float).reshape(self.n, batch)
        if y0 is None:
            y = np.zeros((self.m, batch))
        else:
            y = np.array(y0, dtype=float).reshape(self.m, batch) / self._row_scale
        z = np.clip(C @ x, l2, u2)

        converged = False
        r_prim = r_dual = np.inf
        it = 0
        for it in range(1, self.max_iter + 1):
            rhs = sigma * x - q2 + C.T @ (rho * z - y)
            x_tilde = cho_solve(self._factor, rhs)
            z_tilde = C @ x_tilde
            x = alpha * x_tilde + (1.0 - alpha) * x
            z_relaxed = alpha * z_tilde + (1.0 - alpha) * z
            z_new = np.clip(z_relaxed + y / rho, l2, u2)
            y = y + rho * (z_relaxed - z_new)
            z = z_new

            if it % 5 == 0 or it == self.max_iter:
                Cx = C @ x
                Hx = H @ x
                CTy = C.T @ y
                r_prim = float(np.max(np.abs(Cx - z))) if self.m else 0.0
                r_dual = float(np.max(np.abs(Hx + q2 + CTy)))
                scale_p = max(float(np.max(np.abs(Cx))) if self.m else 0.0, float(np.max(np.abs(z))) if self.m else 0.0)
                scale_d = max(float(np.max(np.abs(Hx))), float(np.max(np.abs(q2))), float(np.max(np.abs(CTy))) if self.m else 0.0)
                if (
                    r_prim <= self.eps_abs + self.eps_rel * scale_p
                    and r_dual <= self.eps_abs + self.eps_rel * scale_d
                ):
                    converged = True
                    break

        # Back to the caller's scaling: z = C x in original rows, y its multiplier.
        z = z / self._row_scale
        y = y * self._row_scale
        if single:
            return QPResult(x[:, 0], z[:, 0], y[:, 0], it, converged, r_prim, r_dual)
        return QPResult(x, z, y, it, converged, r_prim, r_dual)


__all__ = ["BoxQP", "QPResult"]
