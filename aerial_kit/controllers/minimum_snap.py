"""Small minimum-snap polynomial trajectory utilities.

The fit is done in a Chebyshev basis rather than a monomial one. That is not
cosmetic: a degree-seven monomial Vandermonde over ``[0, 1]`` reaches a
condition number near ``1e17`` when the waypoint times sit close together, and
the fit then returns a trajectory whose error *at the waypoints* is of order
``1e12`` - it does not interpolate them at all, while still returning finite
numbers. The Chebyshev basis stays well conditioned on the same nodes, so the
interpolation guarantee below survives clustered times instead of quietly
collapsing.
"""

from __future__ import annotations

import numpy as np


def _cheb_basis(s: np.ndarray, n_coeff: int) -> np.ndarray:
    """Chebyshev basis matrix, ``basis[q, k] = T_k(s[q])`` for ``k < n_coeff``."""
    basis = np.empty((s.shape[0], n_coeff), dtype=float)
    basis[:, 0] = 1.0
    if n_coeff > 1:
        basis[:, 1] = s
    for k in range(2, n_coeff):
        basis[:, k] = 2.0 * s * basis[:, k - 1] - basis[:, k - 2]
    return basis


def _chebyshev_to_monomial(n_coeff: int) -> np.ndarray:
    """Matrix ``T`` with ``T[k, j]`` the coefficient of ``s**j`` in ``T_k(s)``."""
    monomial = np.zeros((n_coeff, n_coeff), dtype=float)
    monomial[0, 0] = 1.0
    if n_coeff > 1:
        monomial[1, 1] = 1.0
    for k in range(2, n_coeff):
        monomial[k, :] = 2.0 * np.concatenate([[0.0], monomial[k - 1, :-1]]) - monomial[k - 2, :]
    return monomial


def _cheb_derivative_operator(n_coeff: int) -> np.ndarray:
    """Matrix mapping Chebyshev coefficients of ``p`` to those of ``dp/ds``.

    Built as a monomial round trip - ``m = T^T a`` takes Chebyshev coefficients
    to monomial ones, ``d/ds`` there is a shift with a weight, and ``T^-T`` takes
    them back - rather than from the coefficient recurrence, which is easy to
    write down with the wrong weight at the ``T_0`` term. The result is checked
    against the closed form ``dT_k/ds = k U_{k-1}`` in the tests.
    """
    monomial = _chebyshev_to_monomial(n_coeff)
    derivative = np.zeros((n_coeff, n_coeff), dtype=float)
    for j in range(1, n_coeff):
        derivative[j - 1, j] = float(j)
    return np.linalg.inv(monomial.T) @ derivative @ monomial.T


def _snap_gram(n_coeff: int) -> np.ndarray:
    """Gram matrix of ``d^4/dtau^4`` for Chebyshev coefficients on tau in [0, 1].

    Four applications of the derivative operator give the fourth derivative in
    ``s``; ``s = 2 tau - 1`` contributes ``2**4``; and Gauss-Legendre integrates
    the squared result exactly at the node count used here, since the integrand
    is a polynomial of degree ``2 (n_coeff - 5)``. The quadrature is in ``s``,
    so the change of variable contributes ``d tau = ds / 2``.
    """
    fourth = np.linalg.matrix_power(_cheb_derivative_operator(n_coeff), 4)

    n_nodes = max(2 * (n_coeff + 1), 32)
    nodes, weights = np.polynomial.legendre.leggauss(n_nodes)
    basis = _cheb_basis(nodes, n_coeff)
    quadrature = basis.T @ (weights[:, None] * basis)

    scale = 2.0 ** 4
    return 0.5 * (scale * scale) * (fourth.T @ quadrature @ fourth)


def minimum_snap_trajectory(
    waypoints: np.ndarray,
    times: np.ndarray | None = None,
    polynomial_order: int = 7,
    snap_weight: float = 1e-4,
    num_samples: int = 100,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit a minimum-snap polynomial that passes **exactly** through the waypoints.

    The waypoints are hard constraints, not a fit target: the returned samples
    interpolate every one of them, for any number of waypoints from two upward,
    including when the times are close together. Among the polynomials of the
    requested order that do so, the one with the least integrated squared snap
    (the fourth derivative) is selected. Where a direction of the remaining
    freedom leaves the snap cost unchanged - which happens when there are few
    waypoints and a high order - ``snap_weight`` regularises it, so the system
    has a unique finite solution instead of being singular.

    Time is normalised to ``[times[0], times[-1]]`` before fitting. Returns
    ``(positions, velocities, accelerations)``, each shaped
    ``(num_samples, 3)``, with derivatives taken with respect to the supplied
    time unit.
    """
    waypoints = np.asarray(waypoints, dtype=float).reshape(-1, 3)
    n_waypoints = len(waypoints)
    if n_waypoints < 2:
        raise ValueError("at least two waypoints are required")

    if times is None:
        times = np.linspace(0.0, 1.0, n_waypoints)
    times = np.asarray(times, dtype=float).reshape(-1)
    if times.shape != (n_waypoints,):
        raise ValueError("times must match the number of waypoints")
    if not np.all(np.diff(times) > 0.0):
        raise ValueError("times must be strictly increasing")

    t0, t1 = float(times[0]), float(times[-1])
    span = t1 - t0

    order = max(int(polynomial_order), n_waypoints - 1)
    n_coeff = order + 1

    tau = (times - t0) / span
    constraint = _cheb_basis(2.0 * tau - 1.0, n_coeff)

    # One tolerance for both the rank count and the solve. Letting ``lstsq``
    # pick its own more aggressive cutoff while counting the rank by hand
    # produces a "solution" that misses the waypoints, which is precisely the
    # failure this function exists to avoid.
    singular = np.linalg.svd(constraint, compute_uv=False)
    tolerance = singular[0] * max(n_waypoints, n_coeff) * np.finfo(float).eps
    rank = int(np.count_nonzero(singular > tolerance))
    if rank < n_waypoints:
        raise ValueError(
            f"waypoint times are too close together to fit a polynomial of order "
            f"{order}: the constraint matrix has rank {rank} but {n_waypoints} "
            f"waypoints must be interpolated"
        )

    # Every interpolant is ``particular + null_basis @ y``: the particular
    # solution meets the waypoints, and the null basis spans the directions that
    # keep meeting them, so the constraints hold exactly for any ``y`` while the
    # snap cost is minimised over what is left.
    particular = np.linalg.lstsq(constraint, waypoints, rcond=tolerance / singular[0])[0]
    null_basis = np.linalg.svd(constraint)[2][rank:].T

    if null_basis.shape[1] == 0:
        coeffs = particular
    else:
        gram = _snap_gram(n_coeff)
        reduced = null_basis.T @ gram @ null_basis
        reduced += float(snap_weight) * np.eye(null_basis.shape[1])
        target = -(null_basis.T @ gram @ particular)
        coeffs = particular + null_basis @ np.linalg.solve(reduced, target)

    sample_tau = (np.linspace(t0, t1, max(int(num_samples), 2)) - t0) / span
    basis = _cheb_basis(2.0 * sample_tau - 1.0, n_coeff)
    operator = _cheb_derivative_operator(n_coeff)

    pos = basis @ coeffs
    vel = 2.0 * (basis @ (operator @ coeffs)) / span
    acc = 4.0 * (basis @ (operator @ (operator @ coeffs))) / (span * span)
    return pos, vel, acc


__all__ = ["minimum_snap_trajectory"]
