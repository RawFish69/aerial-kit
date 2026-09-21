from __future__ import annotations

import numpy as np
import pytest

from aerial_kit.controllers.basic import LQRController, MPCController, MPPIController, PIDController
from aerial_kit.controllers.minimum_snap import minimum_snap_trajectory
from aerial_kit.types import SimState, Waypoint
from sim_py.core.registry import create_controller, register_builtin_components


def _state(pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)) -> SimState:
    return SimState(
        position=np.array(pos, dtype=float),
        velocity=np.array(vel, dtype=float),
        t=0.0,
    )


def _waypoint(pos=(1.0, 0.0, 0.0)) -> Waypoint:
    return Waypoint(position=np.array(pos, dtype=float))


def test_all_position_controllers_return_finite_acceleration():
    state = _state()
    target = _waypoint()
    controllers = [
        PIDController(),
        LQRController(),
        MPCController(),
        MPPIController(),
    ]
    for controller in controllers:
        result = controller.compute(state, target, cfg={})
        assert np.all(np.isfinite(result.accel_cmd))


def test_builtin_controllers_resolve():
    register_builtin_components()
    for name in ("pid", "lqr", "mpc", "mppi", "l1_tecs"):
        assert create_controller(name) is not None


WAYPOINTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 1.0],
        [2.0, 0.0, 2.0],
        [3.0, 1.0, 0.0],
    ],
    dtype=float,
)


def test_minimum_snap_waypoint_continuity():
    times = np.array([0.0, 1.0, 2.0, 3.0])
    pos, vel, acc = minimum_snap_trajectory(WAYPOINTS, times, num_samples=40)
    assert pos.shape == (40, 3)
    assert vel.shape == (40, 3)
    assert acc.shape == (40, 3)
    assert np.all(np.isfinite(pos))
    assert np.all(np.isfinite(vel))
    assert np.all(np.isfinite(acc))


def test_minimum_snap_interpolates_every_waypoint():
    """The claim in the name: the trajectory passes *through* the waypoints.

    Two things about the inputs are deliberate. The sample times are chosen so
    that the waypoint times are themselves samples, because a nearest-sample
    lookup otherwise measures sampling error instead of interpolation error. And
    the span is varied, because the failure this replaces was span-dependent: at
    a span of five the old regulariser was swamped by the Vandermonde entries and
    the fit interpolated to 1e-17, while at the unit span - the function's own
    default - five waypoints were off by 1.5e-2 and six by 7.6e-1.
    """
    tail = np.array([[4.0, 0.0, 1.0], [5.0, -1.0, 0.0]])
    for span in (1.0, 5.0):
        for count in range(2, 7):
            waypoints = np.vstack([WAYPOINTS, tail])[:count]
            num_samples = (count - 1) * 200 + 1
            grid = np.linspace(0.0, span, num_samples)
            # Taken from the grid itself, not rebuilt with linspace: the same
            # value computed two ways is not bit-identical, and this assertion
            # has to be about interpolation, not about float rounding.
            idx = [i * 200 for i in range(count)]
            times = grid[idx]
            pos, _, _ = minimum_snap_trajectory(waypoints, times, num_samples=num_samples)
            assert np.abs(pos[idx] - waypoints).max() < 1e-5, (span, count)


def test_minimum_snap_two_waypoints_is_not_singular():
    """Two waypoints used to raise LinAlgError: 2 constraints and 4 regularised
    coefficients cannot determine 8 unknowns."""
    pos, vel, acc = minimum_snap_trajectory(
        np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 0.5]]), num_samples=50
    )
    assert np.all(np.isfinite(pos)) and np.all(np.isfinite(vel)) and np.all(np.isfinite(acc))
    assert np.allclose(pos[0], [0.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(pos[-1], [1.0, 2.0, 0.5], atol=1e-9)


def test_minimum_snap_clustered_times_still_interpolate():
    """Clustered times are where a monomial basis collapses: at a unit span this
    input is beyond what the previous fit could represent at all."""
    num_samples = 801
    grid = np.linspace(0.0, 1.0, num_samples)
    # Clustered to within three grid steps of each other, and taken *from* the
    # sample grid so the measurement is interpolation error, not sampling error.
    times = grid[[0, 400, 401, 402, 800]]
    waypoints = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [4.0, 0.0, 0.0]]
    )
    pos, _, _ = minimum_snap_trajectory(waypoints, times, num_samples=num_samples)
    idx = [0, 400, 401, 402, 800]
    assert np.array_equal(grid[idx], times)
    assert np.abs(pos[idx] - waypoints).max() < 1e-5


def test_minimum_snap_derivatives_are_the_derivatives_of_position():
    times = np.array([0.0, 1.0, 2.0, 3.0])
    n = 4001
    pos, vel, acc = minimum_snap_trajectory(WAYPOINTS, times, num_samples=n)
    grid = np.linspace(0.0, 3.0, n)
    h = grid[1] - grid[0]
    # Interior only: np.gradient is one-sided at the ends. Its error is O(h^2),
    # so a tight tolerance here would be testing np.gradient, not this code.
    assert np.abs(vel - np.gradient(pos, h, axis=0))[1:-1].max() < 1e-5
    assert np.abs(acc - np.gradient(vel, h, axis=0))[1:-1].max() < 1e-5


def test_minimum_snap_refuses_degenerate_input():
    with pytest.raises(ValueError, match="at least two waypoints"):
        minimum_snap_trajectory(np.array([[0.0, 0.0, 0.0]]))
    with pytest.raises(ValueError, match="strictly increasing"):
        minimum_snap_trajectory(WAYPOINTS, times=np.array([0.0, 1.0, 1.0, 3.0]))
    with pytest.raises(ValueError, match="times must match"):
        minimum_snap_trajectory(WAYPOINTS, times=np.array([0.0, 1.0]))


def test_minimum_snap_minimises_snap():
    """The other half of the name. Fewer waypoints than coefficients leaves the
    fit underdetermined, and the snap cost is what picks between the answers."""
    from aerial_kit.controllers.minimum_snap import (
        _cheb_basis,
        _cheb_derivative_operator,
        _snap_gram,
    )

    waypoints = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 1.0], [0.0, 0.0, 0.0]])
    times = np.array([0.0, 1.0, 2.0])
    order = 7
    n_coeff = order + 1

    tau = (times - times[0]) / (times[-1] - times[0])
    constraint = _cheb_basis(2.0 * tau - 1.0, n_coeff)
    singular = np.linalg.svd(constraint, compute_uv=False)
    tolerance = singular[0] * max(len(times), n_coeff) * np.finfo(float).eps
    rank = int(np.count_nonzero(singular > tolerance))
    null_basis = np.linalg.svd(constraint)[2][rank:].T
    particular = np.linalg.lstsq(constraint, waypoints, rcond=tolerance / singular[0])[0]
    assert null_basis.shape[1] > 0, "this test needs an underdetermined fit"

    gram = _snap_gram(n_coeff)

    def cost(coeffs: np.ndarray) -> float:
        # One coefficient column per position axis; the total cost is the trace.
        return float(np.trace(coeffs.T @ gram @ coeffs))

    ours = particular + null_basis @ np.linalg.solve(
        null_basis.T @ gram @ null_basis + 1e-4 * np.eye(null_basis.shape[1]),
        -(null_basis.T @ gram @ particular),
    )
    # Every perturbation of the null space interpolates the same waypoints, so
    # any of them is a fair competitor on cost.
    rng = np.random.default_rng(0)
    for _ in range(500):
        other = particular + null_basis @ (rng.normal(size=null_basis.shape[1]) * 10.0)[:, None]
        assert cost(ours) <= cost(other) + 1e-9


def test_chebyshev_derivative_operator_matches_closed_form():
    """The operator is built by a monomial round trip; this is the independent
    check that the orientation and the weights are right, against dT_k/ds = k U_{k-1}."""
    from aerial_kit.controllers.minimum_snap import _cheb_basis, _cheb_derivative_operator

    n_coeff = 9
    points = np.array([0.13, -0.77, 0.41, 0.95, -0.31])

    def cheb_u(n: int, s: np.ndarray) -> np.ndarray:
        if n == 0:
            return np.ones_like(s)
        if n == 1:
            return 2.0 * s
        previous, current = np.ones_like(s), 2.0 * s
        for _ in range(2, n + 1):
            previous, current = current, 2.0 * s * current - previous
        return current

    analytic = np.stack(
        [(k * cheb_u(k - 1, points) if k else np.zeros_like(points)) for k in range(n_coeff)],
        axis=1,
    )
    got = _cheb_basis(points, n_coeff) @ _cheb_derivative_operator(n_coeff)
    assert np.abs(got - analytic).max() < 1e-10

