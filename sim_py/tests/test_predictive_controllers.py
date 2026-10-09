"""Constrained MPC, warm-started MPPI, the QP under the MPC and the path reference.

The closed-loop tests fly a point mass (``p' = v, v' = a``) integrated at a
finer step than the controller's, holding each command for one controller step
- the same zero-order hold the simulator wrappers apply.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from aerial_kit.controllers import (
    BoxQP,
    ConstrainedMPC,
    ConstrainedMPCController,
    MPPI,
    PathReference,
    SphereObstacle,
    WarmMPPIController,
    constant_reference,
)
from aerial_kit.controllers.mppi import rollout
from aerial_kit.controllers.position import mpc_position_control
from aerial_kit.types import SimState, Waypoint

REPO_ROOT = Path(__file__).resolve().parents[2]


def fly(controller, reference_fn, *, p0=(0.0, 0.0, 0.0), v0=(0.0, 0.0, 0.0), duration=8.0, sim_dt=0.01):
    """Point-mass closed loop. ``reference_fn(p)`` returns a reference or a target."""
    p = np.array(p0, dtype=float)
    v = np.array(v0, dtype=float)
    hold = max(int(round(controller.dt / sim_dt)), 1)
    steps = int(round(duration / sim_dt))
    positions, velocities, accels = [], [], []
    a = np.zeros(3)
    for i in range(steps):
        if i % hold == 0:
            a = controller.solve(p, v, reference_fn(p)).accel
        v = v + a * sim_dt
        p = p + v * sim_dt
        positions.append(p.copy())
        velocities.append(v.copy())
        accels.append(a.copy())
    return np.array(positions), np.array(velocities), np.array(accels)


def distance_to_polyline(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    out = np.full(len(points), np.inf)
    for a, b in zip(polyline[:-1], polyline[1:]):
        d = b - a
        t = np.clip(((points - a) @ d) / (d @ d), 0.0, 1.0)
        out = np.minimum(out, np.linalg.norm(a + t[:, None] * d - points, axis=1))
    return out


# ---------------------------------------------------------------------------
# BoxQP
# ---------------------------------------------------------------------------


def test_qp_without_active_constraints_is_the_linear_solve():
    rng = np.random.default_rng(3)
    M = rng.normal(size=(6, 6))
    H = M @ M.T + 6.0 * np.eye(6)
    q = rng.normal(size=6)
    qp = BoxQP(H, np.eye(6), eps_abs=1e-10, eps_rel=1e-10, max_iter=5000)
    res = qp.solve(q, -1e6 * np.ones(6), 1e6 * np.ones(6))
    assert res.converged
    np.testing.assert_allclose(res.x, np.linalg.solve(H, -q), atol=1e-6)


def test_qp_box_solution_matches_the_projection_for_a_separable_problem():
    # min 0.5|x|^2 - [2, -3, 0.5].x  s.t. -1 <= x <= 1  ->  x = clip([2, -3, 0.5])
    qp = BoxQP(np.eye(3), np.eye(3), eps_abs=1e-9, eps_rel=1e-9, max_iter=5000)
    res = qp.solve(np.array([-2.0, 3.0, -0.5]), -np.ones(3), np.ones(3))
    assert res.converged
    np.testing.assert_allclose(res.x, [1.0, -1.0, 0.5], atol=1e-6)


def test_qp_batch_equals_solving_each_column_alone():
    H = np.array([[4.0, 1.0], [1.0, 3.0]])
    C = np.vstack([np.eye(2), [[1.0, 1.0]]])
    qs = np.array([[1.0, -4.0, 2.0], [-2.0, 1.0, 5.0]])
    lo = np.array([[-1.0, -1.0, -0.2], [-1.0, -1.0, -0.2], [-0.5, -0.5, -0.5]])
    hi = -lo
    qp = BoxQP(H, C, eps_abs=1e-9, eps_rel=1e-9, max_iter=5000)
    batch = qp.solve(qs, lo, hi).x
    for j in range(3):
        alone = qp.solve(qs[:, j], lo[:, j], hi[:, j]).x
        np.testing.assert_allclose(batch[:, j], alone, atol=1e-6)


def test_qp_rejects_crossed_bounds_and_bad_shapes():
    qp = BoxQP(np.eye(2), np.eye(2))
    with pytest.raises(ValueError, match="lower bound"):
        qp.solve(np.zeros(2), np.ones(2), np.zeros(2))
    with pytest.raises(ValueError):
        BoxQP(np.eye(2), np.eye(3))
    with pytest.raises(ValueError):
        BoxQP(np.ones((2, 3)), np.eye(3))


# ---------------------------------------------------------------------------
# ConstrainedMPC
# ---------------------------------------------------------------------------


def test_mpc_with_inactive_limits_is_the_finite_horizon_lq_it_replaces():
    """Same model, same weights, stage-cost terminal, limits out of reach: the
    QP's first move must be the Riccati gain's, to solver tolerance."""
    mpc = ConstrainedMPC(
        dt=0.1, horizon=10, q_pos=8.0, q_vel=2.0, r_acc=10.0, terminal="stage",
        max_accel_xy=1e3, max_accel_z=1e3, tol=1e-10, max_iter=10000,
    )
    p, v = np.array([1.0, -2.0, 0.5]), np.array([0.3, 0.0, -0.1])
    ours = mpc.solve(p, v, np.zeros(3))
    assert ours.converged
    theirs = mpc_position_control(p, v, np.zeros(3), 8.0, 2.0, 10.0, dt=0.1, horizon=10)
    np.testing.assert_allclose(ours.accel, theirs, atol=1e-6)


def test_mpc_never_plans_outside_its_acceleration_limits():
    mpc = ConstrainedMPC(max_accel_xy=3.0, max_accel_z=1.5)
    sol = mpc.solve(np.array([50.0, -40.0, -20.0]), np.zeros(3), np.zeros(3))
    assert np.all(np.abs(sol.accel_sequence[:, :2]) <= 3.0 + 1e-9)
    assert np.all(np.abs(sol.accel_sequence[:, 2]) <= 1.5 + 1e-9)
    # A far target saturates the first move, in the right direction.
    np.testing.assert_allclose(sol.accel, [-3.0, 3.0, 1.5], atol=1e-3)


def test_mpc_plan_and_flight_respect_the_speed_limit():
    mpc = ConstrainedMPC(max_speed_xy=2.0, max_speed_z=1.0, max_accel_xy=5.0)
    sol = mpc.solve(np.array([30.0, 0.0, 10.0]), np.zeros(3), np.zeros(3))
    assert np.all(np.abs(sol.predicted_velocities[:, 0]) <= 2.0 + 1e-3)
    assert np.all(np.abs(sol.predicted_velocities[:, 2]) <= 1.0 + 1e-3)

    mpc.reset()
    _, vel, _ = fly(mpc, lambda p: np.array([20.0, 0.0, 6.0]), duration=6.0)
    assert np.max(np.abs(vel[:, 0])) <= 2.0 + 0.02
    assert np.max(np.abs(vel[:, 2])) <= 1.0 + 0.02


def test_mpc_starting_above_the_speed_limit_brakes_instead_of_failing():
    """The velocity bound relaxes to what 90 % braking can reach, so the QP is
    feasible from any state. With the target far ahead the plan rides that
    relaxed bound: brake at 90 % of the limit until back under the speed limit."""
    mpc = ConstrainedMPC(max_speed_xy=2.0, max_accel_xy=4.0)
    sol = mpc.solve(np.zeros(3), np.array([6.0, 0.0, 0.0]), np.array([100.0, 0.0, 0.0]))
    assert sol.converged
    assert sol.accel[0] == pytest.approx(-0.9 * 4.0, abs=1e-3)
    assert sol.predicted_velocities[-1, 0] == pytest.approx(2.0, abs=1e-3)


def test_mpc_closed_loop_settles_on_the_target():
    target = np.array([6.0, -3.0, 2.0])
    pos, vel, _ = fly(ConstrainedMPC(), lambda p: target, duration=10.0)
    assert np.linalg.norm(pos[-1] - target) < 0.02
    assert np.linalg.norm(vel[-1]) < 0.02


def test_mpc_tracks_a_path_reference_through_a_corner():
    path_pts = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 2.0], [10.0, 10.0, 2.0]])
    path = PathReference(path_pts, decel_mps2=1.5)
    mpc = ConstrainedMPC(horizon=20, max_speed_xy=3.5)
    pos, _, _ = fly(
        mpc,
        lambda p: path.sample(path.project(p), cruise_mps=3.0, dt=mpc.dt, horizon=mpc.horizon),
        duration=14.0,
    )
    assert np.max(distance_to_polyline(pos, path_pts)) < 0.6
    assert np.linalg.norm(pos[-1] - path_pts[-1]) < 0.05


def test_mpc_delta_penalty_smooths_the_command():
    target = np.array([8.0, 0.0, 0.0])
    _, _, rough = fly(ConstrainedMPC(r_delta=0.0), lambda p: target, duration=5.0)
    _, _, smooth = fly(ConstrainedMPC(r_delta=100.0), lambda p: target, duration=5.0)
    # Sum of squared step-to-step changes, one sample per controller step.
    jerk = lambda a: float(np.sum(np.diff(a[::10, 0]) ** 2))
    assert jerk(smooth) < 0.5 * jerk(rough)


def test_mpc_warm_start_saves_iterations_on_the_next_solve():
    cold = ConstrainedMPC(max_speed_xy=2.0, warm_start=False)
    warm = ConstrainedMPC(max_speed_xy=2.0, warm_start=True)
    p, v, target = np.array([10.0, 4.0, 0.0]), np.zeros(3), np.zeros(3)
    for ctrl in (cold, warm):
        ctrl.solve(p, v, target)
    assert warm.solve(p, v, target).iterations < cold.solve(p, v, target).iterations


@pytest.mark.parametrize(
    "kwargs",
    [{"r_acc": 0.0}, {"q_pos": -1.0}, {"max_accel_xy": 0.0}, {"max_speed_z": -1.0}, {"terminal": "nope"}],
)
def test_mpc_rejects_bad_settings(kwargs):
    with pytest.raises(ValueError):
        ConstrainedMPC(**kwargs)


def test_mpc_rejects_a_bad_state_or_reference():
    mpc = ConstrainedMPC(horizon=5)
    with pytest.raises(ValueError, match="non-finite"):
        mpc.solve(np.array([np.nan, 0.0, 0.0]), np.zeros(3), np.zeros(3))
    with pytest.raises(ValueError, match="5 steps"):
        mpc.solve(np.zeros(3), np.zeros(3), constant_reference(np.zeros(3), 4))


# ---------------------------------------------------------------------------
# MPPI
# ---------------------------------------------------------------------------


def test_rollout_is_the_exact_double_integrator():
    rng = np.random.default_rng(0)
    accels = rng.normal(size=(4, 7, 3))
    p0, v0, dt = np.array([1.0, 2.0, 3.0]), np.array([0.5, -0.5, 0.0]), 0.1
    pos, vel = rollout(p0, v0, accels, dt)
    for k in range(4):
        p, v = p0.copy(), v0.copy()
        for n in range(7):
            p = p + v * dt + 0.5 * accels[k, n] * dt * dt
            v = v + accels[k, n] * dt
            np.testing.assert_allclose(pos[k, n], p, atol=1e-12)
            np.testing.assert_allclose(vel[k, n], v, atol=1e-12)


def test_mppi_is_reproducible_with_a_seed():
    a = MPPI(seed=11).solve(np.zeros(3), np.zeros(3), np.array([3.0, 1.0, 0.0]))
    b = MPPI(seed=11).solve(np.zeros(3), np.zeros(3), np.array([3.0, 1.0, 0.0]))
    np.testing.assert_array_equal(a.accel_sequence, b.accel_sequence)


def test_mppi_keeps_and_shifts_its_nominal_plan():
    mppi = MPPI(seed=0, horizon=10)
    sol = mppi.solve(np.zeros(3), np.zeros(3), np.array([5.0, 0.0, 0.0]))
    np.testing.assert_allclose(mppi.nominal[:-1], sol.accel_sequence[1:])
    np.testing.assert_allclose(mppi.nominal[-1], sol.accel_sequence[-1])
    assert 1.0 <= sol.effective_samples <= mppi.samples + 2
    mppi.reset()
    assert not np.any(mppi.nominal)


def test_mppi_respects_its_acceleration_limits():
    mppi = MPPI(seed=1, max_accel_xy=2.0, max_accel_z=1.0, noise_std=5.0)
    sol = mppi.solve(np.zeros(3), np.zeros(3), np.array([100.0, -100.0, 50.0]))
    assert np.all(np.abs(sol.accel_sequence[:, :2]) <= 2.0 + 1e-9)
    assert np.all(np.abs(sol.accel_sequence[:, 2]) <= 1.0 + 1e-9)


def test_mppi_closed_loop_reaches_the_target():
    target = np.array([6.0, -3.0, 2.0])
    pos, vel, _ = fly(MPPI(seed=0), lambda p: target, duration=10.0)
    assert np.linalg.norm(pos[-1] - target) < 0.3
    assert np.linalg.norm(vel[-1]) < 0.3


def test_mppi_flies_around_an_obstacle_on_the_straight_line():
    target = np.array([10.0, 0.0, 2.0])
    obstacle = SphereObstacle(center=(5.0, 0.0, 2.0), radius=1.5)
    mppi = MPPI(seed=0, obstacles=[obstacle], obstacle_margin=0.5)
    pos, _, _ = fly(mppi, lambda p: target, duration=12.0)
    clearance = np.linalg.norm(pos - np.array(obstacle.center), axis=1)
    assert clearance.min() > obstacle.radius  # never inside the sphere
    assert np.linalg.norm(pos[-1] - target) < 0.4


def test_mppi_obstacles_can_be_passed_per_solve():
    obstacle = SphereObstacle(center=(5.0, 0.0, 2.0), radius=1.5)
    mppi = MPPI(seed=0)
    pos, _, _ = fly(
        type("PerCall", (), {"dt": mppi.dt, "solve": lambda self, p, v, r: mppi.solve(p, v, r, obstacles=[obstacle])})(),
        lambda p: np.array([10.0, 0.0, 2.0]),
        duration=12.0,
    )
    assert np.linalg.norm(pos - np.array(obstacle.center), axis=1).min() > obstacle.radius


def test_mppi_minimum_altitude_keeps_it_off_the_floor():
    mppi = MPPI(seed=2, min_altitude=1.0)
    pos, _, _ = fly(mppi, lambda p: np.array([5.0, 0.0, -2.0]), p0=(0.0, 0.0, 3.0), duration=8.0)
    assert pos[:, 2].min() > 0.8


def test_mppi_rejects_bad_settings():
    with pytest.raises(ValueError):
        MPPI(temperature=0.0)
    with pytest.raises(ValueError):
        MPPI(noise_std=-1.0)
    with pytest.raises(ValueError):
        MPPI(max_accel_z=0.0)
    with pytest.raises(ValueError, match="non-finite"):
        MPPI().solve(np.array([np.inf, 0.0, 0.0]), np.zeros(3), np.zeros(3))


# ---------------------------------------------------------------------------
# PathReference
# ---------------------------------------------------------------------------


L_PATH = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]])


def test_path_projection_finds_the_closest_arc_length():
    path = PathReference(L_PATH)
    assert path.length == pytest.approx(20.0)
    assert path.project(np.array([4.0, 1.0, 0.0])) == pytest.approx(4.0)
    assert path.project(np.array([11.0, 6.0, 0.0])) == pytest.approx(16.0)
    assert path.project(np.array([-5.0, 0.0, 0.0])) == pytest.approx(0.0)
    assert path.project(np.array([10.0, 50.0, 0.0])) == pytest.approx(20.0)


def test_path_projection_honours_its_search_window():
    # A path that doubles back: without s_min the projection would jump back.
    path = PathReference(np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 0.2, 0.0]]))
    p = np.array([3.0, -0.1, 0.0])  # nearer the outbound leg
    assert path.project(p) < 10.0
    assert path.project(p, s_min=10.0) > 10.0


def test_path_sample_stays_on_the_path_and_stops_at_the_end():
    path = PathReference(L_PATH, decel_mps2=2.0)
    ref = path.sample(0.0, cruise_mps=3.0, dt=0.1, horizon=30)
    assert ref.positions.shape == ref.velocities.shape == (30, 3)
    assert np.max(distance_to_polyline(ref.positions, L_PATH)) < 1e-9
    assert np.max(np.linalg.norm(ref.velocities, axis=1)) <= 3.0 + 1e-9

    end = path.sample(19.9, cruise_mps=3.0, dt=0.1, horizon=30)
    np.testing.assert_allclose(end.positions[-1], L_PATH[-1], atol=1e-6)
    assert np.linalg.norm(end.velocities[-1]) < 1e-6
    speeds = np.linalg.norm(end.velocities, axis=1)
    assert np.all(np.diff(speeds) <= 1e-9)  # decelerating into the end


def test_path_with_one_point_or_repeats_is_well_defined():
    single = PathReference(np.array([[1.0, 2.0, 3.0]]))
    ref = single.sample(0.0, cruise_mps=2.0, dt=0.1, horizon=4)
    np.testing.assert_allclose(ref.positions, np.tile([1.0, 2.0, 3.0], (4, 1)))
    assert not np.any(ref.velocities)

    repeated = PathReference(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [5.0, 0.0, 0.0]]))
    assert repeated.points.shape == (2, 3)
    with pytest.raises(ValueError):
        PathReference(np.zeros((0, 3)))


# ---------------------------------------------------------------------------
# Simulator wrappers
# ---------------------------------------------------------------------------


def test_new_controllers_are_registered():
    from sim_py.core.registry import create_controller, register_builtin_components

    register_builtin_components()
    assert isinstance(create_controller("constrained_mpc"), ConstrainedMPCController)
    assert isinstance(create_controller("warm_mppi"), WarmMPPIController)


@pytest.mark.parametrize("cls", [ConstrainedMPCController, WarmMPPIController])
def test_wrapper_holds_its_command_between_controller_steps(cls):
    ctrl = cls()
    cfg = {"controller": {cls.name: {"dt": 0.1, "horizon": 8}}}
    wp = Waypoint(position=np.array([5.0, 0.0, 1.0]))

    def state(t, x=0.0):
        return SimState(position=np.array([x, 0.0, 0.0]), velocity=np.zeros(3), t=t)

    first = ctrl.compute(state(0.0), wp, cfg)
    held = ctrl.compute(state(0.05, x=0.1), wp, cfg)
    assert first.metadata["replanned"] and not held.metadata["replanned"]
    np.testing.assert_array_equal(first.accel_cmd, held.accel_cmd)

    assert ctrl.compute(state(0.1, x=0.2), wp, cfg).metadata["replanned"]  # one dt later
    moved = Waypoint(position=np.array([-5.0, 0.0, 1.0]))
    assert ctrl.compute(state(0.12, x=0.2), moved, cfg).metadata["replanned"]  # new target


def test_mppi_wrapper_reads_obstacles_from_config():
    ctrl = WarmMPPIController()
    cfg = {"controller": {"warm_mppi": {"seed": 0, "obstacles": [{"center": [1, 2, 3], "radius": 0.5}]}}}
    ctrl.compute(SimState(position=np.zeros(3), velocity=np.zeros(3)), Waypoint(position=np.ones(3)), cfg)
    assert ctrl._planner.obstacles == [SphereObstacle((1.0, 2.0, 3.0), 0.5)]


@pytest.mark.parametrize("name", ["constrained_mpc", "warm_mppi"])
def test_quadrotor_example_reaches_goal_with_predictive_controllers(name):
    from aerial_kit.sim import load_config, run_simulation

    config = load_config(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    result = run_simulation(replace(config, controller_name=name))
    assert result.distance_to_goal <= 3.0
    assert result.collisions_detected == 0
