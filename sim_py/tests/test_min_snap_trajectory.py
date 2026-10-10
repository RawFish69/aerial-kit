"""Piecewise minimum-snap trajectories, and flying one on the actuator plant."""

from __future__ import annotations

import numpy as np
import pytest

from aerial_kit.trajectory import Limits, MinSnapTrajectory, path_trapezoid_durations, thin_waypoints

ZIGZAG = np.array([[0.0, 0, 2], [5, 0, 2], [5, 5, 3], [0, 5, 2], [0, 10, 4]])
STRAIGHT = np.array([[0.0, 0, 2] + np.array([88.8, 74.4, 21.0]) * k / 10 for k in range(11)])


@pytest.fixture(scope="module")
def zigzag():
    return MinSnapTrajectory(ZIGZAG, limits=Limits(2.0, 2.0, 5.0))


def test_it_hits_every_waypoint_and_starts_and_ends_at_rest(zigzag):
    for t, w in zip(zigzag.times, zigzag.waypoints):
        np.testing.assert_allclose(zigzag.evaluate(t), w, atol=1e-9)
    for r in (1, 2, 3):
        np.testing.assert_allclose(zigzag.evaluate(zigzag.times[0], r), 0.0, atol=1e-9)
        np.testing.assert_allclose(zigzag.evaluate(zigzag.times[-1], r), 0.0, atol=1e-9)


def test_velocity_through_snap_are_continuous_at_the_waypoints(zigzag):
    for t in zigzag.times[1:-1]:
        for r in range(1, 5):
            np.testing.assert_allclose(zigzag.evaluate(t - 1e-9, r), zigzag.evaluate(t + 1e-9, r), atol=1e-5)


def test_derivatives_are_the_derivatives_of_position(zigzag):
    h = 1e-5
    for t in np.linspace(0.3, zigzag.duration - 0.3, 9):
        for r in range(4):
            fd = (zigzag.evaluate(t + h, r) - zigzag.evaluate(t - h, r)) / (2 * h)
            np.testing.assert_allclose(fd, zigzag.evaluate(t, r + 1), atol=1e-5)


def test_the_limits_hold_and_the_tightest_one_binds(zigzag):
    v, a, j = zigzag.max_derivatives(samples_per_segment=200)
    assert v <= 2.0 + 1e-6 and a <= 2.0 + 1e-6 and j <= 5.0 + 1e-6
    assert max(v / 2.0, a / 2.0, j / 5.0) > 0.97  # not needlessly slow


def test_a_straight_flight_cruises_instead_of_ringing():
    """Fixed waypoint times made the speed swing between 1 and 2.9 m/s down a
    straight line; time redistribution leaves a ramp, a cruise and a ramp."""
    tr = MinSnapTrajectory(STRAIGHT, limits=Limits(3.0, 3.0, 10.0))
    ts = np.linspace(0.3 * tr.duration, 0.7 * tr.duration, 50)
    speed = np.linalg.norm(tr.evaluate(ts, 1), axis=1)
    assert speed.min() > 0.85 * 3.0
    fixed = MinSnapTrajectory(STRAIGHT, limits=Limits(3.0, 3.0, 10.0), optimize_times=False)
    assert tr.duration < 0.9 * fixed.duration


def test_given_durations_are_used_as_given():
    tr = MinSnapTrajectory(ZIGZAG, durations=[1.0, 2.0, 3.0, 4.0])
    np.testing.assert_allclose(tr.times, [0.0, 1.0, 3.0, 6.0, 10.0])
    with pytest.raises(ValueError):
        MinSnapTrajectory(ZIGZAG, durations=[1.0, 2.0])
    with pytest.raises(ValueError):
        MinSnapTrajectory(ZIGZAG, durations=[1.0, -2.0, 3.0, 4.0])


def test_repeated_waypoints_are_dropped_and_bad_input_refused():
    tr = MinSnapTrajectory(np.array([[0.0, 0, 0], [0, 0, 0], [3, 0, 0]]))
    assert len(tr.waypoints) == 2
    with pytest.raises(ValueError):
        MinSnapTrajectory(np.array([[0.0, 0, 0], [0, 0, 0]]))
    with pytest.raises(ValueError):
        MinSnapTrajectory(np.array([[0.0, 0, 0], [np.nan, 0, 0]]))
    with pytest.raises(ValueError):
        MinSnapTrajectory(ZIGZAG, limits=Limits(0.0, 1.0))


def test_evaluation_holds_the_ends_outside_the_trajectory(zigzag):
    np.testing.assert_allclose(zigzag.evaluate(-5.0), ZIGZAG[0])
    np.testing.assert_allclose(zigzag.evaluate(zigzag.duration + 5.0), ZIGZAG[-1], atol=1e-9)
    assert not np.any(zigzag.evaluate(zigzag.duration + 5.0, 1))


def test_flat_reference_yaw_modes(zigzag):
    t = 0.4 * zigzag.duration
    fixed = zigzag.flat_reference(t, yaw=0.3)
    assert fixed.yaw == 0.3 and fixed.yaw_rate == 0.0
    np.testing.assert_allclose(fixed.jerk, zigzag.evaluate(t, 3))
    assert zigzag.flat_reference(t, yaw=None).yaw is None
    along = zigzag.flat_reference(t, yaw="velocity")
    v = zigzag.evaluate(t, 1)
    assert along.yaw == pytest.approx(np.arctan2(v[1], v[0]))
    h = 1e-5
    yaw_rate_fd = (zigzag.flat_reference(t + h, "velocity").yaw - zigzag.flat_reference(t - h, "velocity").yaw) / (2 * h)
    assert along.yaw_rate == pytest.approx(yaw_rate_fd, abs=1e-5)
    with pytest.raises(ValueError):
        zigzag.flat_reference(t, yaw="north")


def test_path_trapezoid_allocation_matches_the_profile():
    d = np.array([1.5, 10.0, 1.5])  # ramp, cruise, ramp at 3 m/s and 3 m/s^2
    np.testing.assert_allclose(path_trapezoid_durations(d, 3.0, 3.0), [1.0, 10.0 / 3.0, 1.0])


def test_thinning_keeps_corners_and_ends_and_drops_the_grid():
    straight = np.array([[float(k), 0.0, 2.0] for k in range(21)])
    corner = np.vstack([straight, [[20.0, float(k), 2.0] for k in range(1, 21)]])
    thin = thin_waypoints(corner, min_spacing=3.0)
    np.testing.assert_allclose(thin[0], corner[0])
    np.testing.assert_allclose(thin[-1], corner[-1])
    assert any(np.allclose(p, [20.0, 0.0, 2.0]) for p in thin)  # the corner
    assert len(thin) < 10


def test_the_geometric_controller_flies_a_min_snap_trajectory_on_the_actuator_plant():
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent))
    import test_attitude_controllers as T
    from aerial_kit.dynamics.actuator_loop import fly

    W = np.array([[2.0, 0.0, 2.0], [6.0, 2.0, 2.5], [6.0, 6.0, 3.0], [2.0, 8.0, 2.0]])
    tr = MinSnapTrajectory(W, limits=Limits(2.5, 3.0, 8.0))
    ctl = T.geometric(reference=tr.as_reference(yaw=None))
    trace = fly(ctl, T.airframe(), T.plant(2.0, east_m=2.0), np.zeros(3), steps=int((tr.duration + 2.0) / T.DT), dt=T.DT)
    ref = tr.evaluate(trace.t)
    err = np.linalg.norm(trace.position - ref, axis=1)
    assert np.sqrt(np.mean(err**2)) < 0.05
    assert np.linalg.norm(trace.position[-1] - W[-1]) < 0.05
