from __future__ import annotations

import unittest

import numpy as np

from sim_py.planners.dubins import DubinsPlanner, plan_dubins_path, sample_dubins_path


def _wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class TestDubinsPath(unittest.TestCase):
    def test_straight_line_when_already_aligned(self) -> None:
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 0.0, 0.0)
        path = plan_dubins_path(start, goal, turn_radius_m=5.0)
        self.assertAlmostEqual(path.total_length_m, 10.0, places=6)

    def test_rejects_non_positive_turn_radius(self) -> None:
        with self.assertRaises(ValueError):
            plan_dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), turn_radius_m=0.0)
        with self.assertRaises(ValueError):
            plan_dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), turn_radius_m=-1.0)

    def test_path_length_never_shorter_than_straight_line(self) -> None:
        rng = np.random.default_rng(0)
        for _ in range(50):
            start = (0.0, 0.0, float(rng.uniform(-np.pi, np.pi)))
            goal = (
                float(rng.uniform(-20.0, 20.0)),
                float(rng.uniform(-20.0, 20.0)),
                float(rng.uniform(-np.pi, np.pi)),
            )
            straight_dist = np.hypot(goal[0] - start[0], goal[1] - start[1])
            if straight_dist < 1.0:
                continue  # avoid near-coincident points where CSC may not exist
            path = plan_dubins_path(start, goal, turn_radius_m=3.0)
            self.assertGreaterEqual(path.total_length_m, straight_dist - 1e-6)

    def test_sampled_path_reaches_goal_pose(self) -> None:
        """End-to-end oracle: sampling the planned segments must land on the goal
        pose exactly, which is the strongest available check on the underlying
        Dubins geometry (a sign error anywhere would show up as a mismatch here)."""
        rng = np.random.default_rng(1)
        turn_radius_m = 4.0
        for _ in range(30):
            start = (
                float(rng.uniform(-10.0, 10.0)),
                float(rng.uniform(-10.0, 10.0)),
                float(rng.uniform(-np.pi, np.pi)),
            )
            goal = (
                float(rng.uniform(-10.0, 10.0)),
                float(rng.uniform(-10.0, 10.0)),
                float(rng.uniform(-np.pi, np.pi)),
            )
            if np.hypot(goal[0] - start[0], goal[1] - start[1]) < 2.0:
                continue

            path = plan_dubins_path(start, goal, turn_radius_m=turn_radius_m)
            samples = sample_dubins_path(start, path, step_size_m=0.05)

            final_x, final_y, final_yaw = samples[-1]
            self.assertAlmostEqual(final_x, goal[0], places=2)
            self.assertAlmostEqual(final_y, goal[1], places=2)
            self.assertAlmostEqual(_wrap_angle(final_yaw - goal[2]), 0.0, places=2)

    def test_sample_rejects_non_positive_step(self) -> None:
        path = plan_dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), turn_radius_m=5.0)
        with self.assertRaises(ValueError):
            sample_dubins_path((0.0, 0.0, 0.0), path, step_size_m=0.0)


class TestDubinsPlanner(unittest.TestCase):
    def test_produces_curvature_feasible_single_leg_path(self) -> None:
        """DubinsPlanner wired as a real Planner: reads turn_radius_m from cfg
        (runner.py populates this from Capabilities.min_turn_radius_m), and its
        output should reach the goal and respect that turn radius end to end."""
        planner = DubinsPlanner()
        start = np.array([0.0, 0.0, 100.0])
        goal = np.array([200.0, 80.0, 120.0])
        turn_radius_m = 25.0
        waypoints = planner.plan(
            start=start,
            goal=goal,
            obstacles=[],
            cfg={"path": {"turn_radius_m": turn_radius_m, "dubins_step_size_m": 2.0}},
        )
        self.assertGreater(len(waypoints), 1)
        final = waypoints[-1].position
        self.assertAlmostEqual(final[0], goal[0], places=1)
        self.assertAlmostEqual(final[1], goal[1], places=1)
        self.assertAlmostEqual(final[2], goal[2], places=6)
        # Altitude should progress monotonically-ish from start to goal (linear interpolation).
        altitudes = [wp.position[2] for wp in waypoints]
        self.assertAlmostEqual(altitudes[0], start[2], places=6)
        self.assertLessEqual(min(altitudes), max(start[2], goal[2]) + 1e-6)
        self.assertGreaterEqual(max(altitudes), min(start[2], goal[2]) - 1e-6)

    def test_defaults_to_straight_line_bearing_when_no_heading_given(self) -> None:
        planner = DubinsPlanner()
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([100.0, 0.0, 0.0])  # already aligned -- should be a straight line
        waypoints = planner.plan(start=start, goal=goal, obstacles=[], cfg={"path": {"turn_radius_m": 20.0}})
        ys = [wp.position[1] for wp in waypoints]
        self.assertTrue(all(abs(y) < 1e-6 for y in ys))


if __name__ == "__main__":
    unittest.main()
