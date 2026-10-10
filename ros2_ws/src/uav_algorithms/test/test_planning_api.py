"""Planning around the obstacles that are actually published.

No ROS needed: markers are duck-typed, and the planner is sim_py's.
"""

import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_algorithms.planning_api import (  # noqa: E402
    plan_trajectory_points_with_obstacles,
    terrain_obstacles_from_markers,
    validate_trajectory_segments,
)


def marker(kind, pos, scale, action=0):
    return NS(type=kind, action=action, pose=NS(position=NS(x=pos[0], y=pos[1], z=pos[2])),
              scale=NS(x=scale[0], y=scale[1], z=scale[2]))


def test_markers_keep_the_planners_geometry_conventions():
    tree, rock = terrain_obstacles_from_markers([
        marker(0, (0, 0, 0), (0, 0, 0), action=3),
        marker(3, (5.0, 1.0, 4.0), (1.0, 1.0, 8.0)),  # mid-height pose
        marker(1, (2.0, 2.0, 1.0), (2.0, 4.0, 2.0)),
        marker(3, (9.0, 9.0, 1.0), (1.0, 1.0, 2.0), action=2),  # DELETE: skipped
    ])
    # Cylinder center is its base; the box center is its middle.
    np.testing.assert_allclose(tree.center, [5.0, 1.0, 0.0])
    assert (tree.height, tree.radius) == (8.0, 0.5)
    assert tree.is_inside([5.0, 1.0, 7.9]) and not tree.is_inside([5.0, 1.0, 8.5])
    np.testing.assert_allclose(rock.center, [2.0, 2.0, 1.0])
    assert rock.is_inside([2.9, 3.9, 0.1]) and not rock.is_inside([2.0, 2.0, 2.5])


def test_the_planner_routes_around_a_given_obstacle_set():
    """A wall of trees across the straight line, given rather than generated."""
    wall = terrain_obstacles_from_markers([
        marker(3, (20.0, float(y), 7.5), (2.0, 2.0, 15.0)) for y in range(10, 31, 2)
    ])
    start, goal = [10.0, 20.0, 3.0], [30.0, 20.0, 3.0]
    points, used = plan_trajectory_points_with_obstacles(
        start, goal, planner_type="astar", terrain_profile="plains",
        collision_inflation_m=1.0, obstacles=wall,
    )
    assert used is wall
    ok, reason = validate_trajectory_segments(points, wall, inflation_m=0.5)
    assert ok, reason
    np.testing.assert_allclose(points[-1][:2], goal[:2], atol=1.5)


def test_everything_the_planner_server_imports_is_exported():
    """planner_server_node imports from the package root; a name missing from
    __init__ crashed the node at startup while every other test passed."""
    import uav_algorithms

    for name in ("plan_trajectory_points_with_obstacles", "terrain_obstacles_from_markers",
                 "validate_trajectory_segments"):
        assert hasattr(uav_algorithms, name)
