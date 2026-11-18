import numpy as np
import pytest

# Needs the built uav_msgs interfaces; the ROS 2 Jazzy CI job runs it.
Waypoint = pytest.importorskip('uav_msgs.msg').Waypoint

from uav_control.fixed_wing_guidance_node import active_waypoint


def waypoint(x, y, z, radius=5.0):
    wp = Waypoint()
    wp.pose.position.x = x
    wp.pose.position.y = y
    wp.pose.position.z = z
    wp.acceptance_radius_m = radius
    return wp


def test_active_waypoint_advances_past_an_arrival():
    points = [waypoint(5.0, 0.0, 20.0), waypoint(40.0, 0.0, 20.0)]
    index, wp, target = active_waypoint(
        points, np.array([0.0, 0.0, 20.0]), 0, 2.0)
    assert index == 1
    assert wp is points[1]
    np.testing.assert_allclose(target, [40.0, 0.0, 20.0])


def test_active_waypoint_uses_default_radius_and_reports_completion():
    points = [waypoint(1.0, 0.0, 20.0, radius=0.0)]
    index, wp, target = active_waypoint(
        points, np.array([0.0, 0.0, 20.0]), 0, 2.0)
    assert index == 1 and wp is None and target is None


def test_arrival_is_horizontal_even_during_an_altitude_transient():
    points = [waypoint(40.0, 0.0, 80.0, radius=5.0), waypoint(80.0, 0.0, 80.0)]
    index, wp, _ = active_waypoint(
        points, np.array([40.0, 0.0, 20.0]), 0, 2.0)
    assert index == 1 and wp is points[1]
