"""The tracker against obstacles in a real ROS 2 graph (sim_fast backend).

A tree stands on the straight line to the goal, published the way
``terrain_generator`` publishes it: a ``CYLINDER`` in a ``MarkerArray`` on
``/terrain/obstacles``, after a ``DELETEALL``.

* MPPI flies around it on its own.
* The MPC cannot; it stops, calls the planner service, and flies the detour it
  gets back. The planner here is a stand-in that returns a fixed detour, which
  keeps the test about the tracker rather than about A*.

Skipped where rclpy is not importable, like ``test_ros_sim_fast.py``.
"""

import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

try:
    from uav_msgs.msg import Command, Trajectory, Waypoint  # noqa: E402
    from uav_msgs.srv import PlanPath  # noqa: E402
    from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402
except ImportError:
    pytest.skip("uav_msgs / visualization_msgs not built/sourced", allow_module_level=True)

from test_ros_sim_fast import (  # noqa: E402
    CommandManagerNode,
    Driver,
    FastSimBackendAdapterNode,
    MpcTrackerNode,
    TelemetryAdapterNode,
    _hold_mode,
)
from aerial_kit.controllers.mppi import CylinderObstacle  # noqa: E402

TREE_XY, TREE_R, TREE_TOP = (6.0, 0.0), 0.6, 12.0
TREE = CylinderObstacle(TREE_XY, TREE_R, 0.0, TREE_TOP)
DETOUR = [(3.0, 3.0, 2.0), (9.0, 3.0, 2.0), (12.0, 0.0, 2.0)]


class World(Node):
    """Publishes the tree, and answers plan requests with a fixed detour."""

    def __init__(self) -> None:
        super().__init__("obstacle_test_world")
        self.pub = self.create_publisher(MarkerArray, "/terrain/obstacles", 5)
        self.create_service(PlanPath, "/uav/planner/plan_path", self._plan)
        self.requests = []
        self.create_timer(0.5, self._publish)

    def _publish(self) -> None:
        clear = Marker()
        clear.action = Marker.DELETEALL
        tree = Marker()
        tree.type = Marker.CYLINDER
        tree.pose.position.x, tree.pose.position.y, tree.pose.position.z = TREE_XY[0], TREE_XY[1], TREE_TOP / 2.0
        tree.pose.orientation.w = 1.0
        tree.scale.x = tree.scale.y = 2.0 * TREE_R
        tree.scale.z = TREE_TOP
        msg = MarkerArray()
        msg.markers = [clear, tree]
        self.pub.publish(msg)

    def _plan(self, request, response):
        self.requests.append(request)
        response.success = True
        response.message = "detour"
        for x, y, z in DETOUR:
            w = Waypoint()
            w.pose.position.x, w.pose.position.y, w.pose.position.z = x, y, z
            w.pose.orientation.w = 1.0
            w.acceptance_radius_m = 0.4
            response.trajectory.waypoints.append(w)
        return response


def fly(controller, replan):
    rclpy.init()
    tracker = MpcTrackerNode(parameter_overrides=[
        Parameter("controller", value=controller),
        Parameter("velocity_loop_tau_s", value=1.0 / 1.5),
        Parameter("command_frame", value="world"),
        Parameter("heading_control_enabled", value=False),
        Parameter("replan_on_block", value=replan),
        Parameter("replan_min_interval_s", value=1.0),
    ])
    world = World()
    driver = Driver()
    nodes = [FastSimBackendAdapterNode(), TelemetryAdapterNode(), CommandManagerNode(), tracker, world, driver]
    executor = SingleThreadedExecutor()  # see test_ros_sim_fast.py for why not multi-threaded
    for node in nodes:
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        assert _hold_mode(driver, Command.MODE_IDLE, lambda: driver.telemetry is not None, 10.0)
        driver.command(Command.MODE_TAKEOFF, arm=True)
        assert _hold_mode(
            driver, Command.MODE_TAKEOFF,
            lambda: driver.telemetry is not None and driver.telemetry.pose.position.z > 1.2, 25.0,
        )
        time.sleep(1.0)  # let the tree arrive
        driver.mission([(12.0, 0.0, 2.0)])
        time.sleep(0.3)
        done = _hold_mode(driver, Command.MODE_MISSION, lambda: driver.status is not None and driver.status.complete, 90.0)
        return done, np.array(driver.track), list(driver.statuses), world.requests, tracker.replans
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        for node in nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_mppi_flies_around_the_tree_without_a_replan():
    done, track, statuses, requests, replans = fly("mppi", replan=False)
    assert done, statuses[-1:]
    assert np.linalg.norm(track[-1] - np.array([12.0, 0.0, 2.0])) < 0.6
    assert float(np.min(TREE.sdf(track))) > 0.2
    assert not requests and replans == 0


def test_mpc_is_blocked_by_the_tree_replans_and_flies_the_detour():
    done, track, statuses, requests, replans = fly("mpc", replan=True)
    assert done, statuses[-1:]
    assert requests and replans >= 1
    req = requests[0]
    assert (req.goal.position.x, req.goal.position.y) == (12.0, 0.0)
    assert any(s.startswith("blocked") or s == "replanning" for s in statuses)
    assert float(np.min(TREE.sdf(track))) > 0.5
    assert track[:, 1].max() > 2.5  # it took the detour
