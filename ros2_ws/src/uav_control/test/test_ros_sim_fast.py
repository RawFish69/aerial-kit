"""mpc_tracker_node in a real ROS 2 graph, against the sim_fast backend.

Runs the same nodes ``sim_fast/launch/bringup.launch.py`` starts - the
point-mass backend, the telemetry adapter and the command manager - with
``mpc_tracker_node`` in place of the mission executor, all in one process on a
single-threaded executor. Arm, take off, switch to mission mode, publish a
three-waypoint trajectory with a hold, and wait for MissionStatus.complete.

Skipped where rclpy is not importable (plain-pytest CI). With a sourced,
built workspace:

    cd ros2_ws/src/uav_control && python3 -m pytest test/test_ros_sim_fast.py -q
"""

import math
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

try:
    from uav_msgs.msg import Command, MissionStatus, Telemetry, Trajectory, Waypoint  # noqa: E402
except ImportError:  # rclpy without this workspace's messages built
    pytest.skip("uav_msgs is not built/sourced", allow_module_level=True)

_SRC = Path(__file__).resolve().parents[2]
for pkg in ("air_unit", "sim_bridge", "uav_control"):
    if str(_SRC / pkg) not in sys.path:
        sys.path.insert(0, str(_SRC / pkg))

from air_unit.command_manager_node import CommandManagerNode  # noqa: E402
from air_unit.telemetry_adapter_node import TelemetryAdapterNode  # noqa: E402
from sim_bridge.fastsim_backend_adapter_node import FastSimBackendAdapterNode  # noqa: E402
from uav_control.mpc_tracker_node import MpcTrackerNode  # noqa: E402


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("uav_control_test_driver")
        self.pub_command = self.create_publisher(Command, "/uav/command", 20)
        self.pub_mission = self.create_publisher(Trajectory, "/uav/mission", 10)
        self.create_subscription(Telemetry, "/uav/telemetry", self._on_telemetry, 20)
        self.create_subscription(MissionStatus, "/uav/mission_status", self._on_status, 20)
        self.telemetry = None
        self.status = None
        self.track = []
        self.statuses = []

    def _on_telemetry(self, msg) -> None:
        self.telemetry = msg
        p = msg.pose.position
        self.track.append((float(p.x), float(p.y), float(p.z)))

    def _on_status(self, msg) -> None:
        self.status = msg
        self.statuses.append(msg.status_text)

    def command(self, mode: int, arm: bool = False) -> None:
        msg = Command()
        msg.mode_request = int(mode)
        msg.planning_mode = Command.PLANNING_OFFBOARD
        msg.arm = bool(arm)
        msg.source_id = "uav_control-test"
        self.pub_command.publish(msg)

    def mission(self, points, hold_index=None) -> None:
        traj = Trajectory()
        traj.sequence_id = 42
        traj.frame_id = "map"
        for i, (x, y, z) in enumerate(points):
            wp = Waypoint()
            wp.pose.position.x, wp.pose.position.y, wp.pose.position.z = float(x), float(y), float(z)
            wp.pose.orientation.w = 1.0
            wp.acceptance_radius_m = 0.4
            wp.hold_time_sec = 1.0 if i == hold_index else 0.0
            wp.desired_speed_mps = 0.0
            traj.waypoints.append(wp)
        self.pub_mission.publish(traj)


def _hold_mode(driver, mode, predicate, timeout):
    """Keep commanding ``mode`` (the command manager times commands out) until ``predicate``."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        driver.command(mode)
        if predicate():
            return True
        time.sleep(0.1)
    return False


@pytest.mark.parametrize("controller", ["mpc", "mppi"])
def test_tracker_flies_a_mission_in_sim_fast(controller):
    rclpy.init()
    # sim_fast's backend integrates the twist as a world-frame velocity and has
    # no yaw, so the tracker is told to command in the world frame. Its velocity
    # loop is a = 1.5 * (v_sp - v): a time constant of 1 / 1.5 s.
    tracker_overrides = [
        Parameter("controller", value=controller),
        Parameter("velocity_loop_tau_s", value=1.0 / 1.5),
        Parameter("command_frame", value="world"),
        Parameter("heading_control_enabled", value=False),
    ]
    nodes = [
        FastSimBackendAdapterNode(),
        TelemetryAdapterNode(),
        CommandManagerNode(),
        MpcTrackerNode(parameter_overrides=tracker_overrides),
    ]
    driver = Driver()
    nodes.append(driver)
    # Single-threaded on purpose. With a MultiThreadedExecutor the tracker's
    # numpy work releases the GIL dozens of times per solve and, under rclpy's
    # waits, takes up to a timer period to get it back each time: solves that
    # take 3 ms alone took 0.7-3.5 s here, the loop ran at ~1 Hz on stale
    # telemetry and MPPI flew off. Launched normally, each node is its own
    # process with rclpy.spin, so that is a property of this harness only.
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        assert _hold_mode(driver, Command.MODE_IDLE, lambda: driver.telemetry is not None, 10.0), "no telemetry"

        driver.command(Command.MODE_TAKEOFF, arm=True)
        assert _hold_mode(
            driver, Command.MODE_TAKEOFF,
            lambda: driver.telemetry is not None and driver.telemetry.pose.position.z > 1.2,
            25.0,
        ), "takeoff did not reach altitude"

        points = [(4.0, 0.0, 2.0), (4.0, 4.0, 2.5), (0.0, 4.0, 2.0)]
        driver.mission(points, hold_index=1)
        time.sleep(0.3)
        done = _hold_mode(
            driver, Command.MODE_MISSION,
            lambda: driver.status is not None and driver.status.complete,
            90.0,
        )
        last = driver.status.status_text if driver.status else "no status"
        assert done, f"mission did not complete ({controller}): {last}; at {driver.track[-1]}"

        x, y, z = driver.track[-1]
        assert math.dist((x, y, z), points[-1]) < 0.6
        assert driver.status.mission_sequence_id == 42
        assert any(s.startswith("holding wp 2/3") for s in driver.statuses), "never held at wp 2"
        # It went out to the first corner rather than cutting straight to the goal.
        assert max(p[0] for p in driver.track) > 3.5
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        for node in nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()
