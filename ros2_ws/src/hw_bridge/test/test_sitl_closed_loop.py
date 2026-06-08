import math
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from drone_msgs.msg import Command, MissionStatus, Telemetry, Trajectory, Waypoint
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

_SRC_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC_DIR / "air_unit"))
sys.path.insert(0, str(_SRC_DIR / "hw_bridge"))

from air_unit.command_manager_node import CommandManagerNode
from air_unit.mission_executor_node import MissionExecutorNode
from air_unit.telemetry_adapter_node import TelemetryAdapterNode
from fake_fc_sim import FakeFcSimNode
from hw_bridge.crsf_backend_adapter_node import CrsfBackendAdapterNode
from hw_bridge.hw_state_estimator_node import HwStateEstimatorNode


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__("sitl_scenario_driver")
        self.pub_command = self.create_publisher(Command, "/uav/command", 20)
        self.pub_mission = self.create_publisher(Trajectory, "/uav/mission", 10)
        self.create_subscription(Telemetry, "/uav/telemetry", self._on_telemetry, 20)
        self.create_subscription(MissionStatus, "/uav/mission_status", self._on_mission_status, 20)
        self.latest_telemetry: Telemetry | None = None
        self.latest_mission_status: MissionStatus | None = None
        self.telemetry_samples: list[tuple[float, float, float, float, bool]] = []

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.latest_telemetry = msg
        self.telemetry_samples.append(
            (
                time.monotonic(),
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
                bool(msg.armed),
            )
        )
        if len(self.telemetry_samples) > 20000:
            self.telemetry_samples = self.telemetry_samples[-10000:]

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self.latest_mission_status = msg

    def publish_command(
        self,
        mode: int,
        *,
        arm: bool = False,
        disarm: bool = False,
        planning_mode: int = Command.PLANNING_OFFBOARD,
    ) -> None:
        msg = Command()
        msg.mode_request = int(mode)
        msg.planning_mode = int(planning_mode)
        msg.arm = bool(arm)
        msg.disarm = bool(disarm)
        msg.manual_override = False
        msg.source_id = "sitl-test"
        self.pub_command.publish(msg)

    def publish_trajectory(
        self,
        points: list[tuple[float, float, float]],
        *,
        sequence_id: int = 1,
        acceptance_radius: float = 0.4,
        desired_speed: float = 1.5,
    ) -> None:
        traj = Trajectory()
        traj.sequence_id = int(sequence_id)
        traj.frame_id = "map"
        for x, y, z in points:
            wp = Waypoint()
            wp.pose.position.x = float(x)
            wp.pose.position.y = float(y)
            wp.pose.position.z = float(z)
            wp.pose.orientation.w = 1.0
            wp.hold_time_sec = 0.0
            wp.acceptance_radius_m = float(acceptance_radius)
            wp.desired_speed_mps = float(desired_speed)
            traj.waypoints.append(wp)
        self.pub_mission.publish(traj)


def _wait_until(predicate, timeout_sec: float, sleep_sec: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(sleep_sec)
    return False


def _wait_with_mode(
    driver: ScenarioDriver,
    mode: int,
    predicate,
    *,
    timeout_sec: float,
    planning_mode: int = Command.PLANNING_OFFBOARD,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        driver.publish_command(mode, arm=False, planning_mode=planning_mode)
        if predicate():
            return True
        time.sleep(0.1)
    return False


@pytest.mark.slow
def test_closed_loop_hover_land_nav_rtl():
    rclpy.init()
    fake = FakeFcSimNode()
    estimator = HwStateEstimatorNode()
    crsf = CrsfBackendAdapterNode()
    telemetry = TelemetryAdapterNode()
    command_mgr = CommandManagerNode()
    mission = MissionExecutorNode()
    driver = ScenarioDriver()

    # Force loopback link for local SITL execution.
    crsf.udp_host = "127.0.0.1"
    crsf.udp_port = 9000

    nodes = [fake, estimator, crsf, telemetry, command_mgr, mission, driver]
    executor = MultiThreadedExecutor(num_threads=8)
    for node in nodes:
        executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert _wait_until(lambda: driver.latest_telemetry is not None, 10.0), "no telemetry received"
        home_xy = (
            float(driver.latest_telemetry.pose.position.x),
            float(driver.latest_telemetry.pose.position.y),
        )

        # Arm + takeoff
        driver.publish_command(Command.MODE_TAKEOFF, arm=True)
        assert _wait_with_mode(
            driver,
            Command.MODE_TAKEOFF,
            lambda: driver.latest_telemetry is not None
            and float(driver.latest_telemetry.pose.position.z) > 1.1
            and int(driver.latest_telemetry.current_mode) in (Command.MODE_TAKEOFF, Command.MODE_HOVER),
            timeout_sec=20.0,
        ), "takeoff did not reach expected altitude"
        assert _wait_with_mode(
            driver,
            Command.MODE_HOVER,
            lambda: driver.latest_telemetry is not None
            and int(driver.latest_telemetry.current_mode) == Command.MODE_HOVER,
            timeout_sec=8.0,
        ), "failed to enter hover"

        # Hover stability window
        samples: list[tuple[float, float, float]] = []
        hover_start = time.monotonic()
        while time.monotonic() - hover_start < 5.0:
            driver.publish_command(Command.MODE_HOVER, arm=False)
            if driver.latest_telemetry is not None:
                samples.append(
                    (
                        float(driver.latest_telemetry.pose.position.x),
                        float(driver.latest_telemetry.pose.position.y),
                        float(driver.latest_telemetry.pose.position.z),
                    )
                )
            time.sleep(0.1)
        assert samples, "no hover samples collected"
        x0, y0, z0 = samples[0]
        max_drift = max(math.hypot(x - x0, y - y0) for x, y, _ in samples)
        max_alt_dev = max(abs(z - z0) for _, _, z in samples)
        assert max_drift < 0.5, f"hover drift too large: {max_drift:.2f} m"
        assert max_alt_dev < 0.3, f"hover altitude deviation too large: {max_alt_dev:.2f} m"

        # Land and confirm disarm near ground.
        assert _wait_with_mode(
            driver,
            Command.MODE_LAND,
            lambda: driver.latest_telemetry is not None
            and (not bool(driver.latest_telemetry.armed))
            and float(driver.latest_telemetry.pose.position.z) < 0.2,
            timeout_sec=25.0,
        ), "land did not complete"

        # Rearm + takeoff for mission + RTL phases.
        driver.publish_command(Command.MODE_TAKEOFF, arm=True)
        assert _wait_with_mode(
            driver,
            Command.MODE_TAKEOFF,
            lambda: driver.latest_telemetry is not None
            and float(driver.latest_telemetry.pose.position.z) > 1.1,
            timeout_sec=20.0,
        ), "second takeoff failed"

        # Mission (GPS-nav) with two waypoints.
        mission_points = [(3.0, 0.0, 1.5), (3.0, 3.0, 1.5)]
        driver.publish_trajectory(mission_points, sequence_id=42)
        assert _wait_with_mode(
            driver,
            Command.MODE_MISSION,
            lambda: driver.latest_mission_status is not None and bool(driver.latest_mission_status.complete),
            timeout_sec=40.0,
        ), "mission did not reach complete"

        telemetry_now = driver.latest_telemetry
        assert telemetry_now is not None
        dist_from_home_before_rtl = math.hypot(
            float(telemetry_now.pose.position.x) - home_xy[0],
            float(telemetry_now.pose.position.y) - home_xy[1],
        )
        assert dist_from_home_before_rtl > 1.0, "vehicle did not move away from home before RTL"

        # RTL should return close to home and land/disarm.
        rtl_min_dist = float("inf")
        deadline = time.monotonic() + 45.0
        rtl_landed = False
        while time.monotonic() < deadline:
            driver.publish_command(Command.MODE_RTL, arm=False)
            if driver.latest_telemetry is not None:
                t = driver.latest_telemetry
                dist = math.hypot(
                    float(t.pose.position.x) - home_xy[0],
                    float(t.pose.position.y) - home_xy[1],
                )
                rtl_min_dist = min(rtl_min_dist, dist)
                if (not bool(t.armed)) and float(t.pose.position.z) < 0.2:
                    rtl_landed = True
                    break
            time.sleep(0.1)

        assert rtl_landed, "RTL did not finish with land/disarm"
        assert rtl_min_dist < 1.2, f"RTL never got close enough to home (min {rtl_min_dist:.2f} m)"
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        for node in nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()
