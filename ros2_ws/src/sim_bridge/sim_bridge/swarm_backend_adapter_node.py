"""ROS 2 plant for N multirotors in one shared MuJoCo scene.

Every drone gets the single-vehicle backend contract under its own namespace,
so anything that flies one ``/uav/backend`` can fly drone ``i`` at
``/<prefix><i>/backend``:

    /<prefix><i>/backend/cmd_twist   geometry_msgs/Twist   world ENU velocity + yaw rate
    /<prefix><i>/backend/enable      std_msgs/Bool         arm
    /<prefix><i>/backend/odom        nav_msgs/Odometry     (published)

Per drone, the same control-source rules as the single multirotor adapter:
fresh cmd_twist flies, enabled-but-stale holds, disarmed in the air lands,
disarmed on the ground idles. Drones share the scene, so they can collide.

    ros2 run sim_bridge swarm_backend_adapter_node --ros-args -p n:=12 -p viewer:=true
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

for _parent in Path(__file__).resolve().parents:
    if (_parent / 'aerial_kit').is_dir() and (_parent / 'sim_py').is_dir():
        if str(_parent) not in sys.path:
            sys.path.insert(0, str(_parent))
        break

from aerial_kit.controllers.cascade import CascadeGains  # noqa: E402
from aerial_kit.controllers.multirotor_manual import ManualLimits, MultirotorPilot  # noqa: E402
from aerial_kit.types import ControlTarget  # noqa: E402
from sim_py.backends.mujoco_guard import SimulationDiverged  # noqa: E402
from sim_py.backends.mujoco_swarm import MujocoSwarm  # noqa: E402

from sim_bridge.fixedwing_backend_adapter_node import close_mujoco_viewer, open_mujoco_viewer  # noqa: E402
from sim_bridge.multirotor_backend_adapter_node import (  # noqa: E402
    LAND_RATE_MPS, SOURCE_HOLD, SOURCE_IDLE, SOURCE_LAND, SOURCE_TWIST, select_source,
)

MAX_DRONES = 64


class _Drone:
    def __init__(self) -> None:
        self.enabled = False
        self.twist: Twist | None = None
        self.twist_time: float | None = None
        self.source: str | None = None


class SwarmBackendAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('swarm_backend_adapter_node')
        for name, default in (
            ('n', 4), ('airframe', 'swarm_quad'), ('spacing_m', 1.5), ('namespace_prefix', 'uav'),
            ('sim_rate_hz', 100.0), ('cmd_timeout_sec', 0.5), ('viewer', False),
        ):
            self.declare_parameter(name, default)
        g = self.get_parameter
        n = int(g('n').value)
        if not 1 <= n <= MAX_DRONES:
            raise ValueError(f'n must be 1..{MAX_DRONES}, got {n}')
        rate = float(g('sim_rate_hz').value)
        if not rate > 0.0:
            raise ValueError('sim_rate_hz must be positive')
        self.period = 1.0 / rate
        self.cmd_timeout = float(g('cmd_timeout_sec').value)
        self.prefix = str(g('namespace_prefix').value).strip('/') or 'uav'
        self.swarm = MujocoSwarm(n=n, airframe=str(g('airframe').value), spacing_m=float(g('spacing_m').value))
        spec = self.swarm.spec
        gains = self.swarm.vehicles[0].controller_defaults('cascade')
        self.pilots = [MultirotorPilot(CascadeGains(**gains), ManualLimits.from_spec(spec)) for _ in range(n)]
        self.drones = [_Drone() for _ in range(n)]
        self.pubs = []
        for i in range(n):
            ns = self.namespace_of(i)
            self.create_subscription(Twist, f'{ns}/backend/cmd_twist', lambda m, i=i: self._on_twist(i, m), 10)
            self.create_subscription(Bool, f'{ns}/backend/enable', lambda m, i=i: self._on_enable(i, m), 10)
            self.pubs.append(self.create_publisher(Odometry, f'{ns}/backend/odom', 10))
        self.diverged = False
        self.viewer = None
        if bool(g('viewer').value):
            self.viewer = open_mujoco_viewer(self.swarm.model, self.swarm.data, self.get_logger(),
                                             distance=max(4.0, float(g('spacing_m').value) * (n ** 0.5) * 2.0))
        self.timer = self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f'swarm of {n} x {spec.name} in one MuJoCo scene; topics /{self.prefix}<i>/backend/*, '
            f'i = 0..{n - 1}')

    def namespace_of(self, i: int) -> str:
        return f'/{self.prefix}{i}'

    def _on_twist(self, i: int, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if not all(np.isfinite(values)):
            return
        d = self.drones[i]
        d.twist, d.twist_time = msg, time.monotonic()

    def _on_enable(self, i: int, msg: Bool) -> None:
        self.drones[i].enabled = bool(msg.data)

    def _target(self, i: int, now: float) -> ControlTarget:
        v = self.swarm.vehicles[i]
        d = self.drones[i]
        state = v.state()
        source = select_source(
            now, enabled=d.enabled, manual_active=False, manual_time=None, manual_timeout_s=0.0,
            twist_time=d.twist_time, twist_timeout_s=self.cmd_timeout, on_ground=v.on_ground(),
            height_agl_m=float(state.position[2]) - self.swarm.rest_z)
        pilot = self.pilots[i]
        if source != d.source:
            self.get_logger().info(f'{self.prefix}{i}: control source {d.source} -> {source}')
            if source in (SOURCE_HOLD, SOURCE_TWIST):
                pilot.reset_yaw(state)
            d.source = source
        if source == SOURCE_TWIST:
            t = d.twist
            return ControlTarget(accel_cmd=np.zeros(3), wrench=pilot.velocity_wrench(
                state, [t.linear.x, t.linear.y, t.linear.z], float(t.angular.z), self.period))
        if source == SOURCE_HOLD:
            return ControlTarget(accel_cmd=np.zeros(3), wrench=pilot.hold_wrench(state, self.period))
        if source == SOURCE_LAND:
            return ControlTarget(accel_cmd=np.zeros(3), wrench=pilot.velocity_wrench(
                state, [0.0, 0.0, -LAND_RATE_MPS], 0.0, self.period))
        assert source == SOURCE_IDLE, source
        return ControlTarget(accel_cmd=np.zeros(3), metadata={'rotor_thrust_n': np.zeros(self.swarm.spec.n_rotors)})

    def _tick(self) -> None:
        now = time.monotonic()
        if not self.diverged:
            try:
                self.swarm.step([self._target(i, now) for i in range(self.swarm.n)], self.period)
            except SimulationDiverged as exc:
                self.diverged = True
                self.get_logger().error(f'plant diverged, simulation frozen: {exc}')
        if self.viewer is not None:
            try:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    self.viewer = None
            except Exception as exc:
                self.get_logger().warning(f'MuJoCo viewer stopped, continuing headless: {exc}')
                self.viewer = None
        stamp = self.get_clock().now().to_msg()
        for i, state in enumerate(self.swarm.states()):
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'map'
            odom.child_frame_id = f'{self.prefix}{i}/base_link'
            p, q, v, w = state.position, state.attitude_quat, state.velocity, state.body_rates
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = (float(x) for x in p)
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x = float(q[0]), float(q[1])
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = float(q[2]), float(q[3])
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = (float(x) for x in v)
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = (float(x) for x in w)
            self.pubs[i].publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SwarmBackendAdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and node.viewer is not None:
            close_mujoco_viewer(node.viewer)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
