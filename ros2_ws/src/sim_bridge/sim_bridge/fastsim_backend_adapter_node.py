from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def _ensure_repo_root_on_path() -> None:
    repo_root = Path(__file__).resolve().parents[5]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))


_ensure_repo_root_on_path()
from aerial_kit.dynamics.pointmass import PointMassDynamics, PointMassParams  # noqa: E402


class FastSimBackendAdapterNode(Node):
    """Headless point-mass backend exposing the same backend topics as Gazebo mode."""

    def __init__(self) -> None:
        super().__init__('fastsim_backend_adapter_node')
        self.declare_parameter('backend_cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('backend_enable_topic', '/uav/backend/enable')
        self.declare_parameter('backend_odom_topic', '/uav/backend/odom')
        self.declare_parameter('sim_rate_hz', 50.0)
        self.declare_parameter('kv_drag', 0.15)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('start_z', 0.0)

        self.backend_cmd_topic = self.get_parameter('backend_cmd_topic').value
        self.backend_enable_topic = self.get_parameter('backend_enable_topic').value
        self.backend_odom_topic = self.get_parameter('backend_odom_topic').value
        self.sim_rate_hz = float(self.get_parameter('sim_rate_hz').value)
        self.max_accel = float(self.get_parameter('max_accel').value)

        self.dyn = PointMassDynamics(PointMassParams(kv_drag=float(self.get_parameter('kv_drag').value)))
        self.dyn.position[2] = float(self.get_parameter('start_z').value)

        self.enabled = False
        self.last_cmd = Twist()
        self.last_vel_cmd = [0.0, 0.0, 0.0]

        self.create_subscription(Twist, self.backend_cmd_topic, self._on_cmd, 20)
        self.create_subscription(Bool, self.backend_enable_topic, self._on_enable, 20)
        self.pub_odom = self.create_publisher(Odometry, self.backend_odom_topic, 20)

        self.timer = self.create_timer(1.0 / max(self.sim_rate_hz, 1.0), self._tick)
        self.get_logger().info(f'Fast sim backend publishing odom on {self.backend_odom_topic}')

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_vel_cmd = [float(msg.linear.x), float(msg.linear.y), float(msg.linear.z)]

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _tick(self) -> None:
        dt = 1.0 / max(self.sim_rate_hz, 1.0)
        if self.enabled:
            # Convert commanded velocity to acceleration target via P term.
            vel_err = [
                self.last_vel_cmd[0] - float(self.dyn.velocity[0]),
                self.last_vel_cmd[1] - float(self.dyn.velocity[1]),
                self.last_vel_cmd[2] - float(self.dyn.velocity[2]),
            ]
            kp = 1.5
            acc = [kp * e for e in vel_err]
        else:
            acc = [
                -1.0 * float(self.dyn.velocity[0]),
                -1.0 * float(self.dyn.velocity[1]),
                -1.5 * float(self.dyn.velocity[2]),
            ]

        # Clamp acceleration.
        for i in range(3):
            if acc[i] > self.max_accel:
                acc[i] = self.max_accel
            elif acc[i] < -self.max_accel:
                acc[i] = -self.max_accel

        self.dyn.step(acc, dt)
        if self.dyn.position[2] < 0.0:
            self.dyn.position[2] = 0.0
            if self.dyn.velocity[2] < 0.0:
                self.dyn.velocity[2] = 0.0

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.dyn.position[0])
        odom.pose.pose.position.y = float(self.dyn.position[1])
        odom.pose.pose.position.z = float(self.dyn.position[2])
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(self.dyn.velocity[0])
        odom.twist.twist.linear.y = float(self.dyn.velocity[1])
        odom.twist.twist.linear.z = float(self.dyn.velocity[2])
        self.pub_odom.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastSimBackendAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
