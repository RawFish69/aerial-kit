"""Self-contained mission source for the headless twin-wing ROS demo."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from uav_msgs.msg import Command, Telemetry, Trajectory, Waypoint


class FixedWingDemoNode(Node):
    def __init__(self) -> None:
        super().__init__('fixed_wing_demo_node')
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('cruise_speed_mps', 15.0)
        self.declare_parameter('telemetry_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('mission_topic', '/uav/mission')
        self.declare_parameter('enable_topic', '/uav/backend/enable')
        self.delay = float(self.get_parameter('start_delay_sec').value)
        self.started_at = None
        self.finished = False
        self.last_position = None

        self.pub_command = self.create_publisher(
            Command, self.get_parameter('command_topic').value, 10)
        self.pub_mission = self.create_publisher(
            Trajectory, self.get_parameter('mission_topic').value, 10)
        self.pub_enable = self.create_publisher(
            Bool, self.get_parameter('enable_topic').value, 10)
        self.create_subscription(
            Telemetry, self.get_parameter('telemetry_topic').value,
            self._on_telemetry, 20)
        self.timer = self.create_timer(0.5, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.started_at is None:
            self.started_at = now
            return
        elapsed = (now - self.started_at).nanoseconds / 1e9
        if elapsed < self.delay or self.finished:
            return

        command = Command()
        command.header.stamp = now.to_msg()
        command.arm = True
        command.mode_request = Command.MODE_MISSION
        self.pub_command.publish(command)
        enable = Bool()
        enable.data = True
        self.pub_enable.publish(enable)

        trajectory = Trajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.sequence_id = 1
        trajectory.frame_id = 'map'
        speed = float(self.get_parameter('cruise_speed_mps').value)
        for x, y, z in ((100.0, 60.0, 55.0), (220.0, 60.0, 55.0),
                        (260.0, -40.0, 55.0)):
            wp = Waypoint()
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = z
            wp.pose.orientation.w = 1.0
            wp.acceptance_radius_m = 12.0
            wp.desired_speed_mps = speed
            trajectory.waypoints.append(wp)
        self.pub_mission.publish(trajectory)

        if self.last_position is not None:
            dist = math.hypot(
                self.last_position[0] - 260.0,
                self.last_position[1] + 40.0,
            )
            if dist < 12.0:
                self.finished = True
                self.get_logger().info(
                    f'fixed-wing ROS demo complete: final distance {dist:.2f} m')

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.last_position = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        if not self.finished and abs(self.last_position[2]) > 1000.0:
            self.get_logger().error(f'fixed-wing state diverged: {self.last_position}')
            self.finished = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedWingDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
