"""Drive the Gazebo twin-wing visual from the authoritative Aerial Kit plant."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from uav_msgs.msg import Telemetry


class GazeboFixedWingPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_fixedwing_pose_node')
        self.declare_parameter('telemetry_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter(
            'set_pose_service', '/world/uav_fixed_wing/set_pose')
        self.declare_parameter('entity_name', 'twin_wing')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.latest = None
        self.pending = None
        self.sent = 0
        self.failures = 0
        service_name = str(self.get_parameter('set_pose_service').value)
        self.client = self.create_client(SetEntityPose, service_name)
        self.create_subscription(
            Telemetry, self.get_parameter('telemetry_topic').value,
            self._on_telemetry, 20)
        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'twin-wing visual follows {self.get_parameter("telemetry_topic").value} '
            f'through {service_name}')

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.latest = msg

    def _tick(self) -> None:
        if self.latest is None or not self.client.service_is_ready():
            return
        if self.pending is not None:
            if not self.pending.done():
                return
            self._finish(self.pending)

        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter('entity_name').value)
        request.entity.type = Entity.MODEL
        request.pose = self.latest.pose
        self.pending = self.client.call_async(request)

    def _finish(self, future) -> None:
        self.pending = None
        try:
            response = future.result()
        except Exception as exc:  # service transport can fail during shutdown
            self.failures += 1
            if self.failures == 1:
                self.get_logger().warning(f'set-pose call failed: {exc}')
            return
        if response is None or not response.success:
            self.failures += 1
            if self.failures == 1:
                self.get_logger().warning('Gazebo refused the twin-wing pose')
            return
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info('Gazebo twin-wing pose stream established')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboFixedWingPoseNode()
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
