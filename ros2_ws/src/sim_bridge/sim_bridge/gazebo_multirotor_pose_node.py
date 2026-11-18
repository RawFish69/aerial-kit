"""Drive a kinematic Gazebo multirotor visual from the MuJoCo plant odometry."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class GazeboMultirotorPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_multirotor_pose_node')
        self.declare_parameter('odom_topic', '/uav/backend/odom')
        self.declare_parameter(
            'set_pose_service', '/world/uav_multirotor/set_pose')
        self.declare_parameter('entity_name', 'fpv_quad_5in')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.latest: Odometry | None = None
        self.pending = None
        self.sent = 0
        self.failures = 0
        service_name = str(self.get_parameter('set_pose_service').value)
        self.client = self.create_client(SetEntityPose, service_name)
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odometry,
            20,
        )
        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'multirotor visual follows {self.get_parameter("odom_topic").value} '
            f'through {service_name}'
        )

    def _on_odometry(self, msg: Odometry) -> None:
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
        request.pose = self.latest.pose.pose
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
                self.get_logger().warning(
                    f'Gazebo refused the {self.get_parameter("entity_name").value} pose')
            return
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info(
                'Gazebo multirotor pose stream established')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GazeboMultirotorPoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
