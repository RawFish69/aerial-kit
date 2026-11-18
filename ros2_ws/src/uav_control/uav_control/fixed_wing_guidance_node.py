"""ROS 2 wrapper for Aerial Kit's twin-wing L1/TECS controller."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from uav_msgs.msg import AirspeedNav, Command, Telemetry, Trajectory

for _parent in Path(__file__).resolve().parents:
    if (_parent / 'aerial_kit').is_dir() and (_parent / 'pyproject.toml').is_file():
        if str(_parent) not in sys.path:
            sys.path.insert(0, str(_parent))
        break

from aerial_kit.controllers.fixed_wing import FixedWingL1TECSController
from aerial_kit.types import SimState, Waypoint


def active_waypoint(waypoints, position, index, default_radius):
    """Advance through waypoints already inside their acceptance radius."""
    index = min(max(int(index), 0), len(waypoints))
    while index < len(waypoints):
        wp = waypoints[index]
        target = np.array([
            float(wp.pose.position.x),
            float(wp.pose.position.y),
            float(wp.pose.position.z),
        ])
        radius = float(wp.acceptance_radius_m)
        if radius <= 0.0:
            radius = default_radius
        # Fixed-wing altitude is held by TECS independently. Waypoint arrival
        # is therefore horizontal: a small vertical-energy transient must not
        # make the aircraft orbit a point it has already overflown.
        horizontal = float(np.linalg.norm(
            np.asarray(position, dtype=float)[:2] - target[:2]))
        if horizontal > radius:
            return index, wp, target
        index += 1
    return index, None, None


class FixedWingGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__('fixed_wing_guidance_node')
        self.declare_parameter('telemetry_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('mission_topic', '/uav/mission')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('airspeed_nav_topic', '/uav/backend/cmd_airspeed_nav')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('waypoint_acceptance_m', 8.0)
        self.declare_parameter('cruise_airspeed_mps', 15.0)
        self.declare_parameter('l1_distance_m', 40.0)
        self.declare_parameter('max_bank_rad', math.radians(45.0))
        self.declare_parameter('trim_throttle_n', 3.0)
        self.declare_parameter('max_thrust_n', 10.0)
        self.declare_parameter('tecs_kp_throttle', 0.15)
        self.declare_parameter('tecs_kp_pitch', 0.08)
        self.declare_parameter('kp_roll', 0.6)
        self.declare_parameter('kd_roll', 0.15)
        self.declare_parameter('kp_pitch', 0.8)
        self.declare_parameter('kd_pitch', 0.2)
        self.declare_parameter('kp_yaw_rate', 2.0)

        self.controller = FixedWingL1TECSController()
        self.telemetry = None
        self.trajectory = None
        self.sequence_id = 0
        self.waypoint_index = 0
        self.complete = False
        self.cfg = {
            'controller': {
                'l1_tecs': {
                    'cruise_airspeed_mps': float(self.get_parameter('cruise_airspeed_mps').value),
                    'l1_distance_m': float(self.get_parameter('l1_distance_m').value),
                    'max_bank_rad': float(self.get_parameter('max_bank_rad').value),
                    'trim_throttle_n': float(self.get_parameter('trim_throttle_n').value),
                    'max_thrust_n': float(self.get_parameter('max_thrust_n').value),
                    'tecs_kp_throttle': float(self.get_parameter('tecs_kp_throttle').value),
                    'tecs_kp_pitch': float(self.get_parameter('tecs_kp_pitch').value),
                    'kp_roll': float(self.get_parameter('kp_roll').value),
                    'kd_roll': float(self.get_parameter('kd_roll').value),
                    'kp_pitch': float(self.get_parameter('kp_pitch').value),
                    'kd_pitch': float(self.get_parameter('kd_pitch').value),
                    'kp_yaw_rate': float(self.get_parameter('kp_yaw_rate').value),
                }
            }
        }
        self.airframe = None
        self.command = None

        self.pub = self.create_publisher(
            AirspeedNav, self.get_parameter('airspeed_nav_topic').value, 10)
        self.create_subscription(
            Telemetry, self.get_parameter('telemetry_topic').value,
            self._on_telemetry, 20)
        self.create_subscription(
            Trajectory, self.get_parameter('mission_topic').value,
            self._on_trajectory, 10)
        self.create_subscription(
            Command, self.get_parameter('command_topic').value,
            self._on_command, 20)
        rate = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_telemetry(self, msg: Telemetry) -> None:
        q = msg.pose.orientation
        self.telemetry = SimState(
            position=np.array([
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
            velocity=np.array([
                msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]),
            t=float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec),
            attitude_quat=np.array([q.w, q.x, q.y, q.z]),
            body_rates=np.array([
                msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]),
            airspeed_mps=(
                float(msg.airspeed_mps) if msg.airspeed_mps > 0.0 else None),
        )

    def _on_trajectory(self, msg: Trajectory) -> None:
        if int(msg.sequence_id) != self.sequence_id:
            self.sequence_id = int(msg.sequence_id)
            self.waypoint_index = 0
            self.complete = False
            self.trajectory = msg

    def _on_command(self, msg: Command) -> None:
        self.command = msg

    def _inactive(self) -> bool:
        return (
            self.telemetry is None
            or self.trajectory is None
            or not self.trajectory.waypoints
            or self.command is None
            or not self.command.arm
            or int(self.command.mode_request) != Command.MODE_MISSION
        )

    def _tick(self) -> None:
        out = AirspeedNav()
        if self._inactive():
            out.valid = False
            self.pub.publish(out)
            return
        if self.complete:
            out.valid = False
            self.pub.publish(out)
            return

        index, waypoint, target = active_waypoint(
            self.trajectory.waypoints, self.telemetry.position,
            self.waypoint_index,
            float(self.get_parameter('waypoint_acceptance_m').value),
        )
        if index != self.waypoint_index:
            self.waypoint_index = index
            self.get_logger().info(
                f'fixed-wing waypoint {index}/{len(self.trajectory.waypoints)} '
                f'at {self.telemetry.position.round(1).tolist()}')
        if waypoint is None:
            self.complete = True
            self.get_logger().info(
                f'fixed-wing mission {self.sequence_id} complete; '
                f'position={self.telemetry.position.round(2).tolist()}')
            out.valid = False
            self.pub.publish(out)
            return

        wp = Waypoint(position=target)
        desired_speed = float(waypoint.desired_speed_mps)
        if desired_speed > 0.0:
            self.cfg['controller']['l1_tecs']['cruise_airspeed_mps'] = desired_speed
        target_out = self.controller.compute(self.telemetry, wp, self.cfg)
        wrench = target_out.wrench
        if wrench is None:
            out.valid = False
            self.pub.publish(out)
            return

        airspeed = max(float(np.linalg.norm(self.telemetry.velocity)), 1.0)
        metadata = target_out.metadata
        out.airspeed_mps = float(
            self.cfg['controller']['l1_tecs']['cruise_airspeed_mps'])
        out.bank_rad = float(metadata['bank_cmd'])
        out.climb_rate_mps = float(
            airspeed * math.sin(float(metadata['pitch_cmd'])))
        out.valid = True
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedWingGuidanceNode()
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
