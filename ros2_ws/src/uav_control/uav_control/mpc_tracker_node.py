"""ROS 2 mission tracker driven by a constrained MPC or an MPPI controller.

A drop-in alternative to ``air_unit``'s ``mission_executor_node`` for offboard
missions: same input topics (mission, command, raw telemetry), same outputs (a
``Twist`` velocity setpoint on the internal mission command topic, which
``command_manager_node`` arbitrates, plus ``MissionStatus``). Launch one or the
other, not both - they publish to the same topic.

What it adds over the executor's P-controller:

* the command comes from a plan over a horizon (``horizon`` x ``plan_dt``),
  with acceleration and speed limits inside the optimisation;
* intermediate waypoints are flown through, not stopped at - see
  :mod:`uav_control.tracking_core`;
* with ``controller: mppi``, spherical obstacles (``obstacle_spheres``) and a
  floor (``min_altitude_m``) are part of the cost;
* the predicted trajectory is published as a ``nav_msgs/Path`` for RViz.

Onboard planning (``Command.PLANNING_ONBOARD``) is the executor's job; this node
flies the trajectory it is given.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from uav_msgs.msg import Command, MissionStatus, Telemetry, Trajectory

from .tracking_core import (
    STATE_IDLE,
    STATE_PAUSED,
    TrackedWaypoint,
    TrackerConfig,
    TrackingCore,
    heading_for_velocity,
    parse_sphere_obstacles,
    world_to_body,
    wrap_angle,
)

_TRACKER_PARAMS = {
    # name: default (types follow the defaults)
    'controller': 'mpc',
    'plan_dt': 0.1,
    'horizon': 20,
    'cruise_speed_mps': 2.0,
    'max_xy_speed_mps': 2.0,
    'max_z_speed_mps': 1.0,
    'max_accel_xy_mps2': 2.5,
    'max_accel_z_mps2': 1.5,
    'approach_decel_mps2': 1.0,
    'settle_speed_mps': 0.3,
    'velocity_loop_tau_s': 0.25,
    'q_pos': 8.0,
    'q_vel': 1.0,
    'r_acc': 0.5,
    'r_delta': 2.0,
    'speed_limit_xy': 'disc',
    'mppi_samples': 384,
    'mppi_temperature': 1.0,
    'mppi_noise_std': 1.0,
    'mppi_seed': 0,
    'obstacle_margin_m': 0.5,
}


class MpcTrackerNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build it in-process.
        super().__init__('mpc_tracker_node', **node_kwargs)
        self.declare_parameter('mission_topic', '/uav/mission')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('telemetry_raw_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('mission_cmd_topic', '/uav/internal/mission_cmd_vel')
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('predicted_path_topic', '/uav/control/predicted_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('command_frame', 'body')  # body | world
        self.declare_parameter('nose_axis', '+y')  # +y (Gazebo models here) | +x
        self.declare_parameter('body_lateral_enabled', False)
        self.declare_parameter('heading_control_enabled', True)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('max_yaw_rate_rps', 1.2)
        self.declare_parameter('heading_min_distance_m', 0.3)
        self.declare_parameter('yaw_alignment_min_speed_scale', 0.1)
        self.declare_parameter('min_altitude_m', -1.0)  # < 0 disables (MPPI only)
        self.declare_parameter('obstacle_spheres', [0.0])  # flat [x, y, z, r, ...] (MPPI only)
        for name, default in _TRACKER_PARAMS.items():
            self.declare_parameter(name, default)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        min_alt = float(p('min_altitude_m'))
        cfg = TrackerConfig(
            **{name: type(default)(p(name)) for name, default in _TRACKER_PARAMS.items()},
            min_altitude_m=min_alt if min_alt >= 0.0 else None,
        )
        spheres = [float(x) for x in p('obstacle_spheres')]
        obstacles = parse_sphere_obstacles(spheres) if len(spheres) >= 4 else []
        if obstacles and cfg.controller != 'mppi':
            self.get_logger().warn('obstacle_spheres is only used by the mppi controller')
        self.core = TrackingCore(cfg, obstacles)

        self.frame_id = str(p('frame_id'))
        self.command_frame = str(p('command_frame')).strip().lower()
        self.nose_axis = str(p('nose_axis')).strip()
        self.body_lateral_enabled = bool(p('body_lateral_enabled'))
        self.heading_control_enabled = bool(p('heading_control_enabled'))
        self.yaw_kp = float(p('yaw_kp'))
        self.max_yaw_rate_rps = abs(float(p('max_yaw_rate_rps')))
        self.heading_min_distance_m = float(p('heading_min_distance_m'))
        self.yaw_alignment_min_speed_scale = float(p('yaw_alignment_min_speed_scale'))

        self.pub_cmd = self.create_publisher(Twist, str(p('mission_cmd_topic')), 10)
        self.pub_status = self.create_publisher(MissionStatus, str(p('mission_status_topic')), 10)
        self.pub_path = self.create_publisher(Path, str(p('predicted_path_topic')), 5)
        self.create_subscription(Trajectory, str(p('mission_topic')), self._on_mission, 10)
        self.create_subscription(Telemetry, str(p('telemetry_raw_topic')), self._on_telemetry, 20)
        self.create_subscription(Command, str(p('command_topic')), self._on_command, 20)

        self.position = None
        self.velocity = None
        self.yaw = None
        self.mode = Command.MODE_IDLE
        self.manual_override = False
        self.mission_sequence_id = 0
        self._was_active = False

        period = 1.0 / max(float(p('control_rate_hz')), 1.0)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'mpc_tracker_node: controller={cfg.controller} horizon={cfg.horizon}x{cfg.plan_dt:.2f}s '
            f'obstacles={len(obstacles)}'
        )

    # -- callbacks ----------------------------------------------------------

    def _on_mission(self, msg: Trajectory) -> None:
        self.mission_sequence_id = int(msg.sequence_id)
        waypoints = [
            TrackedWaypoint(
                position=np.array([w.pose.position.x, w.pose.position.y, w.pose.position.z], dtype=float),
                acceptance_radius_m=float(w.acceptance_radius_m),
                hold_time_sec=float(w.hold_time_sec),
                desired_speed_mps=float(w.desired_speed_mps),
            )
            for w in msg.waypoints
        ]
        start = None if self.position is None else np.array(self.position)
        self.core.load_mission(waypoints, start)
        self.get_logger().info(f'mission {self.mission_sequence_id}: {len(waypoints)} waypoints')

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.velocity = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_command(self, msg: Command) -> None:
        self.mode = int(msg.mode_request)
        self.manual_override = bool(msg.manual_override)

    # -- loop ---------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        if not self.core.has_mission:
            return self._stop(STATE_IDLE, False, 'no mission')
        if self.position is None:
            return self._stop(STATE_IDLE, False, 'waiting for telemetry')
        if self.manual_override:
            return self._stop(STATE_PAUSED, False, 'paused: manual override')
        if self.mode != Command.MODE_MISSION:
            return self._stop(STATE_PAUSED, self.core.complete, 'paused: mode != mission')

        self._was_active = True
        vel = self.velocity if self.velocity is not None else (0.0, 0.0, 0.0)
        out = self.core.step(self._now_s(), np.array(self.position), np.array(vel))

        cmd = Twist()
        v = out.velocity_world.copy()
        dx, dy = (float(c) for c in out.planned_displacement_xy)
        if self.heading_control_enabled and self.yaw is not None and math.hypot(dx, dy) > self.heading_min_distance_m:
            # Face where the plan is going over the horizon (see TrackerOutput).
            yaw_err = wrap_angle(heading_for_velocity(dx, dy, self.nose_axis) - self.yaw)
            cmd.angular.z = max(-self.max_yaw_rate_rps, min(self.max_yaw_rate_rps, self.yaw_kp * yaw_err))
            if not self.body_lateral_enabled and self.command_frame == 'body':
                # Forward-only vehicles must turn before they translate.
                align = max(self.yaw_alignment_min_speed_scale, math.cos(min(abs(yaw_err), math.pi / 2.0)))
                v[:2] *= align
        if self.command_frame == 'body' and self.yaw is not None:
            v = world_to_body(v, self.yaw, self.nose_axis, lateral=self.body_lateral_enabled)
        cmd.linear.x, cmd.linear.y, cmd.linear.z = (float(x) for x in v)
        self.pub_cmd.publish(cmd)
        self._publish_status(out.state, out.complete, out.active_index, out.total_waypoints, out.status_text)
        self._publish_path(out.predicted_positions)

    def _stop(self, state: int, complete: bool, text: str) -> None:
        if self._was_active:
            self.core.pause()  # the warm-start plan is stale after any interruption
            self._was_active = False
        self.pub_cmd.publish(Twist())
        self._publish_status(state, complete, self.core.active_index, len(self.core.waypoints), text)

    def _publish_status(self, state: int, complete: bool, index: int, total: int, text: str) -> None:
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_sequence_id = self.mission_sequence_id
        msg.active_waypoint_index = int(index)
        msg.total_waypoints = int(total)
        msg.state = int(state)
        msg.complete = bool(complete)
        msg.status_text = text
        self.pub_status.publish(msg)

    def _publish_path(self, positions: np.ndarray) -> None:
        if positions.size == 0:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for x, y, z in positions:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
