import rclpy
from uav_msgs.msg import MissionStatus, Trajectory, Waypoint
from uav_msgs.srv import PlanPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from uav_algorithms import (
    plan_trajectory_points_with_obstacles,
    terrain_obstacles_from_markers,
    validate_trajectory_segments,
)


class PlannerServerNode(Node):
    def __init__(self) -> None:
        super().__init__('planner_server_node')
        self.declare_parameter('default_planner_type', 'astar')
        self.declare_parameter('default_terrain_profile', 'plains')
        self.declare_parameter('service_name', 'plan_path')
        self.declare_parameter('path_marker_topic', 'planned_path_markers')
        self.declare_parameter('path_marker_republish_hz', 1.0)
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('default_waypoint_speed_mps', 2.5)
        self.declare_parameter('default_acceptance_radius_m', 1.0)
        self.declare_parameter('default_collision_inflation_m', 1.5)
        self.declare_parameter('default_terrain_clearance_m', 2.0)
        self.declare_parameter('path_z_mode', 'low_corridor')
        self.declare_parameter('low_corridor_altitude_m', 3.0)
        self.declare_parameter('low_corridor_blend_start_goal', False)
        self.declare_parameter('max_path_altitude_m', 6.0)
        self.declare_parameter('validate_segments', True)
        self.declare_parameter('validation_step_m', 0.4)
        self.declare_parameter('astar_grid_resolution_m', 2.0)
        self.declare_parameter('rrt_step_size_m', 5.0)
        self.declare_parameter('rrt_goal_tolerance_m', 2.0)
        self.declare_parameter('rrt_max_iterations', 4000)
        self.declare_parameter('rrt_goal_sample_rate', 0.12)
        self.declare_parameter('rrt_star_rewire_radius_m', 0.0)

        self.default_planner_type = self.get_parameter('default_planner_type').value
        self.default_terrain_profile = self.get_parameter('default_terrain_profile').value
        service_name = self.get_parameter('service_name').value
        path_marker_topic = self.get_parameter('path_marker_topic').value
        mission_status_topic = self.get_parameter('mission_status_topic').value
        republish_hz = float(self.get_parameter('path_marker_republish_hz').value)
        self.default_waypoint_speed_mps = float(self.get_parameter('default_waypoint_speed_mps').value)
        self.default_acceptance_radius_m = float(self.get_parameter('default_acceptance_radius_m').value)
        self.default_collision_inflation_m = float(
            self.get_parameter('default_collision_inflation_m').value
        )
        self.default_terrain_clearance_m = float(
            self.get_parameter('default_terrain_clearance_m').value
        )
        self.path_z_mode = str(self.get_parameter('path_z_mode').value).strip().lower()
        self.low_corridor_altitude_m = float(self.get_parameter('low_corridor_altitude_m').value)
        self.low_corridor_blend_start_goal = bool(
            self.get_parameter('low_corridor_blend_start_goal').value
        )
        self.max_path_altitude_m = float(self.get_parameter('max_path_altitude_m').value)
        self.validate_segments = bool(self.get_parameter('validate_segments').value)
        self.validation_step_m = float(self.get_parameter('validation_step_m').value)
        self.astar_grid_resolution_m = float(self.get_parameter('astar_grid_resolution_m').value)
        self.rrt_step_size_m = float(self.get_parameter('rrt_step_size_m').value)
        self.rrt_goal_tolerance_m = float(self.get_parameter('rrt_goal_tolerance_m').value)
        self.rrt_max_iterations = int(self.get_parameter('rrt_max_iterations').value)
        self.rrt_goal_sample_rate = float(self.get_parameter('rrt_goal_sample_rate').value)
        self.rrt_star_rewire_radius_m = float(
            self.get_parameter('rrt_star_rewire_radius_m').value
        )
        self._logged_terrain_relative_passthrough = False
        self._last_points: list[list[float]] = []
        self._last_total_waypoints = 0
        self._last_active_waypoint_index = 0
        self._last_complete = False

        # TRANSIENT_LOCAL (durable) so late-joining subscribers (e.g. path_marker_spawner) get
        # the last retained path without needing to be up before planning finishes.
        _durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Where the obstacles come from: "generated" builds a forest from the
        # terrain config on every request (the original behaviour); "topic"
        # plans around what terrain_generator publishes; "auto" uses the topic
        # once anything has arrived on it and generates until then.
        self.declare_parameter('obstacle_source', 'auto')
        self.declare_parameter('obstacle_topic', '/terrain/obstacles')
        self.obstacle_source = str(self.get_parameter('obstacle_source').value).strip().lower()
        if self.obstacle_source not in ('auto', 'generated', 'topic'):
            raise ValueError(f"obstacle_source must be auto, generated or topic, got {self.obstacle_source!r}")
        self._published_obstacles = None
        if self.obstacle_source != 'generated':
            self.create_subscription(
                MarkerArray, str(self.get_parameter('obstacle_topic').value), self._on_obstacles, 5
            )
        self.srv = self.create_service(PlanPath, service_name, self._handle_plan_path)
        self.pub_path_markers = self.create_publisher(MarkerArray, path_marker_topic, _durable_qos)
        self.create_subscription(MissionStatus, mission_status_topic, self._on_mission_status, 20)
        # Periodic re-publish so Gazebo spawner always has fresh path even if it restarts.
        if republish_hz > 0.0:
            period = 1.0 / republish_hz
            self.create_timer(period, self._republish_path_markers)
        self.get_logger().info(f'Planner service ready: {self.get_namespace()}/{service_name}')

    def _on_obstacles(self, msg: MarkerArray) -> None:
        first = self._published_obstacles is None
        self._published_obstacles = terrain_obstacles_from_markers(msg.markers)
        if first:
            self.get_logger().info(
                f'planning around {len(self._published_obstacles)} published obstacles from now on'
            )

    def _obstacles_for_request(self):
        """The published obstacles, or None to have the planner generate them."""
        if self.obstacle_source == 'generated':
            return None
        if self._published_obstacles is None:
            if self.obstacle_source == 'topic':
                raise RuntimeError('obstacle_source is "topic" but nothing has arrived on the obstacle topic yet')
            return None
        return self._published_obstacles

    def _handle_plan_path(self, request: PlanPath.Request, response: PlanPath.Response) -> PlanPath.Response:
        planner_type = request.planner_type or self.default_planner_type
        terrain_profile = request.terrain_profile or self.default_terrain_profile
        collision_inflation_m = self._effective_collision_inflation(request)
        start = [request.start.position.x, request.start.position.y, request.start.position.z]
        goal = [request.goal.position.x, request.goal.position.y, request.goal.position.z]
        orig_start = list(start)
        orig_goal = list(goal)
        start, goal = self._normalize_request_altitudes(start, goal)
        planner_options = self._planner_options_for(planner_type)

        try:
            given = self._obstacles_for_request()
            self.get_logger().info(
                'obstacles: generated from the terrain config' if given is None
                else f'obstacles: {len(given)} published'
            )
            points, planning_obstacles = plan_trajectory_points_with_obstacles(
                obstacles=given,
                start_xyz=start,
                goal_xyz=goal,
                planner_type=planner_type,
                terrain_profile=terrain_profile,
                collision_inflation_m=collision_inflation_m,
                terrain_clearance_m=self.default_terrain_clearance_m,
                planner_options=planner_options,
            )
        except Exception as exc:
            response.success = False
            response.message = f'planning failed: {exc}'
            self.get_logger().error(response.message)
            return response

        points = self._apply_path_z_policy(points, orig_start, orig_goal)

        if self.validate_segments:
            try:
                ok, reason = validate_trajectory_segments(
                    points_xyz=points,
                    obstacles=planning_obstacles,
                    inflation_m=collision_inflation_m,
                    step_m=self.validation_step_m,
                )
            except Exception as exc:
                response.success = False
                response.message = f'plan validation failed: {exc}'
                self.get_logger().error(response.message)
                self._clear_path_markers()
                return response

            if not ok:
                response.success = False
                response.message = (
                    f'planner output rejected: {reason} '
                    f'({planner_type}/{terrain_profile}, inflation={collision_inflation_m:.2f}m)'
                )
                self.get_logger().warn(response.message)
                self._clear_path_markers()
                return response

        q = Quaternion()
        q.w = 1.0
        traj = Trajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'map'
        traj.frame_id = 'map'
        traj.sequence_id = 1

        for p in points:
            wp = Waypoint()
            wp.pose.position.x = float(p[0])
            wp.pose.position.y = float(p[1])
            wp.pose.position.z = float(p[2])
            wp.pose.orientation = q
            wp.hold_time_sec = 0.0
            wp.acceptance_radius_m = max(0.2, self.default_acceptance_radius_m)
            wp.desired_speed_mps = max(0.1, self.default_waypoint_speed_mps)
            traj.waypoints.append(wp)

        response.success = True
        response.message = (
            f'planned {len(traj.waypoints)} waypoints using {planner_type}/{terrain_profile} '
            f'(inflation={collision_inflation_m:.2f}m)'
        )
        response.trajectory = traj
        self._last_points = [list(map(float, p)) for p in points]
        self._last_total_waypoints = len(points)
        self._last_active_waypoint_index = 0
        self._last_complete = False
        self._publish_path_markers(self._last_points, active_waypoint_index=0, complete=False)
        self.get_logger().info(response.message)
        return response

    def _effective_collision_inflation(self, request: PlanPath.Request) -> float:
        requested = float(request.collision_inflation_m)
        if requested > 0.0:
            return requested
        return max(0.0, self.default_collision_inflation_m)

    def _planner_options_for(self, planner_type: str) -> dict:
        planner_key = str(planner_type).lower().strip()
        options: dict[str, float | int] = {
            'terrain_clearance': self.default_terrain_clearance_m,
        }
        if planner_key == 'astar':
            options['grid_resolution'] = self.astar_grid_resolution_m
        if planner_key in {'rrt', 'rrt*', 'rrtstar', 'rrt_star'}:
            options['rrt_step_size'] = self.rrt_step_size_m
            options['goal_tolerance'] = self.rrt_goal_tolerance_m
            options['rrt_max_iterations'] = self.rrt_max_iterations
        if planner_key in {'rrt*', 'rrtstar', 'rrt_star'}:
            options['rrt_goal_sample_rate'] = self.rrt_goal_sample_rate
            if self.rrt_star_rewire_radius_m > 0.0:
                options['rrt_star_rewire_radius'] = self.rrt_star_rewire_radius_m
        return options

    def _normalize_request_altitudes(self, start: list[float], goal: list[float]) -> tuple[list[float], list[float]]:
        mode = self.path_z_mode
        if mode != 'low_corridor':
            return start, goal
        corridor_z = float(self.low_corridor_altitude_m)
        return [start[0], start[1], corridor_z], [goal[0], goal[1], corridor_z]

    def _apply_path_z_policy(
        self,
        points,
        start_xyz: list[float],
        goal_xyz: list[float],
    ):
        mode = self.path_z_mode
        if mode == 'goal_altitude':
            return points

        if mode == 'low_corridor':
            if not points:
                return points
            corridor_z = float(self.low_corridor_altitude_m)
            z_cap = max(corridor_z, float(self.max_path_altitude_m))
            adjusted = []
            last_index = len(points) - 1
            for i, p in enumerate(points):
                p_list = [float(p[0]), float(p[1]), float(p[2])]
                if self.low_corridor_blend_start_goal and i == 0:
                    p_list[2] = float(start_xyz[2])
                elif self.low_corridor_blend_start_goal and i == last_index:
                    p_list[2] = float(goal_xyz[2])
                else:
                    p_list[2] = corridor_z
                p_list[2] = max(0.0, min(z_cap, p_list[2]))
                adjusted.append(p_list)
            return adjusted

        if mode == 'terrain_relative':
            # sim_py planners already emit terrain-aware waypoint altitudes; preserve them here.
            if not self._logged_terrain_relative_passthrough:
                self.get_logger().info(
                    'path_z_mode=terrain_relative: preserving planner-computed waypoint altitudes '
                    '(terrain-aware Z from sim_py planner)'
                )
                self._logged_terrain_relative_passthrough = True
            return [[float(p[0]), float(p[1]), float(p[2])] for p in points]

        return points

    def _republish_path_markers(self) -> None:
        """Periodically re-publish the last path so late-joining subscribers receive it."""
        if self._last_points:
            self._publish_path_markers(
                self._last_points,
                active_waypoint_index=self._last_active_waypoint_index,
                complete=self._last_complete,
            )

    def _clear_path_markers(self) -> None:
        self._last_points = []
        self._last_total_waypoints = 0
        self._last_active_waypoint_index = 0
        self._last_complete = False
        self._publish_path_markers([])

    def _on_mission_status(self, msg: MissionStatus) -> None:
        if not self._last_points:
            return
        if int(msg.total_waypoints) > 0 and int(msg.total_waypoints) != self._last_total_waypoints:
            return
        self._last_active_waypoint_index = int(msg.active_waypoint_index)
        self._last_complete = bool(msg.complete)
        self._publish_path_markers(
            self._last_points,
            active_waypoint_index=self._last_active_waypoint_index,
            complete=self._last_complete,
        )

    def _publish_path_markers(
        self,
        points: list[list[float]] | list,
        active_waypoint_index: int = 0,
        complete: bool = False,
    ) -> None:
        if points:
            self.get_logger().info(f'Publishing path markers: {len(points)} points (path finder -> RViz/Gazebo)')
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        header = Header(frame_id='map', stamp=stamp)

        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        if not points:
            self.pub_path_markers.publish(marker_array)
            return

        n = len(points)
        if complete:
            active_idx = n
        else:
            active_idx = max(0, min(int(active_waypoint_index), n - 1))

        ros_points: list[Point] = []
        for p in points:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            ros_points.append(pt)

        # "Glow" effect in RViz: wide translucent line behind the core path.
        def _make_line(marker_id: int, ns: str, pts: list[Point], width: float, color: ColorRGBA) -> Marker:
            m = Marker()
            m.header = header
            m.ns = ns
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color = color
            m.points = pts
            return m

        completed_pts = ros_points[: max(0, min(active_idx + 1, n))]
        remaining_pts = ros_points[max(0, min(active_idx, n - 1)) :] if n >= 2 else ros_points

        if len(completed_pts) >= 2:
            marker_array.markers.append(
                _make_line(
                    0, 'planned_path_glow_done', completed_pts, 0.5,
                    ColorRGBA(r=0.10, g=0.95, b=0.65, a=0.25),
                )
            )
            marker_array.markers.append(
                _make_line(
                    1, 'planned_path_done', completed_pts, 0.2,
                    ColorRGBA(r=0.05, g=0.95, b=0.55, a=0.95),
                )
            )

        if len(remaining_pts) >= 2:
            marker_array.markers.append(
                _make_line(
                    2, 'planned_path_glow_remaining', remaining_pts, 0.5,
                    ColorRGBA(r=1.00, g=0.75, b=0.05, a=0.22),
                )
            )
            marker_array.markers.append(
                _make_line(
                    3, 'planned_path_remaining', remaining_pts, 0.2,
                    ColorRGBA(r=1.00, g=0.55, b=0.05, a=0.98),
                )
            )

        for i, p in enumerate(points):
            dot = Marker()
            dot.header = header
            dot.ns = 'planned_waypoints'
            dot.id = i + 1
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = float(p[0])
            dot.pose.position.y = float(p[1])
            dot.pose.position.z = float(p[2])
            dot.pose.orientation.w = 1.0
            dot.scale.x = 0.4
            dot.scale.y = 0.4
            dot.scale.z = 0.4
            if i == 0:
                dot.color = ColorRGBA(r=0.15, g=0.9, b=0.2, a=0.95)
            elif i == len(points) - 1:
                dot.color = ColorRGBA(r=0.95, g=0.1, b=0.15, a=0.95)
            elif i < active_idx:
                dot.color = ColorRGBA(r=0.05, g=0.85, b=0.75, a=0.9)
            elif i == active_idx and not complete:
                dot.scale.x = 0.6
                dot.scale.y = 0.6
                dot.scale.z = 0.6
                dot.color = ColorRGBA(r=1.0, g=1.0, b=0.9, a=1.0)
            else:
                dot.color = ColorRGBA(r=1.0, g=0.7, b=0.1, a=0.85)
            marker_array.markers.append(dot)

        self.pub_path_markers.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
