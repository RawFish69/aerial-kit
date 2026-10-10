# uav_control

Predictive mission tracking for the air unit. `mpc_tracker_node` flies a
`uav_msgs/Trajectory` with either controller from `aerial_kit.controllers`:

| `controller` | What it is | Use it when |
|---|---|---|
| `mpc` | `ConstrainedMPC`: a QP over the horizon with acceleration and per-axis speed limits as hard constraints, solved by ADMM with warm start | the default; deterministic, smooth (`r_delta`) |
| `mppi` | `MPPI`: warm-started sampling controller; obstacles (`obstacle_spheres`) and a floor (`min_altitude_m`) are part of the cost | you need non-convex costs the QP can't express |

It is a drop-in alternative to `air_unit`'s `mission_executor_node` for offboard
missions. The topics are the same:

| | Topic | Type |
|---|---|---|
| in | `/uav/mission` | `uav_msgs/Trajectory` |
| in | `/uav/command` | `uav_msgs/Command` (tracks only in `MODE_MISSION`, pauses on manual override) |
| in | `/uav/backend/telemetry_raw` | `uav_msgs/Telemetry` |
| out | `/uav/internal/mission_cmd_vel` | `geometry_msgs/Twist` velocity setpoint, arbitrated by `command_manager_node` |
| out | `/uav/mission_status` | `uav_msgs/MissionStatus` |
| out | `/uav/control/predicted_path` | `nav_msgs/Path`: the plan, for RViz |

It publishes to the executor's command topic, so **run one or the other**, not both.

## How it flies a mission

- Waypoints are split into legs that end at a *stop*: a waypoint with
  `hold_time_sec > 0`, or the last one. Within a leg the intermediate waypoints
  are flown through, not stopped at. A `PathReference` moves along the leg at
  cruise speed (or the waypoint's `desired_speed_mps`) and decelerates into the stop.
- The controller plans accelerations over `horizon x plan_dt`. The backends
  take velocity setpoints and run their own velocity loop, so the node sends
  `v + a * velocity_loop_tau_s`, the setpoint that a first-order loop with that
  time constant turns back into the planned acceleration, clamped to
  `max_xy_speed_mps` / `max_z_speed_mps`. **Set `velocity_loop_tau_s` to match
  your backend**: about 0.67 s for `sim_fast` (its velocity gain is 1.5 /s). If
  it is far too small, the aircraft gets only a fraction of each planned
  acceleration and creeps.
- A stop is reached inside its acceptance radius *and* below `settle_speed_mps`;
  then it holds, and moves on.
- Heading follows the plan's xy displacement over the horizon, using the same
  `nose_axis` / `command_frame: body` conventions as the executor.
- Onboard planning (`PLANNING_ONBOARD`) is still the executor's job.

## Minimum-snap reference

`reference: min_snap` makes each leg a minimum-snap trajectory through its waypoints
(`aerial_kit.trajectory.MinSnapTrajectory`), timed to the cruise speed,
`min_snap_accel_mps2` and `min_snap_jerk_mps3`. The controller tracks the trajectory's
positions and velocities over its horizon, not a polyline at constant speed.

- The trajectory clock is governed. It runs at full rate while the aircraft is within
  `min_snap_slow_error_m` of the reference, and slows to a stop at `min_snap_stop_error_m`.
  A gust or a sluggish velocity loop delays the mission; it doesn't leave the aircraft
  chasing a reference that has run off. Pauses don't advance the clock.
- A replanned route starts at the aircraft's current velocity.
- Progress, blockage and arrival are still judged on the leg's polyline, so holds,
  acceptance radii and replanning work the same in both modes.

On the square test mission with the MPC, min-snap lowers peak acceleration from 2.7 to
0.8 m/s² and RMS jerk about 9×, and takes 19.5 s instead of 14 s. To trade some of the
smoothness back for speed, raise `min_snap_accel_mps2` and `min_snap_jerk_mps3`. Use
`path` when time matters more than smoothness.

## Obstacles and replanning

The node subscribes to `obstacle_topic` (default `/terrain/obstacles`, the terrain
generator's `MarkerArray`). Trees are vertical cylinders, rocks are boxes, and a `DELETEALL`
clears the set.

- **MPPI** flies around obstacles itself. They're part of its cost as signed distances,
  and it only sees the ones within reach of its horizon. It reports itself blocked only
  when it makes no progress (`stall_progress_m` in `stall_time_s`), as in front of a wall.
- **MPC** can't avoid anything. If the path within `blocked_lookahead_m` runs into an
  obstacle, it stops short and reports `blocked: path blocked at (x, y, z)`.
- **When blocked**, the node calls `planner_service` (default `/uav/planner/plan_path`) for
  a new route from its current position to the mission goal. While the request is in
  flight it holds position with status `replanning`. Requests are rate-limited by
  `replan_min_interval_s`; `replan_on_block: false` turns this off.

The planner server plans around the same published obstacles (`obstacle_source: auto`).
Before that change it generated its own random forest, so its paths ran through trees that
the tracker, and RViz, could see.

## Run

```bash
colcon build --packages-select uav_msgs uav_algorithms uav_control
ros2 launch uav_control mpc_tracker.launch.py                   # config/mpc_tracker.yaml
ros2 launch uav_control mpc_tracker.launch.py controller:=mppi  # config/mppi_tracker.yaml
```

Both sim bringups can start it in place of the executor:

```bash
ros2 launch sim_fast bringup.launch.py start_demo:=true mission_tracker:=mpc   # or mppi
ros2 launch sim_gazebo bringup.launch.py mission_tracker:=mpc                  # or mppi
```

`sim_fast` sets the frame and `velocity_loop_tau_s` for its backend. In Gazebo the node's
defaults (body frame, nose +Y, forward-only) match the X3 model. Both have been flown
end to end under ROS 2 Jazzy and Gazebo Harmonic.

`aerial_kit` must be importable, either from `pip install -e .` at the repo root
or found through `uav_algorithms.repo_paths` (set `AERIAL_KIT_REPO_ROOT` if the
install tree is outside the repo).

## Test

The tracking logic is in `uav_control/tracking_core.py`, which doesn't depend on
rclpy. `test_tracking_core.py` flies it against a velocity-controlled plant,
with no ROS needed. `test_ros_sim_fast.py` flies the real node in a ROS 2 graph
with the `sim_fast` backend, telemetry adapter and command manager. It skips
itself when rclpy isn't importable, and CI runs it in `ros:jazzy`.

```bash
cd ros2_ws/src/uav_control && python -m pytest test -q
```

The controllers have their own tests in `sim_py/tests/test_predictive_controllers.py`.
