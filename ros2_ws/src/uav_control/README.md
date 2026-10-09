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
  take velocity setpoints, so the node sends the plan's predicted velocity
  `velocity_lead_steps` ahead, clamped to `max_xy_speed_mps` / `max_z_speed_mps`.
- A stop is reached inside its acceptance radius *and* below `settle_speed_mps`;
  then it holds, and moves on.
- Heading follows the plan's xy displacement over the horizon, using the same
  `nose_axis` / `command_frame: body` conventions as the executor.
- Onboard planning (`PLANNING_ONBOARD`) is still the executor's job.

## Run

```bash
colcon build --packages-select uav_msgs uav_algorithms uav_control
ros2 launch uav_control mpc_tracker.launch.py                   # config/mpc_tracker.yaml
ros2 launch uav_control mpc_tracker.launch.py controller:=mppi  # config/mppi_tracker.yaml
```

`aerial_kit` must be importable, either from `pip install -e .` at the repo root
or found through `uav_algorithms.repo_paths` (set `AERIAL_KIT_REPO_ROOT` if the
install tree is outside the repo).

## Test

The tracking logic is in `uav_control/tracking_core.py`, which doesn't depend on
rclpy. The tests fly it against a velocity-controlled plant, with no ROS needed:

```bash
cd ros2_ws/src/uav_control && python -m pytest test -q
```

The controllers have their own tests in `sim_py/tests/test_predictive_controllers.py`.
