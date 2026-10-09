# Examples: terrain, planners and controllers

Worked examples, from a single Python command to a full ROS 2 mission. Each section is
self-contained. The [software guide](SOFTWARE_GUIDE.md) explains the pieces.

> These examples replace an earlier version that launched the removed `sim_dyn` /
> `controllers_*` ROS 2 packages. The same scenarios (forest, mountains, plains; PID, LQR,
> MPC) now run in the standalone simulator and in the `sim_fast` / `sim_gazebo` stacks.

---

## 1. Forest terrain, RRT*, constrained MPC (Python only)

```bash
python -m pip install -e ".[sim]"
./scripts/run_sim_mpc.sh
```

This calls:

```bash
python -m aerial_kit.sim.cli --config sim_py/sim_config.yaml \
    --controller constrained_mpc --terrain forest --planner rrtstar
```

What happens:

1. The terrain generator places a forest from
   `ros2_ws/src/terrain_generator/config/terrain_params.yaml`. It's the same generator and
   config the ROS stack uses.
2. RRT* plans a path from the start to the goal (`path.*` in `sim_config.yaml`), inflated by
   `path.collision_inflation`.
3. The controller tracks the waypoints on the point-mass backend.
4. A 3D plot shows the trees, the planned path (dashed) and the flown path. The terminal
   prints the goal error and the number of collisions.

Headless, saving the plot:

```bash
./scripts/run_sim_mpc.sh headless            # -> results/constrained_mpc.png
```

The forest and goal altitude are randomised on each run (`end_relative_z: "auto"`). Compare
controllers by running each a few times, not once.

### Other terrains and controllers

```bash
./scripts/run_terrain_sim.sh pid mountains        # controller, terrain, [headless]
./scripts/run_terrain_sim.sh mppi plains true
TERRAIN=mountains PLANNER=astar ./scripts/run_sim.sh lqr
```

| Script argument | Controller |
|---|---|
| `pid`, `lqr` | PD, LQR |
| `mpc` | `constrained_mpc`: QP with acceleration and speed limits |
| `mppi` | `warm_mppi`: sampling, with a persistent plan |
| any registered name | e.g. `./scripts/run_sim.sh mpc` for the unconstrained finite-horizon LQ |

Swap the dynamics for a 6-DOF multirotor:

```bash
python -m aerial_kit.sim.cli --config sim_py/sim_config.yaml \
    --controller constrained_mpc --backend multirotor --terrain plains
```

---

## 2. Tuning a controller in the config

Each controller reads `controller.<name>` from the YAML. In a copy of
`sim_py/sim_config.yaml`:

```yaml
controller:
  controller_type: constrained_mpc
  constrained_mpc:
    horizon: 25
    max_speed_xy: 4.0
    speed_limit_xy: disc     # bound |v_xy|, not |vx| and |vy| separately
    r_delta: 5.0             # smoother commands
  warm_mppi:
    samples: 768
    min_altitude: 2.0
    obstacles:
      - {center: [60, 50, 10], radius: 4.0}
```

```bash
python -m aerial_kit.sim.cli --config my_config.yaml --terrain forest
```

What the knobs do:

- `q_pos` / `q_vel` against `r_acc`: tracking against effort.
- `r_delta`: penalises changes in acceleration (jerk). Raise it if the command chatters.
- `horizon x dt`: how far ahead it plans. It should cover the stopping distance at full
  speed.
- MPPI `temperature` is relative to the spread of rollout costs. Lower is greedier.
  Around 0.05-0.2 averages tens of rollouts; much lower degenerates to the single best
  noisy rollout.

---

## 3. The controllers as a library

An MPPI flying around a sphere that sits on the straight line to its goal:

```python
import numpy as np
from aerial_kit.controllers import MPPI, SphereObstacle

mppi = MPPI(obstacles=[SphereObstacle((5.0, 0.0, 2.0), 1.5)], obstacle_margin=0.5, seed=0)
p, v, goal = np.array([0.0, 0.0, 2.0]), np.zeros(3), np.array([10.0, 0.0, 2.0])

for step in range(120):                 # 12 s at the plan step of 0.1 s
    a = mppi.solve(p, v, goal).accel    # world frame, z up
    v = v + a * mppi.dt
    p = p + v * mppi.dt
print(p.round(2))                       # close to the goal, without entering the sphere
```

The tests in `sim_py/tests/test_predictive_controllers.py` are more examples of the same
API: path references, speed limits, warm starts and obstacle avoidance.

---

## 4. A planned mission in ROS 2, headless

No Gazebo needed: `sim_fast` is a point-mass backend behind the same `/uav/*` topics.

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# Offboard A* through the forest, then the demo mission, with the predictive tracker:
ros2 launch sim_fast bringup.launch.py start_demo:=true mission_tracker:=mpc
```

The demo node:

1. waits for telemetry,
2. asks the planner service for a path to a goal 170 m away (86 waypoints on the default
   forest),
3. arms and takes off, holds a hover,
4. switches to mission mode.

Follow it from another terminal:

```bash
ros2 topic echo /uav/mission_status --field status_text   # "tracking wp 41/86 ..."
ros2 topic echo /uav/telemetry --field pose.position
```

Swap `mission_tracker:=mppi`, or `executor` for the original P-controller. On this demo,
both predictive trackers finish the mission in about 135 s, against about 210 s for the
executor. They fly through intermediate waypoints instead of stopping at each one.

Onboard planning (the air unit plans):

```bash
ros2 launch sim_fast bringup.launch.py start_offboard_planner:=false \
    start_onboard_planner:=true start_demo:=true demo_planning_mode:=onboard
```

(Onboard planning needs `mission_tracker:=executor`.)

---

## 5. The same mission in Gazebo

Three terminals, each with the workspace sourced:

```bash
# 1. Gazebo, air unit with the predictive tracker, path visuals
ros2 launch sim_gazebo bringup.launch.py mission_tracker:=mpc start_path_visuals:=true

# 2. Terrain and ground station with the offboard planner
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=forest
ros2 launch ground_station ground.launch.py start_planner:=true start_monitor:=true

# 3. Fly the demo mission
ros2 run ground_station ground_station_demo_mission
```

In RViz, add:

- `Path` on `/uav/control/predicted_path`: the tracker's plan, refreshed every tick
- `MarkerArray` on `/gs/planner/planned_path_markers`: the planner's path
- `MarkerArray` on `/terrain/obstacles`: the trees

The tracker's parameters come from `uav_control/config/mpc_tracker.yaml`. Pass your own with
`tracker_params_file:=/path/to/file.yaml`. `velocity_loop_tau_s` should match how
quickly the Gazebo model's velocity controller follows a setpoint.

More Gazebo flows (dense forest, mountain surface, the custom `lr_drone` model) are in
[ros2_ws/README.md](../ros2_ws/README.md).

---

## 6. When something doesn't fly

| Symptom | Check |
|---|---|
| Collisions in the Python sim | Is the goal itself inside an obstacle? The log says "Planned path has collisions!". Raise `path.collision_inflation`, or move the goal |
| Mission never starts (ROS) | `ros2 topic echo /uav/telemetry --field status_text`: is it in mission mode, armed, not on manual override? |
| Two followers fighting | `ros2 node list` should show only one of `mission_executor_node` and `mpc_tracker_node` |
| Tracker creeps to waypoints | `velocity_loop_tau_s` is too small for the backend |
| Where does a command stop? | `./scripts/debug_topics.sh` follows it hop by hop |

See also [Troubleshooting](SOFTWARE_GUIDE.md#troubleshooting).
