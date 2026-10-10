# aerial-kit - Software Guide

How the software side of the stack fits together, and how to run, tune and test it:
the Python control library and simulator, the ROS 2 workspace, and where the
hardware backends plug in.

> **Moved on from the first ROS 2 prototype.** Earlier versions of this guide described
> C++ `controllers_pid` / `controllers_lqr` / `controllers_mpc` packages, a `sim_dyn`
> simulator, a `safety_gate` node and `/cmd/body_rate_thrust` topics. Those were removed
> when the workspace was consolidated. The current stack is described below; the old one
> is in git history (`git log -- ros2_ws/src/controllers_mpc`).

For airframe support, Docker images and the package table, see the
[stack guide](STACK-GUIDE.md). For worked examples, see [EXAMPLE_USAGE.md](EXAMPLE_USAGE.md).

---

## Contents

1. [Layout](#layout)
2. [Python: control library and simulator](#python-control-library-and-simulator)
3. [Controllers](#controllers)
4. [ROS 2 workspace](#ros-2-workspace)
5. [Mission followers: executor and predictive tracker](#mission-followers-executor-and-predictive-tracker)
6. [Hardware backends](#hardware-backends)
7. [Testing and CI](#testing-and-ci)
8. [Monitoring and debugging](#monitoring-and-debugging)
9. [Configuration reference](#configuration-reference)
10. [Troubleshooting](#troubleshooting)

---

## Layout

```
aerial_kit/        ROS-free control library (pip install aerial-kit): airframes,
                   dynamics, controllers, guidance, simulator API
sim_py/            Standalone simulator: planners, backends, visualizer, tests
ros2_ws/src/       ROS 2 packages (uav_msgs, air_unit, uav_control, sim_fast,
                   sim_gazebo, sim_bridge, planner, ground_station, hw_bridge,
                   mavlink_bridge, terrain_generator, uav_algorithms)
scripts/           Run / build / diagnostic helpers
examples/          Runnable quadrotor, fixed-wing and firmware examples
firmware/          AerialKit flight controller firmware and legacy radio projects
```

Two ways to fly the same algorithms:

```
Python only:   planner -> controller -> dynamics backend -> Matplotlib 3D
ROS 2:         ground station -> /uav/mission -> mission follower -> command manager
               -> backend (sim_fast | Gazebo | CRSF/Betaflight | MAVLink/PX4/ArduPilot)
```

---

## Python: control library and simulator

### Install

```bash
python -m pip install -e ".[sim]"          # numpy, scipy, matplotlib, pyyaml
python -m pip install -e ".[rotorpy]"      # optional RotorPy backend
```

Python 3.10-3.12 are tested in CI.

### Run

```bash
# Bundled examples
python examples/quadrotor/run.py
python examples/fixed_wing/run.py --no-show --save fixed-wing.png

# The simulator CLI (also installed as `aerial-kit-sim`)
python -m aerial_kit.sim.cli --controller constrained_mpc --no-show
python -m aerial_kit.sim.cli --config sim_py/sim_config.yaml \
    --controller warm_mppi --terrain forest --planner rrtstar

# Keyboard teleop
python examples/quadrotor/teleop.py
```

Options of `aerial_kit.sim.cli`:

| Flag | Values |
|---|---|
| `--config` / `--example` | a YAML file, or `quadrotor` / `fixed-wing` |
| `--controller` | `pid`, `lqr`, `mpc`, `mppi`, `constrained_mpc`, `warm_mppi`, `l1_tecs` |
| `--planner` | `straight`, `astar`, `rrt`, `rrtstar`, `dubins` |
| `--backend` | `pointmass`, `multirotor`, `rotorpy`, `mujoco`, `fixedwing` |
| `--airframe` | `quad`, `hex`, `octo`, `twin_wing` |
| `--terrain` | `forest`, `mountains`, `plains` |
| `--sim-time`, `--dt` | override the config |
| `--no-show`, `--save PATH` | headless runs |

The source-tree entry point `python -m sim_py.run_sim` takes the same component names
and reads `sim_py/sim_config.yaml` by default.

The helper scripts wrap the CLI with forest terrain and RRT* by default:

```bash
./scripts/run_sim_pid.sh               # also run_sim_lqr.sh, run_sim_mpc.sh, run_sim_mppi.sh
./scripts/run_sim_mpc.sh headless      # no window; saves results/constrained_mpc.png
TERRAIN=mountains ./scripts/run_sim.sh warm_mppi
./scripts/run_terrain_sim.sh mpc mountains true
```

### Use the library directly

```python
import numpy as np
from aerial_kit.controllers import ConstrainedMPC, MPPI, PathReference, SphereObstacle

path = PathReference(np.array([[0, 0, 2], [10, 0, 2], [10, 10, 3]]), decel_mps2=1.5)
mpc = ConstrainedMPC(max_speed_xy=2.0, speed_limit_xy="disc", max_accel_xy=3.0)

p, v = np.array([0.0, 0.0, 2.0]), np.zeros(3)
ref = path.sample(path.project(p), cruise_mps=2.0, dt=mpc.dt, horizon=mpc.horizon)
plan = mpc.solve(p, v, ref)
plan.accel                  # world-frame acceleration to apply now (z up)
plan.predicted_positions    # (N, 3) the plan behind it
```

---

## Controllers

All position controllers produce a world-frame acceleration demand (`ControlTarget.accel_cmd`,
z up, gravity excluded). Each reads its settings from `controller.<name>` in the
simulator config.

| Name | Class | What it is |
|---|---|---|
| `pid` | `PIDController` | PD on position |
| `lqr` | `LQRController` | infinite-horizon LQR per axis |
| `mpc` | `MPCController` | finite-horizon discrete LQ (Riccati); output clipped afterwards |
| `mppi` | `MPPIController` | stateless sampling MPPI (kept for comparison) |
| `constrained_mpc` | `ConstrainedMPCController` -> `ConstrainedMPC` | QP MPC: acceleration and speed limits inside the optimisation, horizon reference, smoothing term; solved by ADMM (`BoxQP`) with warm start |
| `warm_mppi` | `WarmMPPIController` -> `MPPI` | vectorised MPPI with a persistent nominal plan; sphere obstacles, floor and speed costs |
| `l1_tecs` | `FixedWingL1TECSController` | fixed wing: L1 lateral guidance + TECS, produces a wrench |
| (actuator level) | `GeometricController` | SE(3) tracking (Lee et al.): position to moment on the rotation group, velocity/acceleration/jerk feedforward, recovers from large attitudes |
| (actuator level) | `NMPCController` | iLQR nonlinear MPC on the full model (thrust + body rates, rate-loop lag modelled), soft tilt limit, attitude-tracking rate loop |

### `constrained_mpc`

```yaml
controller:
  constrained_mpc:
    dt: 0.1              # plan step [s]; the wrapper holds each command for one step
    horizon: 20
    q_pos: 8.0
    q_vel: 1.0
    r_acc: 0.5
    r_delta: 0.0         # penalty on step-to-step acceleration change (smoothness)
    terminal: dare       # dare (infinite-horizon cost-to-go) | stage
    max_accel_xy: 6.0
    max_accel_z: 4.0
    max_speed_xy: 3.0    # optional
    max_speed_z: 1.5     # optional
    speed_limit_xy: box  # box: |vx|,|vy| <= limit; disc: |v_xy| <= limit (octagon)
```

- With the limits inactive and `terminal: stage`, it computes exactly the same command as `mpc`. A
  test checks this.
- `speed_limit_xy: box` lets a diagonal reach 1.41x the limit. `disc` bounds the norm, by
  an inscribed polygon (`disc_sides`, default 8), so it gives away at most 8 % along the flats.
- If the aircraft is already above a speed limit, the bound relaxes to what 90 % braking
  can reach, so the QP stays feasible from any state.

### `warm_mppi`

```yaml
controller:
  warm_mppi:
    dt: 0.1
    horizon: 20
    samples: 512
    temperature: 0.1             # relative: lambda = temperature * (median - min) cost
    temperature_mode: relative   # or absolute
    noise_std: 2.0
    q_pos: 8.0
    q_terminal: 20.0
    max_speed: 3.0
    min_altitude: 1.0
    obstacles:
      - {center: [50, 40, 10], radius: 3.0}
    obstacle_margin: 0.5
    seed: 0
```

Both wrappers solve once per plan step and hold the command between solves, because the
simulator calls `compute` every integration step. A change of target triggers an immediate
re-solve.

### Attitude-level controllers

`GeometricController` and `NMPCController` return a body wrench (thrust and moment), not an
acceleration. Fly them on the motor-level plant with `aerial_kit.dynamics.actuator_loop.fly`,
as `CascadeController` is flown. Both take `mass_kg` explicitly (it sets the hover thrust),
and both accept an optional `reference: t -> FlatReference` for trajectory tracking:

```python
from aerial_kit.controllers import GeometricController, GeometricGains, NMPCController, NMPCGains
from aerial_kit.dynamics.actuator_loop import fly

geo = GeometricController(GeometricGains(mass_kg=1.0))
mpc = NMPCController(NMPCGains(mass_kg=1.0, max_tilt_deg=45.0))
trace = fly(mpc, airframe, plant, target_position, steps=3000, dt=0.002)
```

On the test airframe (1 kg quad, 30 ms motor lag), for a 4.2 m step, the time to within
10 cm is 2.4 s for the geometric controller (39 degrees peak tilt) and 1.6 s for the NMPC
(53 degrees). Both recover from 150 degrees of roll; the geometric one loses 2.5 m of
altitude, the cascade 7 m.

---

## ROS 2 workspace

### Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash      # or humble
colcon build --symlink-install
source install/setup.bash
```

`./scripts/build.sh` does the same. ROS packages import `aerial_kit` from the repo. Run
from inside the repo, install it with `pip install -e .`, or set `AERIAL_KIT_REPO_ROOT`.

### Topics

| Topic | Type | From -> to |
|---|---|---|
| `/uav/command` | `uav_msgs/Command` | ground station -> command manager, mission follower |
| `/uav/mission` | `uav_msgs/Trajectory` | ground station / planner -> mission follower |
| `/uav/internal/mission_cmd_vel` | `geometry_msgs/Twist` | mission follower -> command manager |
| `/uav/mission_status` | `uav_msgs/MissionStatus` | mission follower -> ground station |
| `/uav/backend/cmd_twist`, `/uav/backend/enable` | `Twist`, `Bool` | command manager -> backend |
| `/uav/backend/odom` | `nav_msgs/Odometry` | backend / estimator -> telemetry adapter |
| `/uav/backend/telemetry_raw` | `uav_msgs/Telemetry` | telemetry adapter -> command manager, follower |
| `/uav/telemetry` | `uav_msgs/Telemetry` | command manager -> ground station |
| `/uav/control/predicted_path` | `nav_msgs/Path` | predictive tracker -> RViz |

The command manager owns the flight modes: takeoff, hover hold, mission, land and RTL. In
mission mode it forwards the follower's velocity setpoint to the backend.

### Simulators

```bash
# Headless point-mass backend: no Gazebo needed
ros2 launch sim_fast bringup.launch.py start_demo:=true
ros2 launch sim_fast bringup.launch.py start_demo:=true mission_tracker:=mpc

# Gazebo (see ros2_ws/README.md for the forest / mountains demos)
ros2 launch sim_gazebo bringup.launch.py
ros2 launch sim_gazebo bringup.launch.py mission_tracker:=mppi
ros2 launch ground_station ground.launch.py start_planner:=true
ros2 run ground_station ground_station_demo_mission
```

---

## Mission followers: executor and predictive tracker

Two nodes can follow `/uav/mission`. They publish to the same topic, so run only one; both
bringups pick it with `mission_tracker:=executor|mpc|mppi`.

| | `air_unit/mission_executor_node` | `uav_control/mpc_tracker_node` |
|---|---|---|
| Law | P on position to the active waypoint, speed and slew limits | `ConstrainedMPC` or `MPPI` over a horizon |
| Waypoints | stops at each one | flies through intermediate ones; stops at holds and at the end |
| Limits | clamps after the fact | accel and speed limits inside the plan |
| Obstacles | from the planner's path only | MPPI: `obstacle_spheres`, `min_altitude_m` |
| Onboard planning | yes (`PLANNING_ONBOARD`) | no; it flies the trajectory it is given |

The tracker turns a planned acceleration into the velocity setpoint the backend's own
velocity loop needs: `v + a * velocity_loop_tau_s`. Set `velocity_loop_tau_s` to match
the backend; `sim_fast`'s is 0.67 s, and its launch sets it. More detail is in
[uav_control/README.md](../ros2_ws/src/uav_control/README.md).

Frames: Gazebo's models fly nose along body +Y and take body-frame twists, so both
followers default to `command_frame: body`, `nose_axis: +y`. `sim_fast` takes world-frame
velocity and has no yaw, so its launch runs them with `command_frame: world` and heading
control off.

---

## Hardware backends

| Autopilot | Package | Launch |
|---|---|---|
| Betaflight over CRSF (ESP-NOW TX) | `hw_bridge` | `ros2 launch hw_bridge hw_crsf.launch.py udp_host:=192.168.4.1` |
| PX4 / ArduPilot over MAVLink | `mavlink_bridge` | `ros2 launch mavlink_bridge real_hardware.launch.py` |

Wiring, the TX UDP receiver and the packet format are in [HARDWARE.md](HARDWARE.md). MAVLink
setup is in [mavlink_bridge/README.md](../ros2_ws/src/mavlink_bridge/README.md).

`hw_bridge` notes. These changed recently; check your configs:

- `hw_state_estimator_node` runs an **INS EKF** by default (`estimator: ekf`, from
  `aerial_kit.estimation`). It estimates position, velocity, accelerometer bias and baro offset,
  rejects GPS outliers with a chi-square gate, and publishes covariances. Set
  `imu_accel_mode` to match your FC: the default `none` is correct for Betaflight over CRSF
  (attitude only); use `body_specific_force` for a REP-145 IMU. `estimator: complementary`
  keeps the older baro filter (`baro_filter_hz`) and per-fix GPS differencing.
- `crsf_backend_adapter_node` maps the velocity **error** (demand minus measured) to sticks,
  because in Angle mode the sticks command acceleration. `vz_feedback` is on by default.
  `vxy_feedback` (GPS-derived) is opt-in until you have checked its velocity is smooth on
  your vehicle. Without fresh odometry it falls back to the open-loop mapping.
- Before flight, tune `hover_throttle`, `kz` and `kv_xy` per airframe, and keep Betaflight in
  Angle mode.

---

## Testing and CI

```bash
# Library + simulator
python -m pytest sim_py/tests -q

# ROS packages' logic, no ROS needed (run from each package directory)
cd ros2_ws/src/uav_control && python -m pytest test -q
cd ros2_ws/src/hw_bridge   && python -m pytest test -q
cd ros2_ws/src/air_unit    && python -m pytest test -q

# With a sourced, built workspace, the same commands also run the rclpy tests:
#   hw_bridge/test/test_sitl_closed_loop.py: fake FC -> estimator -> CRSF adapter ->
#     command manager -> executor: takeoff, hover, land, mission, RTL
#   uav_control/test/test_ros_sim_fast.py: tracker missions against sim_fast
```

`.github/workflows/python-tests.yml` runs on every pull request:

- the simulator suite on Python 3.10 and 3.12
- the ROS-free package tests
- a `ros:jazzy` job that runs both rclpy integration tests

In-process ROS tests use a `SingleThreadedExecutor`. With several executor threads in one
process, the nodes starve each other of the GIL. A launch runs each node in its own
process, so this doesn't apply there.

---

## Monitoring and debugging

```bash
./scripts/check_sim.sh              # nodes, which follower is running, topic samples
./scripts/debug_topics.sh           # follows a command hop by hop; the first silent hop is the break
./scripts/check_ros2_v2_topics.sh   # /uav/* topic samples

ros2 topic echo /uav/mission_status
ros2 topic echo /uav/telemetry --field status_text
ros2 topic hz /uav/backend/odom
```

In RViz, add `Path` on `/uav/control/predicted_path` to see the tracker's plan, and
`MarkerArray` on `/gs/planner/planned_path_markers` and `/terrain/obstacles`.

---

## Configuration reference

| What | Where |
|---|---|
| Python sim scenario, controllers | `sim_py/sim_config.yaml`, `examples/*/config.yaml`, `aerial_kit/sim/defaults/*.yaml` |
| Terrain (shared with ROS) | `ros2_ws/src/terrain_generator/config/terrain_params.yaml` |
| Predictive tracker | `ros2_ws/src/uav_control/config/mpc_tracker.yaml`, `mppi_tracker.yaml` |
| Executor / command manager (Gazebo) | `ros2_ws/src/sim_gazebo/config/mission_executor_safe_tracking.yaml`, `command_manager_safe_custom.yaml` |
| CRSF adapter, estimator | `ros2_ws/src/hw_bridge/config/crsf_adapter.yaml`, `hw_estimator.yaml` |
| MAVLink | `ros2_ws/src/mavlink_bridge/config/mavlink_bridge_default.yaml` |

---

## Troubleshooting

**`ModuleNotFoundError: aerial_kit` in a ROS node.** Run from inside the repo,
`pip install -e .` the repo, or `export AERIAL_KIT_REPO_ROOT=/path/to/aerial-kit`.

**`rclpy._rclpy_pybind11` not found, or colcon fails with `install_layout`.** The
`python3` on `PATH` is not the one your ROS distro was built for (Jazzy: 3.12, Humble: 3.10).
Put the distro's interpreter first on `PATH` before sourcing and building.

**`No module named 'matplotlib.tri.triangulation'`.** An apt `python3-matplotlib` is
shadowing a pip-installed matplotlib's `mpl_toolkits`. Use one or the other.

**The aircraft creeps to waypoints with the predictive tracker.** `velocity_loop_tau_s` is
much smaller than the backend's velocity-loop time constant, so the aircraft gets only a
fraction of each planned acceleration.

**The mission doesn't start.** The follower acts only in `MODE_MISSION` and pauses on manual
override. Check `/uav/telemetry` `status_text`, and that only one follower is running.

**It doesn't move in `sim_fast`.** Body-frame, forward-only commands with no yaw in the
backend; use the provided launch, which sets `command_frame: world`.

**`gz: command not found`, or Gazebo topics missing.** See the troubleshooting section of
[ros2_ws/README.md](../ros2_ws/README.md).
