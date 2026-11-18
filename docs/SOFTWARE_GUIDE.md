# aerial-kit - Software Guide

How to run, tune and test the software side of the stack: the Python control library and
simulator, the MuJoCo vehicles, the ROS 2 workspace, and where the hardware backends plug
in. For airframe status, architecture and Docker, see the [stack guide](STACK-GUIDE.md).
For worked terrain/planner/controller examples, see [EXAMPLE_USAGE.md](EXAMPLE_USAGE.md).

---

## Contents

1. [Layout](#layout)
2. [Vehicles: how to run each](#vehicles-how-to-run-each)
3. [Python: control library and simulator](#python-control-library-and-simulator)
4. [Controllers](#controllers)
5. [ROS 2 workspace](#ros-2-workspace)
6. [Mission followers: executor and predictive tracker](#mission-followers-executor-and-predictive-tracker)
7. [Hardware backends](#hardware-backends)
8. [Testing](#testing)
9. [Monitoring and debugging](#monitoring-and-debugging)
10. [Configuration reference](#configuration-reference)
11. [Troubleshooting](#troubleshooting)

---

## Layout

```
aerial_kit/        ROS-free control library (pip install aerial-kit): airframes and their
                   specs, dynamics, controllers, guidance, estimation, simulator API
sim_py/            Standalone simulator: planners, backends (incl. MuJoCo), visualizer, tests
ros2_ws/src/       ROS 2 packages (uav_msgs, air_unit, uav_control, sim_fast,
                   sim_gazebo, sim_bridge, planner, ground_station, hw_bridge,
                   mavlink_bridge, terrain_generator, uav_algorithms)
scripts/           Run / build / test / render / asset-generation helpers
examples/          Runnable quadrotor, fixed-wing, single-wing and firmware examples
firmware/          AerialKit flight controller firmware and legacy radio projects
```

Two ways to fly the same algorithms:

```
Python only:   planner -> controller -> dynamics backend -> Matplotlib 3D (or MuJoCo)
ROS 2:         ground station -> /uav/mission -> mission follower -> command manager
               -> backend (sim_fast | MuJoCo | Gazebo | CRSF/Betaflight | MAVLink/PX4/ArduPilot)
```

---

## Vehicles: how to run each

Every vehicle below is simulation-only; nothing here claims flight readiness. Airframe
numbers live in one spec file per vehicle, and the MuJoCo and Gazebo models are generated
from it.

| Vehicle | Spec | Detail |
|---|---|---|
| Generic quad / hex / octo | `aerial_kit/airframes/multirotor.py` (ideal-polygon mixer) | this guide |
| 5" FPV quad `fpv_quad_5in` | `aerial_kit/airframes/multirotor_specs/fpv_quad_5in.yaml` | [MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md) |
| 3" micro long-range quad `micro_lr_quad` | `.../micro_lr_quad.yaml` (reads the `lr_drone` CAD) | [MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md) |
| Research hexacopter `research_hex` | `.../research_hex.yaml` | [MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md) |
| Research octocopter `research_octo` | `.../research_octo.yaml` | [MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md) |
| Swarm quad `swarm_quad`, N in one scene | `.../swarm_quad.yaml` | [MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md#swarm-n-drones-in-one-scene) |
| Twin-motor flying wing | `aerial_kit/airframes/twin_wing/twin_wing.yaml` | [TWIN_WING_AIRFRAME.md](TWIN_WING_AIRFRAME.md), [TWIN_WING_TELEOP.md](TWIN_WING_TELEOP.md) |
| Single-motor flying wing | `aerial_kit/airframes/single_wing/single_wing.yaml` | [SINGLE_WING.md](SINGLE_WING.md) |

### Python, no ROS

```bash
python examples/quadrotor/run.py                                       # generic quad, point mass
python examples/quadrotor/run.py --config examples/quadrotor/mujoco.yaml   # MuJoCo multirotor
python examples/fixed_wing/run.py                                      # twin wing, analytic 6-DOF
python examples/fixed_wing/run.py --config examples/fixed_wing/mujoco.yaml # twin wing, MuJoCo
python examples/single_wing/run.py                                     # single wing, analytic
# Add --no-show [--save out.png] to any of them for a headless run.
```

- MuJoCo multirotor: pick the airframe with `simulation.mujoco_multirotor.airframe` in
  `examples/quadrotor/mujoco.yaml` (`fpv_quad_5in`, `micro_lr_quad`, `research_hex`,
  `research_octo`). The cascade controller takes its mass, inertia and bandwidths from the
  spec.
- Single wing on MuJoCo: set `simulation.backend: mujoco_singlewing` in
  `examples/single_wing/config.yaml`.
- Matplotlib keyboard teleop (generic quad and twin wing): see [Keyboard teleop](#keyboard-teleop-matplotlib).

### ROS 2

Source the underlay and the workspace first (see [Build](#build)).

```bash
# Twin / single wing, autonomous three-waypoint mission (AirspeedNav, L1/TECS)
ros2 launch sim_fast fixed_wing_demo.launch.py                       # twin wing, analytic
ros2 launch sim_fast fixed_wing_mujoco_demo.launch.py \
    [airframe:=single_wing] [viewer:=true] [wind_x_mps:=3.0 wind_y_mps:=1.0] \
    [estimator_mode:=ekf estimator_seed:=7]
ros2 launch sim_gazebo fixed_wing_gazebo_demo.launch.py [headless:=true]   # twin wing, Gazebo visual

# Multirotors, autonomous planned mission (takeoff, fly, land, disarm)
ros2 launch sim_fast bringup.launch.py start_demo:=true               # original point mass
ros2 launch sim_fast bringup.launch.py start_demo:=true sim_backend:=mujoco \
    airframe:=research_hex [viewer:=true] [mission_tracker:=mpc|mppi] [wind_x_mps:=...]

# Swarm: N drones in one MuJoCo scene, formation demo (takeoff, circle, line, grid, land)
ros2 launch sim_fast swarm_mujoco.launch.py n:=12 demo:=true [viewer:=true]

# Keyboard flying (click the small teleop window, Enter to arm)
ros2 launch sim_fast fixed_wing_mujoco_teleop.launch.py [airframe:=single_wing] [start_z_m:=150.0]
ros2 launch sim_fast multirotor_mujoco_teleop.launch.py airframe:=fpv_quad_5in [mode:=velocity|angle|acro] \
    [flight_controller:=firmware]   # the real AK flight core flies it, see below
```

Teleop keys (both): `W/S` pitch / forward-back, `A/D` roll / left-right, `Space/Shift`
throttle or climb, `Q/E` yaw (differential thrust on the twin wing, nothing on the single
wing), `X` re-centre, `P` pause, `C` toggle the MuJoCo viewer between the chase view and
the on-board camera (FPV / nose / payload camera), `Backspace` disarm, `Esc` exit.
Multirotors also take `M` to cycle modes:

| Mode | Sticks mean | Default for |
|---|---|---|
| `velocity` | speed; centred holds position | research hex / octo |
| `angle` | tilt; centred is level and holds altitude | FPV and micro quads |
| `acro` | rotation rate, raw throttle; nothing self-levels (needs a joystick to be flyable) | - |

Safety behaviour (both teleops): manual commands only count while fresh (0.3 s), active
and finite; losing the teleop window's focus re-centres, and 3 s unfocused disarms.
Released or stale, a wing flies a safe cruise circle; a multirotor holds position, lands if
disarmed in the air, and idles on the ground. Closing the teleop window ends the session.
`viewer:=true` never blocks a headless run: without a display or OpenGL the plant keeps
flying.

### Flying the AK firmware itself (firmware-in-the-loop)

`sim_py/fc_sil/` compiles the unmodified AK flight core (`firmware/src/core/flight`,
`filter`, `sensors`) into a host library on first use (`python3 sim_py/fc_sil/build.py`,
needs `cc`) and runs it at 1 kHz against MuJoCo: the plant's state becomes an IMU sample in
the firmware's convention (FRD, rad/s, g with +1 on z when level), the sticks become receiver
counts (ch5 mode, ch6 arm, 172/992/1811), and the firmware's motor outputs drive the rotors
(thrust = T_max x u^2, no battery compensation). Arming, angle/acro, mixer, estimator and
failsafe are the firmware's own code. Quad-X airframes only (`fpv_quad_5in`, `micro_lr_quad`).

```bash
ros2 launch sim_fast multirotor_mujoco_teleop.launch.py airframe:=fpv_quad_5in flight_controller:=firmware
```

```python
from sim_py.fc_sil import FirmwareInTheLoop
sim = FirmwareInTheLoop("fpv_quad_5in")           # on the ground, disarmed
sim.set_sticks(arm=False); sim.step(0.5)          # arm switch must start off
sim.set_sticks(arm=True); sim.step(1.0)           # throttle low for the arm hold -> "armed"
sim.set_sticks(throttle=0.45, roll=0.2); sim.step(2.0)
print(sim.fc.state, sim.state.position, sim.fc.attitude())
```

Differences from `flight_controller:=sim`: Enter arms the firmware through its real
sequence (switch on, throttle held at minimum for 0.8 s, then the sticks pass through).
Throttle centre is hover thrust and full stick is 2x hover
(`firmware_max_thrust_ratio`). There is **no altitude hold**, so climb rate is yours.
`velocity` mode flies as angle, and `cmd_twist` is ignored. If the teleop goes quiet, the
receiver times out and the firmware's quad failsafe **cuts the motors and latches**, so the
aircraft falls, as the real one would. Tests: `sim_py/tests/test_fc_sil.py`,
`ros2_ws/src/sim_bridge/test/test_multirotor_adapter.py -k firmware`.

The wings too (Python): `WingFirmwareInTheLoop("twin_wing" | "single_wing")` runs the firmware's
elevon-wing mixers (airframes 1 and 7) against the MuJoCo wings. Like the real aircraft it is
armed in the hand and then thrown (`launch()`). Stick signs, arming and the lost-link
"circling down" failsafe pass, and with a gravity-only accelerometer it flies level. With the
physical accelerometer it dives: launch thrust tilts the measured gravity about 27 deg, and the
fixed-wing estimator trusts it (`docs/evidence/fc-sil-wing-estimator-finding-2026-11-18.md`).

Through the radio, end to end: `ros2 launch sim_fast bringup.launch.py sim_backend:=mujoco radio_sim:=true
flight_controller:=firmware start_demo:=true` (or `FLIGHT_CONTROLLER=firmware scripts/run_radio_sil_mission.sh`).
The host radio backend arms the firmware through CRSF, the simulated ELRS link carries the frames, and the
adapter feeds the receiver bytes to the firmware at their arrival times; the firmware alone handles
failsafe. The host closes velocity and altitude over telemetry with gains from
`sim_py.fc_sil.radio_stick_settings`. It flies without flipping but does **not** finish the demo
mission yet: the host's altitude loop runs through ~4 s of telemetry/EKF delay and oscillates.
See `docs/evidence/radio-firmware-e2e-2026-11-18.md`; the fix is altitude (and velocity) hold on the FC.

Found this way and fixed (2026-11-17/18): angle mode used to over-bank in sustained tilts because
the estimator under-read the tilt (`docs/evidence/fc-sil-estimator-finding-2026-11-17.md`);
`test_fc_sil` now holds that as a regression test.

### Regenerating models and review images

```bash
python3 scripts/generate_twin_wing_assets.py [--check]       # Gazebo twin-wing model from its YAML
sim_py/.venv/bin/python scripts/render_twin_wing.py --flight  # MuJoCo twin-wing images
sim_py/.venv/bin/python scripts/render_multirotor.py          # MuJoCo multirotor images
python3 scripts/render_gazebo_twin_wing.py                    # Gazebo twin-wing images (ROS sourced)
```

---

## Python: control library and simulator

### Install

```bash
python -m pip install -e ".[sim]"           # numpy, scipy, matplotlib, pyyaml
python -m pip install -e ".[sim,mujoco]"    # + MuJoCo backends
python -m pip install -e ".[rotorpy]"       # optional RotorPy backend
```

Python 3.10-3.12 are supported. For the source-tree venv used by the MuJoCo commands in
this guide: `./scripts/setup_sim_py_venv.sh [--with-rotorpy]`, then
`source sim_py/.venv/bin/activate` (or call `sim_py/.venv/bin/python` directly).

### Run

```bash
# The simulator CLI (also installed as `aerial-kit-sim`)
python -m aerial_kit.sim.cli --controller constrained_mpc --no-show
python -m aerial_kit.sim.cli --config sim_py/sim_config.yaml \
    --controller warm_mppi --terrain forest --planner rrtstar
```

Options of `aerial_kit.sim.cli`:

| Flag | Values |
|---|---|
| `--config` / `--example` | a YAML file, or `quadrotor` / `fixed-wing` |
| `--controller` | `pid`, `lqr`, `mpc`, `mppi`, `constrained_mpc`, `warm_mppi`, `l1_tecs`, `single_wing_l1_tecs`; wrench controllers (`--backend actuator` or `mujoco_multirotor`): `geometric`, `nmpc`, `cascade` |
| `--planner` | `straight`, `astar`, `rrt`, `rrtstar`, `dubins` |
| `--backend` | multirotor: `pointmass`, `multirotor`, `rotorpy`, `mujoco` (point mass in MuJoCo), `actuator` (motor-level), `mujoco_multirotor`; wings: `fixedwing`, `mujoco_fixedwing`, `singlewing`, `mujoco_singlewing` |
| `--airframe` | `quad`, `hex`, `octo`, `twin_wing`, `single_wing` |
| `--terrain` | `forest`, `mountains`, `plains` |
| `--sim-time`, `--dt` | override the config |
| `--no-show`, `--save PATH` | headless runs |

The source-tree entry point `python -m sim_py.run_sim` takes the same component names and
reads `sim_py/sim_config.yaml` by default. The helper scripts wrap the CLI with forest
terrain and RRT* by default:

```bash
./scripts/run_sim_pid.sh               # also run_sim_lqr.sh, run_sim_mpc.sh, run_sim_mppi.sh
./scripts/run_sim_mpc.sh headless      # no window; saves results/constrained_mpc.png
TERRAIN=mountains ./scripts/run_sim.sh warm_mppi
./scripts/run_terrain_sim.sh mpc mountains true
```

### Scenario settings (`sim_py/sim_config.yaml`)

- Start/goal: `path.start_relative_*`, `path.end_relative_*`; `end_relative_z: "auto"`
  picks a random goal altitude below the tallest tree.
- Planner: `path.planner_type` (`--planner` overrides). Runtime: `controller.sim_time`,
  `controller.dt` (`--sim-time`, `--dt` override). Backend: `simulation.backend`.
- Terrain look and scale: `visual.forest_density_scale`, `visual.tree_height_scale`,
  `visual.height_ratio`, `visual.tree_radius_ref`.
- Terrain generation is shared with ROS: `ros2_ws/src/terrain_generator/config/terrain_params.yaml`
  (forest tree count ≈ `forest.grid_size² · forest.density`); point at another file with
  `python -m sim_py.run_sim --terrain-config PATH`.

### Keyboard teleop (Matplotlib)

```bash
python examples/quadrotor/teleop.py      # generic quad
python examples/fixed_wing/teleop.py     # twin wing
```

or `python -m aerial_kit.sim.teleop [--airframe fixed-wing]`. Click the plot window first;
the HUD says `NO FOCUS` when keys are not reaching it, and held keys are dropped when it
loses focus.

| Key | Quadrotor | Fixed wing |
|---|---|---|
| `W/S` or `Up/Down` | forward / backward | pitch: dive / climb |
| `A/D` or `Left/Right` | strafe left / right | bank: turn left / right |
| `Space` / `Shift` | climb / descend | throttle up / down |
| `Q` / `E` | yaw left / right | differential-thrust yaw nudge |
| `X`, `P`, `C`, `-`/`=`, `H`, `Esc` | neutralize, pause, camera, zoom, help, exit | same |

Controls are body-relative. Tuning: `controller.teleop` / `controller.teleop_fixedwing` and
`visual.teleop_*` in `examples/quadrotor/config.yaml` and
`aerial_kit/sim/defaults/fixed_wing.yaml`. For ROS keyboard flying of the MuJoCo vehicles,
see [Vehicles](#ros-2).

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
| `single_wing_l1_tecs` | `SingleWingL1TECSController` | the same, with the yaw demand dropped (no yaw actuator) |
| `cascade` (wrench) | `CascadeController` | position PID -> attitude -> body rate -> wrench |
| `geometric` (wrench) | `GeometricController` | SE(3) tracking (Lee et al.): position to moment on the rotation group, velocity/acceleration/jerk feedforward, recovers from large attitudes |
| `nmpc` (wrench) | `NMPCController` | iLQR nonlinear MPC on the full model (thrust + body rates, rate-loop lag modelled), soft tilt limit, attitude-tracking rate loop |

Pilot flight modes for the MuJoCo multirotors (`velocity`, `angle`, `acro`) are in
`aerial_kit/controllers/multirotor_manual.py`, built on the cascade's stages.

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

- With the limits inactive and `terminal: stage`, it computes exactly the same command as
  `mpc`. A test checks this.
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

### Wrench (attitude-level) controllers

`GeometricController`, `NMPCController` and `CascadeController` return a body wrench (thrust
and moment), not an acceleration, so they fly a motor-level plant: `--backend actuator`
(the firmware-matched quad-X `ActuatorPlant`) or `mujoco_multirotor` (the four MuJoCo
airframes), or in code with `aerial_kit.dynamics.actuator_loop.fly`.

```bash
python -m aerial_kit.sim.cli --controller geometric --backend actuator --terrain forest --planner rrtstar
```

On a wrench backend the planned path becomes a minimum-snap trajectory
(`aerial_kit.trajectory.MinSnapTrajectory`: piecewise 7th order, rest at both ends,
continuous through snap, segment times optimised, limited by `path.trajectory_v_max`,
`trajectory_a_max` and `trajectory_j_max`), which `geometric` and `nmpc` track with
feedforward. `path.trajectory: waypoints` chases the planned waypoints instead.

The plant supplies the controller's physical defaults unless the controller section sets
them: `actuator` passes its mass; `mujoco_multirotor` passes mass, composite inertia and the
spec's cascade bandwidths. The `actuator` plant's settings are under `simulation.actuator`
(`mass_kg`, `arm_length_m`, `max_thrust_per_motor_n`, `motor_tau_s`, `plant_dt`);
`mujoco_multirotor`'s under `simulation.mujoco_multirotor` (see
[MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md)).

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

#### Wind

The actuator plant has wind: a steady part plus first-order Gauss-Markov gusts, acting
through translational drag (`F = -kv_drag (v - wind)`, so no drag means no wind):

```yaml
simulation:
  actuator:
    kv_drag: 0.3
    wind_mps: [0.0, -5.0, 0.0]   # east, north, up
    gust_sigma_mps: 1.0          # per axis, one sigma
    gust_tau_s: 2.0              # gust correlation time
```

Gusts come from their own seeded generator, so turning them on leaves the sensor noise
sequence unchanged. The MuJoCo backends take a steady `wind_mps` (ENU) in their own
section.

### Flying on an estimated state

By default the controller is handed the backend's true state. With
`simulation.estimator.mode: ekf` it is handed the INS EKF's estimate instead
(`aerial_kit.estimation.SimulatedIns`), from simulated GPS, baro and an accelerometer with
noise and bias, on every backend:

```yaml
simulation:
  estimator:
    mode: ekf                # truth (default) | ekf
    gps_rate_hz: 5.0
    gps_sigma_xy_m: 1.0
    gps_sigma_z_m: 2.0
    baro_rate_hz: 20.0
    baro_sigma_m: 0.3
    baro_offset_m: 0.0
    accel_sigma_mps2: 0.1
    accel_bias_mps2: [0.0, 0.0, 0.0]
    # ekf: {gps_sigma_xy: 2.0, ...}   # InsConfig overrides; defaults match the sensors
```

Attitude and body rates are passed through from the truth, as a flight controller's AHRS
would supply them. Waypoint progress follows the estimate, while collisions, the recorded
trajectory and the distance to goal use the truth. `SimulationResult.estimated_trajectory`
holds the estimate beside each true sample, and `estimation_error_rms()` summarises it.
With the default sensors, the quadrotor example estimates position to about 0.5 m RMS. In
ROS, the fixed-wing launches take `estimator_mode:=ekf`.

---

## ROS 2 workspace

### Build

```bash
source ~/.local/share/ros/jazzy/setup.bash    # or /opt/ros/<distro>/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`./scripts/build.sh` does the same. ROS packages import `aerial_kit` from the repo: run
from inside the repo, install it with `pip install -e .`, or set `AERIAL_KIT_REPO_ROOT`.
The MuJoCo launches need `mujoco` importable by the ROS Python.

### Topics

| Topic | Type | From -> to |
|---|---|---|
| `/uav/command` | `uav_msgs/Command` | ground station -> command manager, mission follower |
| `/uav/mission` | `uav_msgs/Trajectory` | ground station / planner -> mission follower |
| `/uav/internal/mission_cmd_vel` | `geometry_msgs/Twist` | mission follower -> command manager |
| `/uav/mission_status` | `uav_msgs/MissionStatus` | mission follower -> ground station |
| `/uav/backend/cmd_twist`, `/uav/backend/enable` | `Twist`, `Bool` | command manager -> multirotor backend |
| `/uav/backend/odom` | `nav_msgs/Odometry` | multirotor backend / estimator -> telemetry adapter |
| `/uav/backend/telemetry_raw` | `uav_msgs/Telemetry` | telemetry adapter or wing backend -> command manager, follower |
| `/uav/telemetry` | `uav_msgs/Telemetry` | command manager -> ground station |
| `/uav/control/predicted_path` | `nav_msgs/Path` | predictive tracker -> RViz |
| `/uav/backend/cmd_airspeed_nav` | `uav_msgs/AirspeedNav` | fixed-wing guidance -> wing backend |
| `/uav/backend/cmd_actuators` | `uav_msgs/ActuatorCommand` | wing teleop -> wing backend (physical L/R, TE-up +) |
| `/uav/backend/manual` | `uav_msgs/MultirotorManualCommand` | multirotor teleop -> multirotor backend (mode + sticks) |
| `/uav/sim/viewer_camera` | `std_msgs/String` | teleop (`C`) -> MuJoCo backend: `next`, `chase` or `onboard` |

The command manager owns the multirotor flight modes: takeoff, hover hold, mission, land
and RTL. In mission mode it forwards the follower's velocity setpoint to the backend.

### Simulator backends

| Launch | Backend node | Plant |
|---|---|---|
| `sim_fast bringup.launch.py` | `fastsim_backend_adapter_node` or (`sim_backend:=mujoco`) `multirotor_backend_adapter_node` | point mass, or a MuJoCo multirotor |
| `sim_fast fixed_wing_*` | `fixedwing_backend_adapter_node` | twin or single wing, analytic or MuJoCo |
| `sim_gazebo bringup.launch.py` | `gazebo_backend_adapter_node` | Gazebo quad (`lr_drone` / `x3`) |
| `sim_gazebo fixed_wing_gazebo_demo.launch.py` | wing backend + `gazebo_fixedwing_pose_node` | analytic twin wing, Gazebo as a kinematic visual |

Gazebo forest / mountains demos and the ground station are in
[ros2_ws/README.md](../ros2_ws/README.md):

```bash
ros2 launch sim_gazebo bringup.launch.py [mission_tracker:=mppi]
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
| Obstacles | from the planner's path only | live `/terrain/obstacles`: MPPI avoids them; a blocked path triggers a replan from the planner service |
| Onboard planning | yes (`PLANNING_ONBOARD`) | no; it flies the trajectory it is given |

The tracker turns a planned acceleration into the velocity setpoint the backend's own
velocity loop needs: `v + a * velocity_loop_tau_s`. The `sim_fast` bringup sets it per
backend: 0.67 s for the point mass, `1 / velocity_kp` of the airframe spec for MuJoCo. More
detail is in [uav_control/README.md](../ros2_ws/src/uav_control/README.md).

Frames: Gazebo's models fly nose along body +Y and take body-frame twists, so both
followers default to `command_frame: body`, `nose_axis: +y`. `sim_fast` takes world-frame
velocity, so its launch runs them with `command_frame: world` and heading control off.

---

## Hardware backends

| Autopilot | Package | Launch |
|---|---|---|
| Betaflight over CRSF (ESP-NOW TX) | `hw_bridge` | `ros2 launch hw_bridge hw_crsf.launch.py udp_host:=192.168.4.1` |
| Direct ELRS/CRSF host UART | `hw_bridge` | `ros2 run hw_bridge crsf_radio_backend_node --ros-args -p transport:=serial -p serial_port:=/dev/ttyUSB0` |
| PX4 / ArduPilot over MAVLink | `mavlink_bridge` | `ros2 launch mavlink_bridge real_hardware.launch.py` |

Wiring, the TX UDP receiver and the packet format are in [HARDWARE.md](HARDWARE.md). MAVLink
setup is in [mavlink_bridge/README.md](../ros2_ws/src/mavlink_bridge/README.md).

`hw_bridge` notes:

- `crsf_radio_backend_node` sends real CRSF RC frames over a 400000-baud serial
  transport. Its `transport:=ros` mode feeds the SIL radio below. Channel mapping,
  arming sequence, command-frame conversion, reconnect behavior, telemetry topics
  and failsafe behavior are in [HARDWARE.md](HARDWARE.md).
- `hw_state_estimator_node` runs an **INS EKF** by default (`estimator: ekf`, from
  `aerial_kit.estimation`). It estimates position, velocity, accelerometer bias and baro
  offset, rejects GPS outliers with a chi-square gate, and publishes covariances. Set
  `imu_accel_mode` to match your FC: the default `none` is correct for Betaflight over CRSF
  (attitude only); use `body_specific_force` for a REP-145 IMU. `estimator: complementary`
  keeps the older baro filter (`baro_filter_hz`) and per-fix GPS differencing.
- `crsf_backend_adapter_node` maps the velocity **error** (demand minus measured) to sticks,
  because in Angle mode the sticks command acceleration. `vz_feedback` is on by default.
  `vxy_feedback` (GPS-derived) is opt-in until you have checked its velocity is smooth on
  your vehicle. Without fresh odometry it falls back to the open-loop mapping.
- Before flight, tune `hover_throttle`, `kz` and `kv_xy` per airframe, and keep Betaflight in
  Angle mode.
- Sensor topics for hardware estimation: `/uav/hw/imu` (`sensor_msgs/Imu`), `/uav/hw/baro`
  (`std_msgs/Float64`, m), `/uav/hw/gps` (`sensor_msgs/NavSatFix`).
- The simulator's elevon convention is physical left/right, trailing-edge-up positive, the
  same as [FIRMWARE_WING_CONTROL.md](FIRMWARE_WING_CONTROL.md).

### Simulated ELRS radio

The standard bringup can insert a deterministic 150 Hz radio between the ROS
command stack and the MuJoCo plant:

```bash
ros2 launch sim_fast bringup.launch.py start_demo:=true sim_backend:=mujoco \
    airframe:=fpv_quad_5in radio_sim:=true radio_rate_hz:=150 \
    radio_latency_ms:=8 radio_packet_loss_percent:=10
```

The path is command manager -> `crsf_radio_backend_node` -> simulated ELRS link
-> CRSF receiver -> plant. Attitude, GPS, battery and flight-mode telemetry travel
back through the same link into the unchanged hardware estimator. Publish
`std_msgs/Bool` true on `/uav/radio/link_cut` to cut the link and false to restore
it.

The simulated receiver models the FC's arming rule: arm high with throttle below
`throttle_low` for `arm_hold_sec`. It then publishes decoded AETR sticks as
`uav_msgs/MultirotorManualCommand` on `/uav/radio/decoded_manual`.

The SIL launch uses the manual controller's VELOCITY mode and `command_frame:
world`, rotating demand into body right/forward before channel encoding. This
proves CRSF stick transport, arming and loss behavior with a closed velocity
loop; it does **not** prove the plant's ANGLE-mode response or a real FC's
Angle-mode tuning. A direct angle-stick path exists for hardware
(`stick_mapping: angle`) and must be validated with props removed. Hardware
`vxy_feedback` remains off by default until checked per vehicle.

Recorded campaign commands:

```bash
RADIO_SIL_TIMEOUT_SEC=600 ./scripts/run_radio_sil_mission.sh fpv_quad_5in 0 8 150 fpv-0pct
RADIO_SIL_TIMEOUT_SEC=600 ./scripts/run_radio_sil_mission.sh fpv_quad_5in 10 8 150 fpv-10pct
RADIO_SIL_TIMEOUT_SEC=600 ./scripts/run_radio_sil_mission.sh research_hex 0 8 150 hex-0pct
./scripts/run_radio_sil_link_loss.sh fpv_quad_5in
```

---

## Testing

```bash
sim_py/.venv/bin/python -m pytest -q sim_py/tests     # library + simulator (MuJoCo tests skip without mujoco)
./scripts/test_ros2.sh                                # builds the workspace, runs every ROS package's tests
python3 scripts/generate_twin_wing_assets.py --check  # Gazebo twin-wing model matches its YAML

# ROS packages' pure logic without a ROS install (from each package directory)
cd ros2_ws/src/uav_control && python -m pytest test -q
```

`./scripts/test_ros2.sh` needs the ROS underlay (`ROS_SETUP`, default
`~/.local/share/ros/jazzy/setup.bash`); pass test paths to run a subset. It includes the
rclpy integration tests (`hw_bridge/test/test_sitl_closed_loop.py`: fake FC -> estimator ->
CRSF adapter -> command manager -> executor; `uav_control/test/test_ros_sim_fast.py`:
tracker missions against `sim_fast`) and the MuJoCo adapter/teleop tests.

`.github/workflows/python-tests.yml` runs the simulator suite, the ROS-free package tests
and a `ros:jazzy` job on every pull request.

In-process ROS tests use a `SingleThreadedExecutor`. With several executor threads in one
process, the nodes starve each other of the GIL. Don't run ROS tests while a launch is
flying on the same `ROS_DOMAIN_ID`: tests publish on the same `/uav/*` topics.

---

## Monitoring and debugging

```bash
./scripts/check_sim.sh              # nodes, which follower is running, topic samples
./scripts/debug_topics.sh           # follows a command hop by hop; the first silent hop is the break
./scripts/check_ros2_v2_topics.sh   # /uav/* topic samples

ros2 topic echo /uav/mission_status
ros2 topic echo /uav/telemetry --field status_text
ros2 topic echo /uav/backend/telemetry_raw --field status_text   # wing: active control source
ros2 topic hz /uav/backend/odom
```

Mission and teleop measurements: `scripts/fixed_wing_mission_monitor.py`,
`scripts/multirotor_mission_monitor.py`, `scripts/teleop_session_probe.py`,
`scripts/multirotor_teleop_probe.py`. Measured results are in [evidence/](evidence/).

In RViz, add `Path` on `/uav/control/predicted_path` to see the tracker's plan, and
`MarkerArray` on `/gs/planner/planned_path_markers` and `/terrain/obstacles`.

---

## Configuration reference

| What | Where |
|---|---|
| Python sim scenario, controllers | `sim_py/sim_config.yaml`, `examples/*/config.yaml`, `examples/*/mujoco.yaml`, `aerial_kit/sim/defaults/*.yaml` |
| Airframe specs (mass, geometry, motors, battery, materials) | `aerial_kit/airframes/multirotor_specs/*.yaml`, `twin_wing/twin_wing.yaml`, `single_wing/single_wing.yaml` |
| Terrain (shared with ROS) | `ros2_ws/src/terrain_generator/config/terrain_params.yaml` |
| Predictive tracker | `ros2_ws/src/uav_control/config/mpc_tracker.yaml`, `mppi_tracker.yaml` |
| Executor / command manager (Gazebo) | `ros2_ws/src/sim_gazebo/config/mission_executor_safe_tracking.yaml`, `command_manager_safe_custom.yaml` |
| MuJoCo / teleop launch arguments | `ros2_ws/src/sim_fast/launch/*.launch.py` (`ros2 launch ... --show-args`) |
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

**Teleop keys do nothing.** The small Tk teleop window must have keyboard focus; the MuJoCo
viewer usually opens second and takes it. Click the teleop window. Its status line shows
`ARMED`/`DISARMED` and the live command.

**`viewer:=true` but no window.** The adapter logs why (`no DISPLAY`, `MUJOCO_GL=egl`, or an
OpenGL probe failure) and keeps flying headless.

**The aircraft creeps to waypoints with the predictive tracker.** `velocity_loop_tau_s` is
much smaller than the backend's velocity-loop time constant, so the aircraft gets only a
fraction of each planned acceleration. Use the provided `sim_fast` bringup, which sets it.

**The mission doesn't start.** The follower acts only in `MODE_MISSION` and pauses on manual
override. Check `/uav/telemetry` `status_text`, and that only one follower is running.

**It doesn't move in `sim_fast`.** Body-frame, forward-only commands with no yaw in the
point-mass backend; use the provided launch, which sets `command_frame: world`.

**Processes left behind after Ctrl-C.** `ros2 launch` signals its nodes; if a launch was
killed hard, check with `ps -eo args | grep _node` and stop leftovers before the next run.

**`gz: command not found`, or Gazebo topics missing.** See the troubleshooting section of
[ros2_ws/README.md](../ros2_ws/README.md).
