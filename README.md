# UAV Controller + Path Planning

Multi-purpose quadcopter control stack with:
- **ROS 2 control + safety pipeline** (hardware + Gazebo Sim / fast sim)
- **Standalone Python simulator** (no ROS) for fast iteration on planners/controllers
- **ESP32 TX/RX link** for manual flight + autonomous command relay

## What's in this repo

- **Controllers**
  - **ROS 2**: PID / LQR / MPC (work-in-progress depending on package)
  - **Python-only** (`sim_py`): PID / LQR / MPC position controllers + Matplotlib teleop
- **Path planning (Python-only)**: straight / A* / RRT planners
- **Terrain generation**: forest / mountains / plains (shared between ROS and Python sim)
- **Safety**: validation, limiting, watchdog (`safety_gate`)
- **Hardware link**: CRSF adapter + ESP-NOW based TX/RX + protocol bridging

## Demo

### Python Sim & ROS2 Gazebo Sim

<p>
  <img src="docs/forest_rotor.png" alt="Forest RotorPy demo with quad pose overlay" width="49%">
  <img src="docs/mountain_rrt_star_1.png" alt="Mountain path planner with RRT*" width="49%">
</p>

<p>
  <img src="docs/sim_demo_1.png" alt="ROS2 Gazebo simulation demo" width="98%">
</p>

### Hovering & Landing (IMU + Barometer + GPS)

https://github.com/user-attachments/assets/44837663-b281-45db-9803-5aaa9812833d

*Autonomous hover and landing, commanded over the CRSF/ESP-NOW link. The state estimate fuses IMU
attitude, barometric altitude, and GPS position.*

## Architecture (high level)

**Manual Flight**

```
TX (IMU+Joystick) -> ESP-NOW -> RX -> Protocol Bridge -> Flight Controller
                                      (CRSF/SBUS/PPM/iBus/FrSky)
```

**Autonomous (Hardware-in-the-loop)**

```
/uav/backend/cmd_twist + /uav/backend/enable
    -> hw_bridge (crsf_backend_adapter_node | px4_backend_adapter_node)
    -> FC command link (CRSF/ESP-NOW or MAVLink)

FC sensors (/uav/hw/imu, /uav/hw/baro, /uav/hw/gps)
    -> hw_state_estimator_node
    -> /uav/backend/odom
    -> telemetry_adapter_node -> /uav/backend/telemetry_raw
```

**Simulation**

- **ROS 2 (Gazebo / fast sim)**:

```
Ground Station / Air Unit -> sim_bridge -> Gazebo Sim (or sim_fast)
```

- **Python-only (no ROS)**:

```
Planner -> Controller -> Dynamics backend (pointmass/rotorpy) -> Matplotlib 3D
```

## Quick start (Python-only simulator)

```bash
./scripts/setup_sim_py_venv.sh
source sim_py/.venv/bin/activate
python -m sim_py.run_sim
```

Optional RotorPy backend:

```bash
./scripts/setup_sim_py_venv.sh --with-rotorpy
source sim_py/.venv/bin/activate
python -m sim_py.run_sim --backend rotorpy
```

Useful overrides:

```bash
# Switch controller
python -m sim_py.run_sim --controller mpc

# Change terrain type (still uses the terrain config YAML unless overridden)
python -m sim_py.run_sim --terrain forest

# Override sim time / dt (if you pass these, they override sim_config.yaml)
python -m sim_py.run_sim --sim-time 240 --dt 0.01

# Use a different terrain config file
python -m sim_py.run_sim --terrain-config ros2_ws/src/terrain_generator/config/terrain_params.yaml

# Select dynamics backend (default: pointmass)
python -m sim_py.run_sim --backend rotorpy

# Interactive teleop (Matplotlib, best with RotorPy backend)
python -m sim_py.run_sim --controller teleop --backend rotorpy --terrain forest
```

### Python sim teleop (Matplotlib)

`sim_py` now includes an interactive teleop mode for quick manual flying in the Matplotlib 3D viewer.

- Launch: `python -m sim_py.run_sim --controller teleop --backend rotorpy`
- Controls (focus the plot window first):
  - `W/S`: +/- X
  - `A/D`: +/- Y
  - `R/F`: +/- Z
  - `P`: pause/resume
  - `Esc`: close

Teleop is acceleration-command based and works best with the RotorPy backend.

### Python sim configuration

- **Main config**: `sim_py/sim_config.yaml`
  - **Start/goal**: `path.start_relative_*`, `path.end_relative_*`
    - `end_relative_z: "auto"` picks a random goal altitude in \([0, \text{tallest tree}]\)
  - **Planner**: `path.planner_type` = `straight` | `astar` | `rrt` | `rrt*`
  - **Runtime**: `controller.sim_time`, `controller.dt`
  - **Backend**: `simulation.backend` = `pointmass` | `rotorpy` (CLI `--backend` overrides)
  - **Terrain appearance / scaling**:
    - `visual.forest_density_scale`: scales forest density (clamped to 1.0)
    - `visual.tree_height_scale`: scales sampled tree heights
    - `visual.height_ratio`: sets map height as `height_ratio * tallest_tree`
    - `visual.tree_radius_ref`: reference radius for drawing thicker/thinner trunks

- **Terrain config** (shared with ROS):
  - `ros2_ws/src/terrain_generator/config/terrain_params.yaml`
  - Forest obstacle count is mainly set by:
    - `forest.grid_size` and `forest.density`
    - expected trees ~= \(grid\_size^2 \cdot density\)

## Quick start (ROS 2)

### Gazebo / Ground-Air stack (current)

The rebuilt Gazebo + ground-station / air-unit stack now lives in `ros2_ws`.

- Workspace docs / runbook: `ros2_ws/README.md`
- Primary sim bringup: `ros2 launch sim_gazebo bringup.launch.py`
- Ground station bringup: `ros2 launch ground_station ground.launch.py`

### Hardware autonomous flight

CRSF/Betaflight backend launch:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch hw_bridge hw_crsf.launch.py udp_host:=192.168.4.1
```

PX4/MAVLink backend launch:

```bash
ros2 launch hw_bridge hw_px4.launch.py mavlink_url:=udpin:0.0.0.0:14540
```

Sensor topic contract for hardware estimation:
- `/uav/hw/imu` (`sensor_msgs/msg/Imu`)
- `/uav/hw/baro` (`std_msgs/msg/Float64`, meters)
- `/uav/hw/gps` (`sensor_msgs/msg/NavSatFix`)

Bring-up and TX integration details are in `docs/HARDWARE.md`.

**Before real flight:** tune `hover_throttle` and mapping gains per airframe, and keep Betaflight in **Angle mode** for the velocity-to-stick mapping used by `hw_bridge`.

### Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Gazebo / Fast simulation

```bash
ros2 launch sim_gazebo bringup.launch.py
ros2 launch ground_station ground.launch.py

# Fast headless backend
ros2 launch sim_fast bringup.launch.py
```

For the current tested Gazebo + terrain + planner demo commands (including dense forest and path visualization), use:
- `ros2_ws/README.md` -> `Recommended Test Flows (Current)`

### ROS2 Gazebo

<img src="docs/sim_demo_2.png" alt="ROS2 Gazebo demo screenshot (additional)" width="900">

### Legacy ROS2 prototype scripts

The old RViz/controller prototype workspace was replaced during consolidation.
If you still need the legacy PID/LQR/MPC ROS2 stack, recover it from git history.
Current ROS2 workflows are documented in `ros2_ws/README.md`.

## Docker quick start
Dockerfiles are split by workflow:

- ROS 2 Humble: `docker/Dockerfile.humble`
- Python simulator/tools: `docker/Dockerfile.sim`
- Firmware tooling (PlatformIO): `docker/Dockerfile.firmware`

Build and run:

```bash
# ROS 2 image
docker build -f docker/Dockerfile.humble --target ros-dev -t uav-controller:ros-humble .
docker run --rm -it --network=host --privileged -v "$PWD":/workspace uav-controller:ros-humble

# sim_py + Python tools image
docker build -f docker/Dockerfile.sim -t uav-controller:sim .
docker run --rm -it -v "$PWD":/workspace uav-controller:sim

# firmware / PlatformIO image
docker build -f docker/Dockerfile.firmware -t uav-controller:firmware .
docker run --rm -it -v "$PWD":/workspace uav-controller:firmware
```

More details: `docker/README.md`.

## GPS module (ESP32)

The `gps/` project is an ESP32 GPS bring-up/telemetry module using `Adafruit_GPS`.

- Supports PMTK/NMEA modules (Adafruit Ultimate GPS / MTK33xx style)
- Supports u-blox modules with UBX configuration (while parsing NMEA output)
- Auto-probes common UART baud rates (9600/38400/115200), parses fix/satellite/SNR metrics, and prints diagnostics over serial
- Build/flash protocol options:
  - `pio run -d gps -e gps_auto -t upload` (AUTO detect PMTK vs UBLOX)
  - `pio run -d gps -e gps_pmtk -t upload` (force PMTK mode)
  - `pio run -d gps -e gps_ublox -t upload` (force UBLOX mode)

<img src="docs/gps_demo_1.png" alt="GPS telemetry demo output" width="700">

## Packages / folders

| Path | Type | Purpose |
|------|------|---------|
| `ros2_ws/src/air_unit` | Python | Air-side command manager / mission executor / telemetry adapter |
| `ros2_ws/src/ground_station` | Python | CLI, monitor, and demo mission tools |
| `ros2_ws/src/planner` | Python | ROS2 planner service wrapper for `sim_py` planners |
| `ros2_ws/src/sim_bridge` | Python | Backend adapters (Gazebo / fast sim) |
| `ros2_ws/src/hw_bridge` | Python | Hardware backend adapters + estimator (CRSF/PX4) |
| `ros2_ws/src/sim_fast` | Python | Headless simulation bringup |
| `ros2_ws/src/sim_gazebo` | Python | Gazebo Sim bringup and assets |
| `ros2_ws/src/px4_bridge` | Python | PX4 bridge scaffolding |
| `ros2_ws/src/uav_algorithms` | Python | Shared algorithms / planning API helpers |
| `ros2_ws/src/drone_msgs` | ROS msgs/srvs | Command, telemetry, mission, planner interfaces |
| `ros2_ws/src/terrain_generator` | Python | Terrain + obstacles (forest/mountains/plains) |
| `sim_py` | Python | Standalone planner/controller/dynamics/visualization |
| `ESPNOW_TX` | ESP32 | ESP-NOW based TX/RX firmware |
| `LoRa_TX` | ESP32 | LoRa TX experiments |
| `gps` | ESP32 | GPS telemetry module (Adafruit_GPS / NMEA + PMTK + UBX support) |
| `Utils` | Python | Protocol decoder/monitor + tools |

## ROS 2 topics (current stack)

The current ROS2 Gazebo/fast-sim stack uses the `/uav/...` namespace by default.

- `/uav/command` (`drone_msgs/msg/Command`)
- `/uav/mission` (`drone_msgs/msg/Trajectory`)
- `/uav/telemetry` (`drone_msgs/msg/Telemetry`)
- `/uav/mission_status` (`drone_msgs/msg/MissionStatus`)
- `/uav/backend/cmd_twist` (`geometry_msgs/msg/Twist`)
- `/uav/backend/enable` (`std_msgs/msg/Bool`)
- `/uav/backend/odom` (`nav_msgs/msg/Odometry`)
- `/uav/backend/telemetry_raw` (`drone_msgs/msg/Telemetry`)
- `/uav/hw/imu` (`sensor_msgs/msg/Imu`)
- `/uav/hw/baro` (`std_msgs/msg/Float64`)
- `/uav/hw/gps` (`sensor_msgs/msg/NavSatFix`)

Gazebo bridged topics:

- `/model/x3/odometry`
- `/X3/gazebo/command/twist`
- `/X3/enable`

See `ros2_ws/README.md` for the full topic/node diagram and troubleshooting notes.

## Docs

- **[docs/SOFTWARE_GUIDE.md](docs/SOFTWARE_GUIDE.md)**: software guide (start here)
- **[docs/EXAMPLE_USAGE.md](docs/EXAMPLE_USAGE.md)**: terrain + controller examples
- **`sim_py/INFO.md`**: standalone simulator architecture + usage
- **`docker/README.md`**: Docker build/run commands by workflow
- **`docs/HARDWARE.md`**: TX integration for autonomous mode
- **`ESPNOW_TX/README.md`**: TX/RX firmware details
- **`gps/DASHBOARD.md`**: GPS serial dashboard usage
- **`Utils/README.md`**: protocol monitor / decoder tooling

## Notes

- **ROS 2 Humble** is required for ROS-based control + RViz simulation
- The **Python-only sim** (`sim_py`) is designed for fast iteration (no ROS needed)

## Hardware photo

<img src="docs/brushed.jpg" alt="Brushed Motor Quadcopter" width="400">

*Test brushed quad with the custom TX/RX.*
