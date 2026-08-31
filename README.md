# aerial-kit

**UAS Controller, Planner, Simulator, Firmware** — a control, planning, simulation, and
radio-link stack for **aerial robots**. Alongside the ROS 2 / Python control stack, this
repo carries fully custom ESP32 flight-link firmware (ESP-NOW, ELRS, LoRa, GPS) — custom
multirotor firmware is being added here too, alongside the existing links. The shared,
ROS-free control layer is also published standalone on PyPI as
[`aerial-kit`](https://pypi.org/project/aerial-kit/) (`pip install aerial-kit`).

- **Controller**: ROS 2 control + safety pipeline (hardware + Gazebo Sim / fast sim), plus
  a standalone `aerial_kit` package (PID / LQR / MPC, L1 + TECS for fixed wing)
- **Planner**: straight / A* / RRT / RRT* / Dubins path planners
- **Simulator**: standalone Python simulator (no ROS) for fast iteration on planners/
  controllers, plus ROS 2 + Gazebo simulation
- **Firmware**: ESP32 firmware (ESP-NOW, ELRS, LoRa, GPS) for manual flight + autonomous
  command relay, plus autopilot bridges for PX4, ArduPilot, and Betaflight

## What's in this repo

- **Controllers**
  - **ROS 2**: PID / LQR / MPC (work-in-progress depending on package)
  - **Python-only** (`sim_py`): PID / LQR / MPC position controllers + Matplotlib teleop
- **Path planning (Python-only)**: straight / A* / RRT planners
- **Terrain generation**: forest / mountains / plains (shared between ROS and Python sim)
- **Safety**: validation, limiting, watchdog (`safety_gate`)
- **Hardware link**: CRSF adapter + ESP-NOW / ELRS TX/RX + protocol bridging
- **Autopilot bridges**: MAVLink (PX4 / ArduPilot) and CRSF (Betaflight)

## Demo

All demos below are **quadcopter** flights — see [Supported airframes](#supported-airframes)
for what else is in progress.

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

## Supported airframes

| Airframe | Simulation | Control |
|----------|-----------|---------|
| Quadcopter | ✅ Working | ✅ PID / LQR / MPC |
| Hexacopter | ✅ Mixer verified | ✅ PID / LQR / MPC |
| Octacopter | ✅ Mixer verified | ✅ PID / LQR / MPC |
| Twin-motor wing | 🚧 6-DOF backend | 🚧 L1/TECS guidance |
| Single-motor wing | ⏳ To be added | ⏳ To be added |
| Monocopter | ⏳ To be added | ⏳ To be added |
| TVC (thrust-vectored) | ⏳ To be added | ⏳ To be added |

Quadcopter is the reference airframe and everything in the demos above is a quad. The
pieces that are still quad-shaped today are the `rotorpy` dynamics backend, the
`hover_throttle` / velocity-to-stick mapping in `hw_bridge`, and the Gazebo `x3` /
`lr_drone` models. Lifting those into a shared airframe layer is the active line of work;
see [Roadmap](#roadmap).

## Autopilot & firmware support

| Platform | Link | Package | Status |
|----------|------|---------|--------|
| Betaflight | CRSF over ESP-NOW/UDP relay | `ros2_ws/src/hw_bridge` (`crsf_backend_adapter_node`) | ✅ Working |
| PX4 | MAVLink over USB / UART / telemetry radio | `ros2_ws/src/mavlink_bridge` | ✅ Working |
| ArduPilot | MAVLink, same node (`flight_stack: ardupilot`) | `ros2_ws/src/mavlink_bridge` | 🚧 Supported, in testing |
| Custom firmware (this repo) | ESP-NOW + CRSF, 2.4 GHz SX1280 ELRS, LoRa telemetry | `firmware/` | ✅ Working |

All autopilot backends implement the same `/uav/backend/*` contract as the simulation
adapters, so the mission and control stack above them is unchanged between sim, Betaflight,
and PX4/ArduPilot.

## Architecture (high level)

**Manual Flight**

```
TX (IMU+Joystick) -> ESP-NOW -> RX -> Protocol Bridge -> Flight Controller
                                      (CRSF/SBUS/PPM/iBus/FrSky)
```

**Autonomous (Hardware-in-the-loop)**

```
/uav/backend/cmd_twist + /uav/backend/enable
    -> hw_bridge (crsf_backend_adapter_node)      -> Betaflight over CRSF/ESP-NOW
     | mavlink_bridge (mavlink_bridge_node)       -> PX4 / ArduPilot over MAVLink

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

PX4 / ArduPilot MAVLink backend launch:

```bash
# Direct MAVLink to the flight controller (USB / UART / telemetry radio).
# Defaults come from mavlink_bridge/config/mavlink_bridge_default.yaml.
ros2 launch mavlink_bridge real_hardware.launch.py

# To change connection_url / baud / flight_stack ("px4" or "ardupilot"),
# copy that YAML and point the launch at it:
ros2 launch mavlink_bridge real_hardware.launch.py \
    mavlink_bridge_params_file:=/path/to/my_vehicle.yaml

# SITL / UDP via the older hw_bridge adapter
ros2 launch hw_bridge hw_px4.launch.py mavlink_url:=udpin:0.0.0.0:14540
```

Wiring and parameter details: `ros2_ws/src/mavlink_bridge/README.md`.

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

The `firmware/gps/` project is an ESP32 GPS bring-up/telemetry module using `Adafruit_GPS`.

- Supports PMTK/NMEA modules (Adafruit Ultimate GPS / MTK33xx style)
- Supports u-blox modules with UBX configuration (while parsing NMEA output)
- Auto-probes common UART baud rates (9600/38400/115200), parses fix/satellite/SNR metrics, and prints diagnostics over serial
- Build/flash protocol options:
  - `pio run -d firmware/gps -e gps_auto -t upload` (AUTO detect PMTK vs UBLOX)
  - `pio run -d firmware/gps -e gps_pmtk -t upload` (force PMTK mode)
  - `pio run -d firmware/gps -e gps_ublox -t upload` (force UBLOX mode)

The host-side live dashboard is `tools/gps_dashboard.py` (see `tools/GPS_DASHBOARD.md`).

<img src="docs/gps_demo_1.png" alt="GPS telemetry demo output" width="700">

## Packages / folders

| Path | Type | Purpose |
|------|------|---------|
| `ros2_ws/src/air_unit` | Python | Air-side command manager / mission executor / telemetry adapter |
| `ros2_ws/src/ground_station` | Python | CLI, monitor, and demo mission tools |
| `ros2_ws/src/planner` | Python | ROS2 planner service wrapper for `sim_py` planners |
| `ros2_ws/src/sim_bridge` | Python | Backend adapters (Gazebo / fast sim) |
| `ros2_ws/src/hw_bridge` | Python | Hardware backend adapters + estimator (CRSF/Betaflight) |
| `ros2_ws/src/mavlink_bridge` | Python | MAVLink backend adapter for PX4 / ArduPilot |
| `ros2_ws/src/sim_fast` | Python | Headless simulation bringup |
| `ros2_ws/src/sim_gazebo` | Python | Gazebo Sim bringup and assets |
| `ros2_ws/src/uav_algorithms` | Python | Shared algorithms / planning API helpers |
| `ros2_ws/src/drone_msgs` | ROS msgs/srvs | Command, telemetry, mission, planner interfaces |
| `ros2_ws/src/terrain_generator` | Python | Terrain + obstacles (forest/mountains/plains) |
| `sim_py` | Python | Standalone planner/controller/dynamics/visualization |
| `firmware/espnow` | ESP32 | ESP-NOW TX/RX firmware + protocol bridging |
| `firmware/elrs` | ESP32 | ExpressLRS-compatible SX1280 TX/RX (CRSF over the air) |
| `firmware/lora` | ESP32 | LoRa point-to-point template (long-range telemetry) |
| `firmware/gps` | ESP32 | GPS telemetry module (Adafruit_GPS / NMEA + PMTK + UBX) |
| `tools` | Python | RC protocol decoders/monitors, 3D visualizer, GPS dashboard |
| `assets/urdf` | URDF/SDF | Airframe descriptions, one folder per airframe family |

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
- **`firmware/README.md`**: firmware project index (ESP-NOW / ELRS / LoRa / GPS)
- **`firmware/espnow/README.md`**: TX/RX firmware details
- **`firmware/elrs/README.md`**: ExpressLRS-compatible link details
- **`ros2_ws/src/mavlink_bridge/README.md`**: PX4 / ArduPilot wiring and setup
- **`tools/README.md`**: protocol monitor / decoder tooling
- **`tools/GPS_DASHBOARD.md`**: GPS serial dashboard usage
- **`assets/urdf/README.md`**: airframe description conventions

## Roadmap

The repo is moving from a quadcopter stack to a general aerial-robotics stack. In order:

1. **Airframe abstraction** — done: a shared `aerial_kit` package provides
   `Airframe`/`Capabilities`/`Wrench`/`trim()`, with `sim_py.core` re-exporting so existing
   imports are unaffected. Hex/octo mixers are config-only, unit-tested, and fly the same
   mission as quad. A controller/airframe `CommandKind` mismatch fails at startup with a
   readable error. Published standalone on PyPI as
   [`aerial-kit`](https://pypi.org/project/aerial-kit/) (`pip install aerial-kit`) — no ROS,
   no matplotlib dependency.
2. **Twin-motor wing** — **under development, simulation-only, not flown.**
   `TwinWingAirframe`, `FixedWingBackend` (6-DOF + placeholder aero model), L1/TECS
   guidance with coordinated turns, and Dubins-planner path generation all pass their own
   simulated acceptance tests, but nothing here has touched real hardware and the aero
   model isn't fitted to any specific airframe. Still needed: URDF and a sim demo
   (user-supplied assets), then real flight-testing before any of this is trusted the way
   the quadcopter path is.
3. **Multirotor variants** — done: hexacopter and octacopter reuse the multirotor control
   law with a different mixer (config-only).
4. **Remaining airframes** — single-motor wing, monocopter, TVC.
5. **Autopilot breadth** — flight-test the ArduPilot path, and keep the Betaflight, PX4, and
   custom-firmware backends behind one backend contract.

## Notes

- **ROS 2 Humble** is required for ROS-based control + RViz simulation
- The **Python-only sim** (`sim_py`) is designed for fast iteration (no ROS needed)
- Airframe-specific behavior belongs in the control stack, not the firmware — the ESP32
  projects in `firmware/` carry RC channels and telemetry and are airframe-agnostic
