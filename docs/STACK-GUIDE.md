# Aerial Kit stack guide

What the stack supports, how it fits together, and where it is going. To **run** anything
(simulators, vehicles, teleop, ROS launches, tests), see the
[software guide](SOFTWARE_GUIDE.md). For the custom flight controller and web app, see
[firmware](flight-controller-firmware.md) and [configurator](web-configurator.md).

## Supported airframes

Everything below is simulation; nothing claims flight readiness.

| Airframe | Simulation | Control |
|----------|-----------|---------|
| Quadcopter (generic) | ✅ point mass, 6-DOF, RotorPy, firmware-matched actuator plant | ✅ PID / LQR / MPC / MPPI; geometric / NMPC / cascade |
| Hexa- / octocopter (generic) | ✅ mixer verified | ✅ same as quad |
| 5" FPV quad, 3" micro long-range quad (`lr_drone` CAD), research hex, research octo | ✅ MuJoCo: spinning rotors, motor speed + battery sag + ground effect, contact; ROS autonomous + keyboard (velocity / angle / acro) | ✅ cascade and the wrench controllers; ROS executor / MPC / MPPI |
| Twin-motor flying wing | ✅ 6-DOF Python, MuJoCo (articulated elevons and props), ROS autonomous + keyboard, Gazebo kinematic visual | ✅ L1/TECS |
| Single-motor (pusher) flying wing | ✅ 6-DOF Python, MuJoCo, ROS autonomous + keyboard | ✅ L1/TECS (no yaw actuator) |
| Swarm quad | ⏳ planned (one more multirotor spec) | ⏳ |
| Monocopter, TVC | ⏳ no dynamics model yet | ⏳ |

Aerodynamic, propulsion and battery numbers are research defaults, not fitted to real
aircraft. Per-vehicle detail: [MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md),
[TWIN_WING_AIRFRAME.md](TWIN_WING_AIRFRAME.md), [SINGLE_WING.md](SINGLE_WING.md).

Still quad-shaped: the `rotorpy` backend, the `hover_throttle` / velocity-to-stick mapping
in `hw_bridge`, and the Gazebo `x3` / `lr_drone` models (Gazebo models for the new
multirotors are in progress).

## Autopilot & firmware support

| Platform | Link | Package | Status |
|----------|------|---------|--------|
| Betaflight | CRSF over ESP-NOW/UDP relay | `ros2_ws/src/hw_bridge` (`crsf_backend_adapter_node`) | ✅ Working |
| ELRS USB TX Rev A3 | one boot-selected TX/RX image; direct CRSF over host UART and over-the-air ELRS | `ros2_ws/src/hw_bridge` (`crsf_radio_backend_node`) | 🧪 SIL validated; RF bench pending |
| PX4 | MAVLink over USB / UART / telemetry radio | `ros2_ws/src/mavlink_bridge` | ✅ Working |
| ArduPilot | MAVLink, same node (`flight_stack: ardupilot`) | `ros2_ws/src/mavlink_bridge` | 🚧 Supported, in testing |
| Custom firmware (this repo) | ESP-NOW + CRSF, 2.4 GHz SX1280 ELRS, LoRa telemetry | `firmware/` (link projects) | ✅ Working |

`firmware/legacy/wing` is **not** part of the table above: it is a bench demonstration
with its own controller, not a supported flight configuration — see
[`firmware/legacy/README.md`](../firmware/legacy/README.md#airframe-demonstrations--not-flight-controllers).

All autopilot backends implement the same `/uav/backend/*` contract as the simulation
adapters, so the mission and control stack above them is unchanged between sim,
Betaflight and PX4/ArduPilot.

## Architecture

**Manual flight**

```
TX (IMU+Joystick) -> ESP-NOW -> RX -> Protocol Bridge -> Flight Controller
                                      (CRSF/SBUS/PPM/iBus/FrSky)
```

**Autonomous (hardware in the loop)**

```
/uav/backend/cmd_twist + /uav/backend/enable
    -> hw_bridge (crsf_backend_adapter_node)      -> Betaflight over CRSF/ESP-NOW
     | hw_bridge (crsf_radio_backend_node)        -> CRSF UART -> ELRS TX/RX
     | mavlink_bridge (mavlink_bridge_node)       -> PX4 / ArduPilot over MAVLink

FC sensors (/uav/hw/imu, /uav/hw/baro, /uav/hw/gps)
    -> hw_state_estimator_node -> /uav/backend/odom
    -> telemetry_adapter_node  -> /uav/backend/telemetry_raw
```

**Simulation**

```
ROS 2:        ground station / air unit -> sim_bridge -> point mass | MuJoCo | Gazebo
              (multirotors: cmd_twist / odom; wings: AirspeedNav / telemetry;
               keyboard: ActuatorCommand or MultirotorManualCommand)
              optional SIL: CRSF backend -> simulated ELRS latency/loss -> receiver -> backend
Python only:  planner -> controller -> dynamics backend -> Matplotlib 3D
```

One spec file per airframe drives its dynamics parameters, its MuJoCo model and (for the
twin wing) its generated Gazebo model, so they cannot drift apart.

## Docker

Dockerfiles are split by workflow:

- ROS 2 Humble: `docker/Dockerfile.humble`
- Python simulator/tools: `docker/Dockerfile.sim`
- Firmware tooling (PlatformIO): `docker/Dockerfile.firmware`

```bash
docker build -f docker/Dockerfile.humble --target ros-dev -t uav-controller:ros-humble .
docker run --rm -it --network=host --privileged -v "$PWD":/workspace uav-controller:ros-humble

docker build -f docker/Dockerfile.sim -t uav-controller:sim .
docker run --rm -it -v "$PWD":/workspace uav-controller:sim

docker build -f docker/Dockerfile.firmware -t uav-controller:firmware .
docker run --rm -it -v "$PWD":/workspace uav-controller:firmware
```

More details: `docker/README.md`. The development machine runs ROS 2 Jazzy; the Humble
image is the containerised option.

## GPS module (ESP32)

`firmware/legacy/gps/` is an ESP32 GPS bring-up/telemetry module using `Adafruit_GPS`:
PMTK/NMEA and u-blox (UBX config) modules, auto-probed baud rates (9600/38400/115200),
fix/satellite/SNR diagnostics over serial.

```bash
pio run -d firmware/legacy/gps -e gps_auto -t upload    # auto-detect PMTK vs UBLOX
pio run -d firmware/legacy/gps -e gps_pmtk -t upload    # force PMTK
pio run -d firmware/legacy/gps -e gps_ublox -t upload   # force UBLOX
```

The host-side live dashboard is `tools/gps_dashboard.py` (see `tools/GPS_DASHBOARD.md`).

<img src="../docs/gps_demo_1.png" alt="GPS telemetry demo output" width="700">

## Packages / folders

| Path | Type | Purpose |
|------|------|---------|
| `aerial_kit` | Python | ROS-free library: airframes and specs, dynamics, controllers, estimation, simulator API |
| `sim_py` | Python | Standalone simulator: planners, backends (incl. MuJoCo), visualizer, tests |
| `ros2_ws/src/air_unit` | Python | Command manager / mission executor / telemetry adapter |
| `ros2_ws/src/ground_station` | Python | CLI, monitor and demo mission tools |
| `ros2_ws/src/planner` | Python | ROS 2 planner service wrapper for `sim_py` planners |
| `ros2_ws/src/sim_bridge` | Python | Simulation backend adapters: point mass, MuJoCo multirotor, fixed wing (analytic / MuJoCo), Gazebo |
| `ros2_ws/src/hw_bridge` | Python | Hardware backend adapters + estimator (CRSF/Betaflight) |
| `ros2_ws/src/mavlink_bridge` | Python | MAVLink backend adapter for PX4 / ArduPilot |
| `ros2_ws/src/sim_fast` | Python | Headless and MuJoCo bringups, fixed-wing and teleop launches |
| `ros2_ws/src/sim_gazebo` | Python | Gazebo Sim bringup and assets |
| `ros2_ws/src/uav_algorithms` | Python | Shared algorithms / planning API helpers |
| `ros2_ws/src/uav_control` | Python | Predictive mission tracker (constrained MPC / MPPI), fixed-wing guidance, keyboard teleop nodes |
| `ros2_ws/src/uav_msgs` | ROS msgs/srvs | Command, telemetry, mission, planner, AirspeedNav, actuator and manual-stick interfaces |
| `ros2_ws/src/terrain_generator` | Python | Terrain + obstacles (forest/mountains/plains) |
| `firmware/legacy/espnow` | ESP32 | ESP-NOW TX/RX firmware + protocol bridging |
| `firmware/legacy/elrs` | ESP32 | ExpressLRS-compatible SX1280 TX/RX (CRSF over the air) |
| `firmware/legacy/lora` | ESP32 | LoRa point-to-point template (long-range telemetry) |
| `firmware/legacy/gps` | ESP32 | GPS telemetry module (Adafruit_GPS / NMEA + PMTK + UBX) |
| `firmware/legacy/wing` | Arduino / STM32 | Flying-wing bench demonstration — **not a flight controller** |
| `tools` | Python | RC protocol decoders/monitors, 3D visualizer, GPS dashboard |
| `assets/urdf` | URDF/SDF | Airframe descriptions, one folder per airframe family |

## Docs

- **[SOFTWARE_GUIDE.md](SOFTWARE_GUIDE.md)**: how to run everything (start here)
- **[EXAMPLE_USAGE.md](EXAMPLE_USAGE.md)**: worked terrain + planner + controller examples
- **[MULTIROTOR_MUJOCO.md](MULTIROTOR_MUJOCO.md)**, **[TWIN_WING_AIRFRAME.md](TWIN_WING_AIRFRAME.md)**,
  **[TWIN_WING_TELEOP.md](TWIN_WING_TELEOP.md)**, **[SINGLE_WING.md](SINGLE_WING.md)**: per-vehicle models
- **[evidence/](evidence/)**: measured test, mission and teleop results
- **`ros2_ws/README.md`**: Gazebo demos, topic diagram, troubleshooting
- **`sim_py/INFO.md`**: standalone simulator architecture
- **[HARDWARE.md](HARDWARE.md)**: TX integration for autonomous mode
- **[FIRMWARE_WING_CONTROL.md](FIRMWARE_WING_CONTROL.md)**: firmware wing mixing
- **`firmware/README.md`**, **`firmware/legacy/*/README.md`**: firmware and radio-link projects
- **`ros2_ws/src/mavlink_bridge/README.md`**: PX4 / ArduPilot wiring and setup
- **`tools/README.md`**, **`tools/GPS_DASHBOARD.md`**: protocol tools, GPS dashboard
- **`docker/README.md`**, **`assets/urdf/README.md`**

## Roadmap

1. **Airframe abstraction** — done. `aerial_kit` provides `Airframe` / `Capabilities` /
   `Wrench` / `trim()`; a controller/airframe `CommandKind` mismatch fails at startup.
   Published on PyPI as [`aerial-kit`](https://pypi.org/project/aerial-kit/).
2. **Multirotor family** — done in simulation: generic quad/hex/octo mixers, plus four
   spec-driven MuJoCo airframes (FPV, micro long-range, research hex and octo) with ROS
   autonomous and keyboard flight. Next: Gazebo models for them, a swarm quad.
3. **Fixed wings** — simulation only, not flown. Twin and single-motor wings with L1/TECS,
   6-DOF and MuJoCo plants, ROS missions and keyboard teleop. Next: fitted aerodynamics,
   native Gazebo aerodynamics, flight test.
4. **Remaining airframes** — monocopter, TVC (each needs its dynamics and control first).
5. **Autopilot breadth** — flight-test the ArduPilot path, and keep the Betaflight, PX4
   and custom-firmware backends behind one backend contract.

## Notes

- The **Python-only sim** (`sim_py`, `aerial_kit.sim`) needs no ROS; ROS 2 Jazzy is the
  tested distro for the workspace.
- Airframe-specific behaviour belongs in the control stack, not the link firmware: the
  ESP32 link projects (`espnow`, `elrs`, `lora`, `gps`) carry RC channels and telemetry and
  are airframe-agnostic. `firmware/legacy/wing` is the exception (its own mixer, controller
  and guidance) and is a bench demonstration. The AerialKit flight controller is in
  [`firmware/`](../firmware/README.md).
