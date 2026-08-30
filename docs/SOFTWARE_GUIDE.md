# UAV Controller - Software Guide

A comprehensive guide to the UAV Controller system covering simulation, hardware integration, controllers, and all features.

---

## Table of Contents

1. [Overview](#overview)
2. [Quick Start](#quick-start)
3. [Installation & Setup](#installation--setup)
4. [Simulation](#simulation)
5. [Controllers](#controllers)
6. [Terrain Generator](#terrain-generator)
7. [Hardware Integration](#hardware-integration)
8. [Monitoring & Debugging](#monitoring--debugging)
9. [Configuration & Tuning](#configuration--tuning)
10. [Troubleshooting](#troubleshooting)

---

## Overview

The UAV Controller is a multi-purpose control system for **aerial robots**. Multirotors are
implemented and flying today; fixed-wing, monocopter, and TVC airframes are being added — see
the airframe table in the [root README](../README.md#supported-airframes). Features:

- **Airframes**: quadcopter (reference, flying); hexacopter, octacopter, twin-motor wing,
  single-motor wing, monocopter, and TVC are planned
- **Controllers**: PID (cascaded), LQR (optimal), MPC (Model Predictive Control)
- **Safety**: Multi-layer validation, slew limiting, timeout watchdog
- **Autopilots**: Betaflight over CRSF, PX4 / ArduPilot over MAVLink, and this repo's own
  ESP32 firmware
- **Hardware**: Custom TX/RX with universal protocol support (CRSF, SBUS, PPM, iBus, FrSky)
- **Simulation**: 200Hz dynamics with RViz visualization
- **Terrain Generation**: Forest, Mountains, and Plains environments

> Anything in this guide that names a quadcopter — hover throttle, four-motor mixing, the
> `x3` / `lr_drone` Gazebo models — is describing the reference airframe, not a limit of the
> architecture.

### Architecture

**Simulation Mode:**
```
Controllers → Safety Gate → Simulator → RViz
```

**Autonomous Flight (Hardware):**
```
ROS Controllers → Safety Gate → CRSF Adapter → UDP → TX → ESP-NOW → RX → Protocol → FC
```

**Manual Flight (Hardware):**
```
TX (IMU+Joystick) → ESP-NOW → RX → Protocol → FC
```
*No computer needed for manual flight!*

---

## Quick Start

### Build the Workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Your First Simulation

**With Visualization:**
```bash
# PID controller
./scripts/run_sim_pid.sh

# LQR controller
./scripts/run_sim_lqr.sh

# MPC controller
./scripts/run_sim_mpc.sh
```

**Headless Mode (No Display):**
```bash
./scripts/run_sim_pid.sh headless
./scripts/run_sim_lqr.sh headless
./scripts/run_sim_mpc.sh headless
```

The vehicle (a quadcopter in the default configuration) should hover at 1m altitude in RViz
(or run silently in headless mode).

---

## Installation & Setup

### Prerequisites

- **ROS 2 Humble** (required for autonomous control)
- **Python 3.8+** (for simulation and utilities)
- **C++17 compiler** (GCC 9+ or Clang 10+)
- **Eigen3** (for matrix operations)

### Build Instructions

```bash
# Navigate to workspace
cd ros2_ws

# NOTE: No Python virtual environment is required for the ROS 2 workspace.

# Build all packages
# Ensure your ROS 2 distro is sourced (e.g. source /opt/ros/jazzy/setup.bash)
colcon build --symlink-install

# Source the workspace (do this every time you open a new terminal)
source install/setup.bash
```

### Build Specific Packages

```bash
# Build only controllers
colcon build --packages-select controllers_pid controllers_lqr controllers_mpc

# Build with verbose output
colcon build --event-handlers console_direct+

# Rebuild after config changes
colcon build --packages-select <package_name>
```

---

## Simulation

### Running Simulations

#### Method 1: Using Scripts (Recommended)

**With Visualization:**
```bash
./scripts/run_sim_pid.sh    # PID controller
./scripts/run_sim_lqr.sh     # LQR controller
./scripts/run_sim_mpc.sh     # MPC controller
```

**Headless Mode:**
```bash
./scripts/run_sim_pid.sh headless
./scripts/run_sim_lqr.sh headless
./scripts/run_sim_mpc.sh headless
```

#### Method 2: Direct Launch Files

**With Visualization:**
```bash
ros2 launch sim_dyn sim_pid.launch.py use_rviz:=true
ros2 launch sim_dyn sim_lqr.launch.py use_rviz:=true
ros2 launch sim_dyn sim_mpc.launch.py use_rviz:=true
```

**Headless Mode:**
```bash
ros2 launch sim_dyn sim_pid.launch.py use_rviz:=false
ros2 launch sim_dyn sim_lqr.launch.py use_rviz:=false
ros2 launch sim_dyn sim_mpc.launch.py use_rviz:=false
```

### When to Use Headless Mode

- SSH sessions without X11 forwarding
- CI/CD pipelines and automated testing
- Remote servers without display
- Performance testing (no visualization overhead)
- Batch simulations for data collection

### Simulation Features

- **200Hz dynamics** with RK4 integration
- **Realistic physics** including drag and gravity
- **RViz visualization** (optional)
- **Real-time state publishing** (odometry, attitude, angular velocity)
- **TF transforms** for visualization

### Starting RViz Separately

If you started in headless mode but want visualization later:

```bash
rviz2 -d install/sim_dyn/share/sim_dyn/rviz/overview.rviz
```

---

## Controllers

### PID Controller

**Cascaded PID** with anti-windup protection.

**Configuration:** `ros2_ws/src/controllers_pid/config/pid_params.yaml`

**Key Parameters:**
- `kp`, `ki`, `kd`: Proportional, integral, derivative gains
- `anti_windup`: Integral term clamping
- `control_rate`: Update frequency (default: 100 Hz)

**Tuning Tips:**
- Start with low `kp` values
- Increase `kd` to reduce oscillations
- Use `anti_windup` to prevent integral saturation

### LQR Controller

**Linear Quadratic Regulator** for optimal state feedback.

**Configuration:** `ros2_ws/src/controllers_lqr/config/lqr_params.yaml`

**Key Parameters:**
- `K`: Gain matrix (4x6, row-major format)
- `mass`, `Jx`, `Jy`, `Jz`: System parameters
- `hover_thrust`: Nominal thrust for hover

**Tuning:**
- Compute K matrix offline using LQR solver
- Adjust Q and R matrices for desired performance
- Default gains are conservative (not true LQR solution)

### MPC Controller

**Model Predictive Control** with prediction horizon.

**Configuration:** `ros2_ws/src/controllers_mpc/config/mpc_params.yaml`

**Key Parameters:**
- `horizon`: Prediction horizon (default: 4)
- `Ts`: Sampling time (default: 0.1s)
- `Q`, `S`, `R`: State, terminal, and control cost matrices
- `Jx`, `Jy`, `Jz`, `Jtp`: Inertia parameters

**Features:**
- Incremental MPC (control increments)
- LPV (Linear Parameter Varying) model updates
- QP solver for optimization

---

## Terrain Generator

Generate realistic environments for path planning and obstacle avoidance testing.

### Available Terrain Types

1. **Forest** - Dense trees with configurable density
2. **Mountains** - Peaks and ridges with varying heights
3. **Plains** - Sparse obstacles (bushes, rocks, trees)

### Usage

**Launch Terrain Generator:**
```bash
# Forest
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=forest

# Mountains
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=mountains

# Plains
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=plains
```

**Configuration:** `ros2_ws/src/terrain_generator/config/terrain_params.yaml`

**Forest Parameters:**
- `grid_size`: Number of grid cells (default: 10)
- `radius_range`: Tree radius range [min, max] in meters
- `height_range`: Tree height range [min, max] in meters
- `density`: Probability of tree per cell (0-1)

**Mountains Parameters:**
- `num_peaks`: Number of mountain peaks
- `base_size_range`: Base size range [min, max] in meters
- `height_range`: Peak height range [min, max] in meters

**Plains Parameters:**
- `num_obstacles`: Number of obstacles
- `obstacle_types`: List of types ['bush', 'rock', 'tree']

**Visualization:**
- Terrain obstacles are published as RViz markers
- View in RViz: `/terrain/obstacles` topic
- Different colors for each terrain type

---

## Hardware Integration

### Overview

The system supports two hardware modes:

1. **Manual Flight**: TX with IMU+joystick, no ROS needed
2. **Autonomous Flight**: ROS controllers → TX via UDP → ESP-NOW → RX → FC

### TX/RX System

**Transmitter Features:**
- Hybrid IMU (BNO085/MPU6050) + Joystick control
- ESP-NOW wireless (low latency)
- Configurable sensitivity and update rates

**Receiver Features:**
- **Universal Protocol Support**:
  - ✅ CRSF (Crossfire/ELRS) - Betaflight, INAV
  - ✅ SBUS (Futaba) - Universal compatibility
  - ✅ PPM - Traditional flight controllers
  - ✅ iBus (FlySky) - FlySky receivers
  - ✅ FrSky S.PORT - FrSky telemetry systems

### Autonomous Mode Setup

**Step 1: Modify TX Firmware**

Add UDP support to `firmware/espnow/src/main.cpp` in the `#ifdef BUILD_TX` section:

```cpp
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const int UDP_PORT = 9000;
bool udpActive = false;

void setup() {
  // ... existing setup ...
  
  // Setup WiFi AP for ROS connection
  WiFi.softAP("UAV_TX", "uav12345");
  Serial.printf("TX IP: %s\n", WiFi.softAPIP().toString().c_str());
  
  udp.begin(UDP_PORT);
  udpActive = true;
}

void loop() {
  // Check for ROS commands via UDP
  if (udpActive) {
    int packetSize = udp.parsePacket();
    if (packetSize == 20) {
      uint8_t buffer[20];
      udp.read(buffer, 20);
      
      float* floats = (float*)buffer;
      float roll = floats[0];
      float pitch = floats[1];
      float yaw = floats[2];
      float throttle = floats[3];
      
      // Convert to CRSF channels
      rcChannels[0] = (uint16_t)(roll * 819.0 + 992.0);
      rcChannels[1] = (uint16_t)(pitch * 819.0 + 992.0);
      rcChannels[2] = (uint16_t)(172.0 + throttle * 1639.0);
      rcChannels[3] = (uint16_t)((-yaw) * 819.0 + 992.0);
      
      CustomProtocol_SendRcCommand(rcChannels);
    }
  }
  
  // ... existing code ...
}
```

**Step 2: Flash TX**

```bash
cd firmware/espnow
pio run -e transmitter -t upload
```

**Step 3: Connect and Run**

```bash
# Connect computer to "UAV_TX" WiFi network (password: uav12345)
# TX IP will be 192.168.4.1

# Launch ROS autonomous control
cd UAV-Controller
source ros2_ws/install/setup.bash
./scripts/run_crsf_link_pid.sh transport:=udp udp_host:=192.168.4.1
```

### Protocol Configuration

Edit `firmware/espnow/src/config.h` to select output protocol:

```cpp
#define OUTPUT_PROTOCOL PROTOCOL_CRSF  // or PROTOCOL_SBUS, PROTOCOL_PPM, etc.
```

Then reflash the receiver.

---

## Monitoring & Debugging

### Check Running Nodes

```bash
ros2 node list

# Expected nodes:
# /dynamics_node
# /pid_controller (or /lqr_controller or /mpc_controller)
# /safety_gate
# /rviz2 (if visualization enabled)
```

### Monitor Topics

**Controller Output:**
```bash
ros2 topic echo /cmd/body_rate_thrust
```

**After Safety Gate:**
```bash
ros2 topic echo /cmd/final/body_rate_thrust
```

**Simulator State:**
```bash
ros2 topic echo /state/odom
ros2 topic echo /state/attitude
ros2 topic echo /state/angular_velocity
```

**Check Publishing Rates:**
```bash
ros2 topic hz /cmd/body_rate_thrust  # Should be ~100 Hz
ros2 topic hz /state/odom           # Should be ~200 Hz
```

### Run Diagnostics

```bash
./scripts/check_sim.sh
```

### Debug Topics

```bash
./scripts/debug_topics.sh
```

### Universal Protocol Visualizer

Monitor RC channels from your receiver:

**3D Web Visualizer (Recommended):**
```bash
cd tools
pip install -r requirements.txt
python drone_visualizer.py COM3  # or /dev/ttyUSB0 on Linux
```

Then open browser to `http://localhost:5000`

**Features:**
- Switch protocols on-the-fly (CRSF/SBUS/iBus)
- 3D quadcopter visualization
- Real-time telemetry
- Flight mode switching (ANGLE/ACRO)

**Terminal Monitors:**
```bash
python crsf_serial.py COM3   # For CRSF
python sbus_serial.py COM3   # For SBUS
python ibus_serial.py COM3   # For iBus
```

---

## Configuration & Tuning

### Configuration Files

All configuration files are in each package's `config/` directory:

| Package | Config File | Purpose |
|---------|------------|---------|
| `controllers_pid` | `pid_params.yaml` | PID gains, anti-windup, limits |
| `controllers_lqr` | `lqr_params.yaml` | LQR K matrix, system parameters |
| `controllers_mpc` | `mpc_params.yaml` | MPC horizon, cost matrices, parameters |
| `safety_gate` | `safety_params.yaml` | Safety limits, timeouts |
| `sim_dyn` | `sim_params.yaml` | Dynamics model parameters |
| `adapters_crsf` | `crsf_params.yaml` | TX connection settings |
| `terrain_generator` | `terrain_params.yaml` | Terrain generation parameters |

### Tuning Controllers

#### PID Tuning

Edit `ros2_ws/src/controllers_pid/config/pid_params.yaml`:

```yaml
pid_controller:
  ros__parameters:
    kp: [2.0, 2.0, 1.0]  # Increase for faster response
    ki: [0.1, 0.1, 0.05]  # Increase to reduce steady-state error
    kd: [0.5, 0.5, 0.3]  # Increase to reduce oscillations
```

**Tuning Process:**
1. Start with low gains
2. Increase `kp` until oscillations appear
3. Increase `kd` to dampen oscillations
4. Add `ki` to eliminate steady-state error
5. Rebuild: `colcon build --packages-select controllers_pid`

#### LQR Tuning

Edit `ros2_ws/src/controllers_lqr/config/lqr_params.yaml`:

```yaml
lqr_controller:
  ros__parameters:
    K: [6.0, 0.0, 0.0, 1.0, 0.0, 0.0, ...]  # 4x6 matrix, row-major
```

**Note:** For true LQR, compute K matrix offline using your A, B, Q, R matrices.

#### MPC Tuning

Edit `ros2_ws/src/controllers_mpc/config/mpc_params.yaml`:

```yaml
mpc_controller:
  ros__parameters:
    horizon: 4  # Increase for longer prediction
    Q: [[10.0, 0.0, 0.0], ...]  # State cost (higher = prioritize tracking)
    R: [[10.0, 0.0, 0.0], ...]  # Control cost (higher = smoother control)
```

### Safety Limits

Edit `ros2_ws/src/safety_gate/config/safety_params.yaml`:

```yaml
safety_gate:
  ros__parameters:
    max_roll_rate: 5.0   # rad/s
    max_pitch_rate: 5.0  # rad/s
    max_yaw_rate: 3.0    # rad/s
    max_thrust: 1.0      # normalized
    min_thrust: 0.0      # normalized
```

---

## Troubleshooting

### Build Issues

**Build Errors:**
```bash
# Clean build
cd ros2_ws
rm -rf build install log
colcon build --symlink-install

# Verbose output
colcon build --event-handlers console_direct+
```

**Missing Dependencies:**
```bash
# Install Eigen3 (Ubuntu/Debian)
sudo apt-get install libeigen3-dev

# Install ROS 2 dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Simulation Issues

**Quad Falling / Deadlock:**
The simulation may have a circular dependency where controller needs state and dynamics needs commands. To break the deadlock:

```bash
# Publish initial hover command to start simulation
ros2 topic pub --once /cmd/final/body_rate_thrust common_msgs/msg/BodyRateThrust \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'body'}, \
   body_rates: {x: 0.0, y: 0.0, z: 0.0}, \
   thrust: 0.5}"
```

**Check thrust model parameter:**
```bash
ros2 param get /dynamics_node c1  # Should be 2.0

# If wrong, rebuild simulator
colcon build --packages-select sim_dyn
source install/setup.bash
```

**Nodes Not Starting:**
```bash
# Always source workspace!
source ros2_ws/install/setup.bash

# Check for build errors
colcon build --event-handlers console_direct+
```

**RViz "Could not connect to display":**
- Run in headless mode: `use_rviz:=false`
- Or use remote desktop/X11 forwarding for SSH

**RViz Not Showing Terrain:**
1. Enable Grid display in RViz (check the box)
2. Add terrain obstacles display: Click "Add" → "By topic" → `/terrain/obstacles` → "MarkerArray"
3. Set Fixed Frame to `map` (not `<Fixed Frame>`)
4. Zoom out if terrain is far from origin
5. Verify terrain is publishing: `ros2 topic hz /terrain/obstacles`

**Oscillations:**
- Reduce PID `kp` gains
- Increase PID `kd` gains
- Edit config, rebuild: `colcon build --packages-select controllers_pid`

### Hardware Issues

**TX Not Receiving UDP:**
```bash
# Check WiFi connection
ping 192.168.4.1

# Verify UDP port
netstat -ulnp | grep 9000
```

**RX Not Responding:**
- Check ESP-NOW link (LED indicators)
- Verify CRSF wiring (GPIO 21)
- Check receiver power

**No FC Response:**
- Verify FC set to CRSF input
- Check protocol match in `config.h`
- Test with manual TX mode first

### Topic Issues

**No Data on Topics:**
```bash
# Check if nodes are running
ros2 node list

# Check topic exists
ros2 topic list

# Check publishing rate
ros2 topic hz /topic_name
```

---

## Advanced Topics

### Running Tests

```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```

### Package Structure

```
ros2_ws/src/
├── controllers_pid/     # PID controller
├── controllers_lqr/     # LQR controller
├── controllers_mpc/     # MPC controller
├── safety_gate/         # Safety validation
├── sim_dyn/             # Dynamics simulator
├── adapters_crsf/       # ROS → TX bridge
├── terrain_generator/   # Terrain generation
└── common_msgs/         # Custom message types
```

### Message Types

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd/body_rate_thrust` | `BodyRateThrust` | Controller output |
| `/cmd/final/body_rate_thrust` | `BodyRateThrust` | After safety (sim) |
| `/cmd/final/rc` | `VirtualRC` | After safety (hardware) |
| `/state/odom` | `Odometry` | Current state |
| `/state/attitude` | `QuaternionStamped` | Orientation |
| `/state/angular_velocity` | `Vector3Stamped` | Body rates |
| `/terrain/obstacles` | `MarkerArray` | Terrain obstacles |

---

## Quick Reference

### Common Commands

```bash
# Build
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# Run simulation
./scripts/run_sim_pid.sh
./scripts/run_sim_lqr.sh
./scripts/run_sim_mpc.sh

# Headless mode
./scripts/run_sim_pid.sh headless

# Monitor topics
ros2 topic echo /state/odom
ros2 topic hz /cmd/body_rate_thrust

# Check nodes
ros2 node list

# Run diagnostics
./scripts/check_sim.sh
```

### Configuration Locations

- PID: `ros2_ws/src/controllers_pid/config/pid_params.yaml`
- LQR: `ros2_ws/src/controllers_lqr/config/lqr_params.yaml`
- MPC: `ros2_ws/src/controllers_mpc/config/mpc_params.yaml`
- Safety: `ros2_ws/src/safety_gate/config/safety_params.yaml`
- Sim: `ros2_ws/src/sim_dyn/config/sim_params.yaml`
- Terrain: `ros2_ws/src/terrain_generator/config/terrain_params.yaml`

---

## Additional Resources

- **📖 [Example Usage](EXAMPLE_USAGE.md)** - Complete walkthrough of using controllers with generated terrain
- **Hardware Details**: See `docs/HARDWARE.md` for detailed TX/RX integration
- **Firmware**: See `firmware/README.md` for the ESP-NOW / ELRS / LoRa / GPS project index
- **Protocol Visualizer**: See `tools/README.md` for monitoring tools

---

## Support

For issues, check:
1. This guide's Troubleshooting section
2. Build output: `colcon build --event-handlers console_direct+`
3. Node logs: Check terminal output when launching
4. Topic monitoring: `ros2 topic echo /topic_name`

---

**Last Updated**: 2026-01-26
