# mavlink_bridge

Real-flight-controller backend adapter for UAV-Controller: talks MAVLink to
PX4 or ArduPilot and implements the same `/uav/backend/*` contract as the
sim adapters in `sim_bridge` (`gazebo_backend_adapter_node`,
`fastsim_backend_adapter_node`), so `command_manager_node` /
`mission_executor_node` fly a real vehicle with no changes upstream.

## Wiring

This is **not** the ELRS RC link. The ELRS firmware in `../ELRS` carries
CRSF RC channels (roll/pitch/yaw/throttle + switches) over the air to the
flight controller's RC input — it does not carry MAVLink. `mavlink_bridge`
needs a **separate, direct MAVLink connection** between the companion
computer running this node and the FC:

- USB: companion computer USB port to the FC's USB/UART port (e.g.
  Pixhawk `/dev/ttyACM0`).
- Direct UART: companion computer UART pins wired to an FC telemetry port
  (`TELEM1`/`TELEM2` on PX4-family boards), typically at 57600 or 921600
  baud depending on how that port is configured on the FC side.
- Telemetry radio (e.g. SiK radio pair): one radio on the companion
  computer's USB/serial, one on the FC's telemetry port. Same
  `connection_url` mechanism, just a different serial device.

Genuine ExpressLRS does support a MAVLink-passthrough mode for backhauling
telemetry over the RC radio to a ground station, but that's a separate
feature from this project's current ELRS firmware (CRSF-only today) and
isn't the intended path for autonomous offboard velocity setpoints anyway
— use a direct link as above.

## Dependency

`pymavlink` is not currently packaged elsewhere in this repo. Install via
rosdep (`python3-pymavlink`) if your distro has the key, otherwise:

```bash
pip install pymavlink
```

## Connection strings

`connection_url` is passed straight to `mavutil.mavlink_connection`, so it
works unchanged across real hardware and simulation:

| Target | `connection_url` |
|---|---|
| Real FC over USB | `/dev/ttyACM0` (set `baud` to match, e.g. 57600) |
| PX4 SITL | `udp:127.0.0.1:14540` |
| ArduCopter SITL | `udp:127.0.0.1:14550` |

Override via a custom params file (copy
`config/mavlink_bridge_default.yaml`) and:

```bash
ros2 launch mavlink_bridge real_hardware.launch.py \
  mavlink_bridge_params_file:=/path/to/your_config.yaml
```

## Safety model

- **The bridge never switches flight mode.** It only streams velocity
  setpoints (`SET_POSITION_TARGET_LOCAL_NED`, body-frame) and relays
  arm/disarm from `/uav/backend/enable`. You put the FC into OFFBOARD
  (PX4) or GUIDED (ArduPilot) yourself via the RC transmitter's mode
  switch, and can flip back to a manual mode at any time regardless of
  what the ROS graph is doing.
- **Disarm is safety-gated.** If `enable=False` arrives while the FC
  reports nonzero speed or altitude above `disarm_max_speed_mps` /
  `disarm_max_altitude_m`, the bridge withholds the disarm command and
  logs a warning instead of cutting motors mid-flight, retrying once the
  vehicle is slow and low.
- Reported `armed` state (in `/uav/backend/telemetry_raw` and the disarm
  interlock) comes from the FC's own `HEARTBEAT.base_mode` bit, not from
  this node's memory of what it last requested — so a pilot-triggered
  arm/disarm from RC is reflected correctly too.
- The FC's own RC failsafe/kill-switch is independent of this bridge and
  remains your primary safety net.

## Pre-flight checklist

1. **Unit tests**: `colcon test --packages-select mavlink_bridge` (pure-math
   frame-transform tests, no hardware needed).
2. **SITL**: point `connection_url` at a PX4 or ArduCopter SITL instance,
   launch `real_hardware.launch.py`, confirm `/uav/backend/odom` and
   `/uav/backend/telemetry_raw` populate, confirm arm/disarm behave as
   expected (including the disarm interlock while SITL reports motion).
3. **Props-off bench test on real hardware**: connect the FC, arm via
   `/uav/backend/enable`, publish a small positive `angular.z` on
   `/uav/backend/cmd_twist` and confirm the FC reports yaw moving the
   expected direction. If it's backwards, flip `yaw_rate_sign` in your
   params file (default `-1.0`). Also sanity-check `linear.x`/`linear.y`
   forward/right sign the same way.
4. Only after 1–3 pass: a real flight, low-altitude/tethered first.
