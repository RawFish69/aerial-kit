# Hardware Integration

## Overview

Firmware (`firmware/`) handles the radio link and flight-controller I/O. ROS provides the
autonomous algorithms.

**Manual flight**: TX with IMU+joystick. No ROS needed.  
**Autonomous flight**: ROS → TX via UDP. Requires TX modification.

This page covers the **Betaflight / CRSF** path over this repo's own ESP32 TX/RX. For
**PX4 or ArduPilot over MAVLink**, use `mavlink_bridge` instead — see
`ros2_ws/src/mavlink_bridge/README.md`. Both paths present the same `/uav/backend/*`
contract, so the control stack above them is identical.

## Direct CRSF host link

The Aerial Kit ELRS USB TX Rev A3 can also accept CRSF directly from a host. This
is separate from the older ESP-NOW/UDP bridge below:

```text
ROS velocity command -> crsf_radio_backend_node -> CRSF UART -> ELRS TX ~~ RF ~~> ELRS RX
FC CRSF telemetry    <- crsf_radio_backend_node <- CRSF UART <-----------------------+
```

The host UART is 420000 baud, 8N1, non-inverted and full duplex. Pin mapping,
the need for a 3.3 V USB-UART adapter, role-image selection and native USB-CDC
limits are in
[`firmware/elrs-usb-tx-rev-a3/docs/HOST-LINK.md`](../firmware/elrs-usb-tx-rev-a3/docs/HOST-LINK.md).

The channel map is AETR on channels 1-4, arm on channel 5, and flight mode on
channel 6. Arm is asserted only while `/uav/backend/enable` is true. Disabled
commands use minimum throttle and disarm; a stale command while enabled uses
neutral hover sticks and remains armed.

Decoded telemetry continues to use the existing hardware topics:

- Attitude -> `/uav/hw/imu`
- GPS -> `/uav/hw/gps`
- Battery -> `/uav/hw/battery`
- Flight mode -> `/uav/radio/flight_mode`

RSSI, link quality and SNR are additive JSON on `/uav/radio/link_stats`; no
existing message field was changed. `/uav/radio/status` carries human-readable
transport status.

### Radio failsafe

The ROS backend replaces a command older than `command_timeout_sec` (0.5 s by
default) with neutral hover sticks while `/uav/backend/enable` remains true. On
the radio side, the simulated receiver disables the plant after
`receiver_timeout_sec` (0.35 s) without a fresh channel frame; an airborne
multirotor backend then auto-lands instead of holding its last command.

The real flight controller's own failsafe remains authoritative. Configure it
to stop motors or perform the intended controlled landing when CRSF input goes
stale; do not configure hold-last-command. Aerial Kit firmware documents its
link-loss behavior in
[`firmware/docs/04-flight-core.md`](../firmware/docs/04-flight-core.md). Every
real FC must be verified with props removed; the SIL result is not a substitute
for that bench check.

The stick mapping and `hover_throttle` described here are **multirotor-specific**. Fixed-wing
and other airframes need their own mapping; that work is tracked in the roadmap in the
[root README](STACK-GUIDE.md#roadmap).

## ROS-Side Hardware Contract

Hardware autonomous flight is wired through the same backend contract used by sim:

- Commands from control stack:
  - `/uav/backend/cmd_twist` (`geometry_msgs/msg/Twist`)
  - `/uav/backend/enable` (`std_msgs/msg/Bool`)
- Estimated state back to control stack:
  - `/uav/backend/odom` (`nav_msgs/msg/Odometry`)

For Betaflight CRSF mode, the estimator consumes:

- `/uav/hw/imu` (`sensor_msgs/msg/Imu`)
- `/uav/hw/baro` (`std_msgs/msg/Float64`, altitude meters)
- `/uav/hw/gps` (`sensor_msgs/msg/NavSatFix`)

Launch paths:

```bash
ros2 launch hw_bridge hw_crsf.launch.py udp_host:=192.168.4.1
ros2 launch mavlink_bridge real_hardware.launch.py
```

> The old `ros2 launch hw_bridge hw_px4.launch.py` is legacy/deprecated; use
> `mavlink_bridge` for PX4 and ArduPilot (see `ros2_ws/src/mavlink_bridge/README.md`).

> Betaflight path assumes **Angle mode** (stick commands represent tilt angles).
> Tune `hover_throttle` and mapping gains before real flight.

## TX UDP Receiver Code

In `firmware/legacy/espnow/src/main.cpp`, `#ifdef BUILD_TX` section:

```cpp
// At top with includes
#include <WiFi.h>
#include <WiFiUdp.h>

// With TX configuration
WiFiUDP udp;
const int UDP_PORT = 9000;
bool udpActive = false;

// In setup(), after protocol init:
void setup() {
  // ... existing setup ...
  
  // Setup WiFi AP for ROS connection
  WiFi.softAP("UAV_TX", "uav12345");
  Serial.printf("TX IP: %s\n", WiFi.softAPIP().toString().c_str());
  
  udp.begin(UDP_PORT);
  udpActive = true;
  Serial.printf("UDP listening on port %d for ROS commands\n", UDP_PORT);
}

// In loop(), before other RC sending logic:
void loop() {
  unsigned long now = millis();
  CustomProtocol_Update();
  
  // Check for ROS commands via UDP
  if (udpActive) {
    int packetSize = udp.parsePacket();
    if (packetSize == 20) {  // DirectCommandPayload size
      uint8_t buffer[20];
      udp.read(buffer, 20);
      
      // Parse: 4 floats + 1 uint32
      float* floats = (float*)buffer;
      float roll = floats[0];      // -1.0 to 1.0
      float pitch = floats[1];     // -1.0 to 1.0
      float yaw = floats[2];       // -1.0 to 1.0
      float throttle = floats[3];  // 0.0 to 1.0
      
      // Convert to CRSF channels and send (yaw is inverted to match Betaflight)
      rcChannels[0] = (uint16_t)(roll * 819.0 + 992.0);
      rcChannels[1] = (uint16_t)(pitch * 819.0 + 992.0);
      rcChannels[2] = (uint16_t)(172.0 + throttle * 1639.0);
      // Left rotation must be negative to align with Betaflight's convention
      rcChannels[3] = (uint16_t)((-yaw) * 819.0 + 992.0);
      
      CustomProtocol_SendRcCommand(rcChannels);
      lastRcSendMs = now;
    }
  }
  
  // ... existing IMU/joystick or bench test code ...
}
```

### Flash and Run

```bash
# Flash TX
pio run -d firmware/legacy/espnow -e transmitter -t upload

# Connect computer to UAV_TX WiFi (192.168.4.1)

# Launch ROS (CRSF hardware backend)
cd aerial-kit
source ros2_ws/install/setup.bash
./scripts/run_crsf_link_pid.sh udp_host:=192.168.4.1
```

## Packet Format

DirectCommandPayload: 20 bytes, little-endian `<ffffI`
- 4 floats: roll, pitch, yaw (-1..1), throttle (0..1)
- 1 uint32: timestamp

## Troubleshooting

**TX not receiving**: Check WiFi connection, ping 192.168.4.1  
**RX not responding**: Check ESP-NOW link, CRSF wiring (GPIO 21)  
**No FC response**: Verify FC set to CRSF input

## Follow-up Integration Work

The outbound UDP/TX command path above is active. Hardware sensor topic population is a separate
integration step:

- GPS serial/NMEA -> `/uav/hw/gps`
- Betaflight telemetry (attitude/baro) -> `/uav/hw/imu` + `/uav/hw/baro`

Those bridges/firmware hooks are intentionally separate from the ROS backend package and can be
implemented incrementally on the hardware side.
