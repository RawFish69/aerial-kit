# Hardware Integration

## Overview

TX_RX handles hardware. ROS provides autonomous algorithms.

**Manual flight**: TX with IMU+joystick. No ROS needed.  
**Autonomous flight**: ROS → TX via UDP. Requires TX modification.

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
ros2 launch hw_bridge hw_px4.launch.py mavlink_url:=udpin:0.0.0.0:14540
```

> Betaflight path assumes **Angle mode** (stick commands represent tilt angles).
> Tune `hover_throttle` and mapping gains before real flight.

## TX UDP Receiver Code

In TX_RX/src/main.cpp, `#ifdef BUILD_TX` section:

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
cd TX_RX
pio run -e transmitter -t upload

# Connect computer to UAV_TX WiFi (192.168.4.1)

# Launch ROS (CRSF hardware backend)
cd UAV-Controller
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
