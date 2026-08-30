# TX/RX System

**ESP32-based transmitter/receiver with hybrid IMU+Joystick control and universal protocol support**

---

## ⚡ Quick Start

### 1. Hardware Setup

**Transmitter:**
- ESP32 + IMU (BNO085 or MPU6050) + Joystick
- IMU: GPIO 4 (SDA), GPIO 5 (SCL)
- Joystick: GPIO 0 (Throttle), GPIO 1 (Yaw), GPIO 6 (Button)

**Receiver:**
- ESP32 only
- UART: GPIO 21 (TX to FC), GPIO 20 (RX from FC)
- PPM: GPIO 5 (if using PPM protocol)
- LED: GPIO 8 (status: blinking = no TX link, solid = connected)

### 2. Configure

**All settings in ONE file:** `src/config.h`

```cpp
// TX: Choose IMU type
#define IMU_TYPE IMU_BNO085  // or IMU_MPU6050

// RX: Choose output protocol
#define OUTPUT_PROTOCOL PROTOCOL_CRSF  // or SBUS, PPM, IBUS, FRSKY_SPORT
```

### 3. Build & Flash

```bash
# Transmitter
pio run -e transmitter --target upload

# Receiver
pio run -e receiver --target upload
```

### 4. Flight Controller Setup

**For CRSF/SBUS/iBus (Betaflight/INAV):**
1. Connect ESP32 TX (GPIO 21) → FC RX pin
2. In FC: Enable Serial RX, set protocol (CRSF/SBUS/IBUS)
3. Save and reboot

**For PPM:**
1. Connect ESP32 GPIO 5 → FC PPM input
2. In FC: Set Receiver Mode to PPM
3. Save and reboot

---

## 🎮 Control Scheme

| Input | Controls | Notes |
|-------|----------|-------|
| **IMU Tilt** | Roll + Pitch | Tilt the transmitter to control |
| **Joystick Up/Down** | Throttle | Push up for more throttle |
| **Joystick Left/Right** | Yaw | Left/right for rotation |
| **Button Press** | ARM/DISARM | Toggle arming (AUX1) |

---

## 🔧 Configuration (`src/config.h`)

### Transmitter Settings

```cpp
// -------- IMU --------
#define IMU_TYPE IMU_BNO085              // IMU_MPU6050 or IMU_BNO085
#define ROLL_SENSITIVITY 45.0            // Degrees for full deflection
#define PITCH_SENSITIVITY 45.0

// -------- Joystick --------
#define JOYSTICK_THROTTLE_PIN 0
#define JOYSTICK_YAW_PIN 1
#define JOYSTICK_BUTTON_PIN 6

// -------- Communication --------
#define RC_SEND_FREQUENCY_HZ 100         // 50, 100, 150, 250, 500 Hz
```

### Receiver Settings

```cpp
// -------- Protocol Selection --------
#define OUTPUT_PROTOCOL PROTOCOL_CRSF    // Change this!
// Options: PROTOCOL_CRSF, PROTOCOL_SBUS, PROTOCOL_PPM, 
//          PROTOCOL_IBUS, PROTOCOL_FRSKY_SPORT

// -------- Pins --------
#define UART_TX_PIN 21                   // For UART protocols
#define UART_RX_PIN 20
#define PPM_OUTPUT_PIN 5                 // For PPM protocol

// -------- LED --------
#define LED_PIN 8                        // Status indicator
```

---

## 📡 Supported Protocols

| Protocol | Type | Flight Controllers | Baud Rate | Pins |
|----------|------|-------------------|-----------|------|
| **CRSF** | UART | Betaflight, INAV, ArduPilot | 420000 | 21, 20 |
| **SBUS** | UART | All (universal) | 100000 | 21, 20 |
| **PPM** | GPIO | Traditional FCs | N/A | 5 |
| **iBus** | UART | FlySky FCs | 115200 | 21, 20 |
| **FrSky** | UART | FrSky receivers | 57600 | 21, 20 |

**To switch protocols:** Change `OUTPUT_PROTOCOL` in `config.h` and reflash.

---

## 📁 Project Structure

```
firmware/espnow/src/
├── config.h ⭐              # ALL configuration here
├── main.cpp                 # Main TX/RX logic
├── protocol.h/cpp           # ESP-NOW wireless
├── espnow_TX/RX.h/cpp       # Wireless handlers
├── imu_handler.h/cpp        # IMU (TX only)
├── joystick_handler.h/cpp   # Joystick (TX only)
├── protocol_base.h          # Protocol interface
└── universal_bridge.h/cpp ⭐ # ALL 5 output protocols
```

---

## 🔍 Calibration

### IMU Calibration

**BNO085:** No calibration needed - automatic!

**MPU6050:**
1. Power on transmitter
2. Keep still on level surface for 2 seconds
3. Wait for "Calibration complete!" message

### Joystick Calibration

Usually automatic. If drift occurs:
```cpp
// Increase deadband in config.h
joystick.setYawDeadband(0.10);  // Default: 0.05
```

---

## 🎯 Tuning

### IMU Sensitivity

Adjust in `config.h`:
- **Too sensitive (hard to control)**: Increase to 60-90°
- **Too slow (not responsive)**: Decrease to 30-40°

```cpp
#define ROLL_SENSITIVITY 45.0   // Adjust this
#define PITCH_SENSITIVITY 45.0  // And this
```

### Update Rate

```cpp
#define RC_SEND_FREQUENCY_HZ 100  // Options: 50, 100, 150, 250, 500
```

- **50 Hz**: Maximum range
- **100 Hz**: Balanced (recommended)
- **150-250 Hz**: Sport/racing
- **500 Hz**: Ultra-low latency

---

## ⚠️ Troubleshooting

### No Link Between TX and RX
- Check both devices are powered
- Verify both on same WiFi channel
- Check ESP-NOW initialization in Serial Monitor

### IMU Not Found
- Check wiring: SDA → GPIO 4, SCL → GPIO 5
- Try swapping SDA/SCL pins
- Verify correct IMU_TYPE in config.h
- Check I2C address (MPU6050: 0x68, BNO085: 0x4A)

### Flight Controller Not Responding
- **UART protocols**: Verify TX→RX connection (GPIO 21 → FC RX)
- **PPM**: Verify GPIO 5 → FC PPM input
- Check protocol matches in FC configuration
- Verify correct baud rate (CRSF: 420000, SBUS: 100000, iBus: 115200)

### Joystick Drift
```cpp
// Increase deadband in config.h
joystick.setYawDeadband(0.10);
joystick.setThrottleDeadband(0.05);
```

### Controls Inverted
```cpp
// In main.cpp, TX section, add negative sign:
float rollNormalized = -imu.getRoll();   // Invert roll
float pitchNormalized = -imu.getPitch(); // Invert pitch
```

---

## 🚀 Advanced Features

### Protocol-Specific Settings

Each protocol has tunable parameters in `config.h`:

**CRSF:**
```cpp
#define CRSF_OUTPUT_FREQUENCY_HZ 100
```

**SBUS:**
```cpp
#define SBUS_OUTPUT_FREQUENCY_HZ 50
#define SBUS_INVERT_SIGNAL true
```

**PPM:**
```cpp
#define PPM_FRAME_LENGTH_US 20000
#define PPM_PULSE_LENGTH_US 400
#define PPM_CHANNEL_COUNT 8
```

### Adding New Protocols

1. Add enum to `config.h`:
```cpp
enum OutputProtocol {
    // ... existing
    PROTOCOL_NEW = 5
};
```

2. Add protocol class to `universal_bridge.cpp`
3. Add case to factory method

That's it! Only one file to modify.

---

## 📊 Channel Mapping

| Channel | Function | Source | Range | Notes |
|---------|----------|--------|-------|-------|
| CH1 | Roll | IMU | ±100% | Tilt left/right |
| CH2 | Pitch | IMU | ±100% | Tilt forward/back |
| CH3 | Throttle | Joystick | 0-100% | Up/down |
| CH4 | Yaw | Joystick | ±100% | Left/right |
| CH5 | AUX1 (Arm) | Button | Toggle | Press to arm/disarm |
| CH6-16 | Available | - | Center | For future use |

**CRSF Format:** 172-1811 (center: 992)

---

## 🔌 Wiring Quick Reference

### Transmitter
```
ESP32 GPIO 4 → IMU SDA
ESP32 GPIO 5 → IMU SCL
ESP32 GPIO 0 → Joystick VRY (Throttle)
ESP32 GPIO 1 → Joystick VRX (Yaw)
ESP32 GPIO 6 → Joystick SW (Button)
ESP32 3.3V   → IMU + Joystick VCC
ESP32 GND    → IMU + Joystick GND
```

### Receiver (UART Protocols)
```
ESP32 GPIO 21 → FC RX (Serial RX)
ESP32 GPIO 20 → FC TX (Serial TX, optional)
ESP32 GPIO 8  → LED + (with 220Ω resistor) [Blinking = No TX | Solid = Connected]
ESP32 GND     → FC GND + LED -
ESP32 Power   → FC 3.3V/5V or external
```

### Receiver (PPM Protocol)
```
ESP32 GPIO 5 → FC PPM Input
ESP32 GPIO 8 → LED + (with 220Ω resistor) [Blinking = No TX | Solid = Connected]
ESP32 GND    → FC GND + LED -
ESP32 Power  → FC 3.3V/5V or external
```

---

## 💡 Tips

- **BNO085 recommended** for IMU - no drift, better accuracy
- **Start with 100 Hz** update rate - balanced performance
- **Use debug mode** (`ENABLE_DEBUG_OUTPUT true`) for troubleshooting
- **LED indicator** (any color works):
  - **Blinking** (500ms) = Receiver on but no TX connection
  - **Solid ON** = Connected to TX and receiving signal
- **Protocol switching**: Just change config.h, reflash (30 seconds)
- **UART vs GPIO**:
  - UART protocols (CRSF/SBUS/iBus/FrSky): Use GPIO 21/20
  - GPIO protocols (PPM): Use GPIO 5
  - **No wiring changes** when switching between UART protocols!
- **For testing/debugging**: Use CRSF (can monitor via USB with `tools/`)
- **For flight**: Use whatever your FC needs

---

## 📚 Technical Details

### System Architecture

**Manual Flight:**
```
TX (IMU+Joystick) → ESP-NOW → RX → Protocol Bridge → FC
```

**Wireless:**
- ESP-NOW protocol (low latency, no WiFi setup needed)
- Automatic pairing
- Link quality monitoring

**Output:**
- Universal protocol bridge
- All 5 protocols in one file
- Automatic protocol selection from config

### Build Flags
- `BUILD_TX`: Transmitter (includes IMU/joystick code)
- `BUILD_RX`: Receiver (includes protocol bridges)

### Dependencies
- **TX**: Adafruit MPU6050, Adafruit BNO08x, WiFi
- **RX**: WiFi only

---

## 🐛 Debug Mode

Enable in `config.h`:
```cpp
#define ENABLE_DEBUG_OUTPUT true
```

Shows:
- RC channel values
- Link status
- Protocol name
- Frame counts
- Telemetry data

Disable for flight (reduces overhead):
```cpp
#define ENABLE_DEBUG_OUTPUT false
```

---

## 📝 Notes

- ROS 2 integration available (see main project README)
- Supports autonomous mode via UDP (TX firmware modification needed)
- All channel values in CRSF format (172-1811) internally
- Automatic conversion to protocol-specific formats
- Failsafe: Center sticks, min throttle, disarm on link loss

---

## 🎓 Summary

1. **Configure** in `config.h` (IMU type, protocol, pins)
2. **Flash** TX and RX
3. **Connect** RX to FC
4. **Setup** FC to match protocol
5. **Fly!**

Questions? Check `config.h` - all settings have inline comments!
