# Firmware

All ESP32 / PlatformIO projects in this repo live here. Each subfolder is an
independent PlatformIO project with its own `platformio.ini`, built with
`pio run -d firmware/<project>`.

| Project | Purpose | Radio / bus |
|---------|---------|-------------|
| [`espnow/`](espnow/) | Custom TX/RX link: IMU + joystick manual flight, and autonomous command relay from the ROS 2 stack | ESP-NOW (2.4 GHz), outputs CRSF / SBUS / PPM / iBus to the flight controller |
| [`elrs/`](elrs/) | ExpressLRS-compatible TX/RX for autonomous flight — computer sends CRSF over UART to TX, RX emits CRSF to the FC | SX1280 2.4 GHz FLRC |
| [`lora/`](lora/) | Point-to-point LoRa template for long-range, low-rate telemetry or a backup command channel | SX1276/SX1278/RFM9x (433/868/915 MHz) |
| [`gps/`](gps/) | GPS bring-up and telemetry module (NMEA + PMTK + UBX) | UART to GPS module |

Host-side tools that talk to these boards over serial live in [`../tools/`](../tools/).

## Build

```bash
# Install PlatformIO, then build any project from the repo root:
pio run -d firmware/espnow
pio run -d firmware/elrs
pio run -d firmware/lora
pio run -d firmware/gps

# Upload a specific environment
pio run -d firmware/gps -e gps_auto -t upload
```

Environment names are defined per project — see each project's `platformio.ini`
and README.

A containerized PlatformIO toolchain is available: see [`../docker/README.md`](../docker/README.md).

## Airframe support

These projects are airframe-agnostic: they carry RC channels and telemetry, and
do not assume a particular vehicle. Airframe-specific behavior (mixing,
allocation, control laws) lives in the control stack, not here — see the
airframe table in the [root README](../README.md#supported-airframes).
