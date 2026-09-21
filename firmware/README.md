# Firmware

All ESP32 / PlatformIO projects in this repo live here. Each subfolder is an
independent PlatformIO project with its own `platformio.ini`, built with
`pio run -d firmware/<project>`.

There are two kinds of project here, and the difference matters: the **link and
telemetry** projects are the supported radio path, and the **airframe
demonstrations** are bench sketches that fly nothing.

### Link and telemetry projects

| Project | Purpose | Radio / bus |
|---------|---------|-------------|
| [`espnow/`](espnow/) | Custom TX/RX link: IMU + joystick manual flight, and autonomous command relay from the ROS 2 stack | ESP-NOW (2.4 GHz), outputs CRSF / SBUS / PPM / iBus to the flight controller |
| [`elrs/`](elrs/) | ExpressLRS-compatible TX/RX for autonomous flight — computer sends CRSF over UART to TX, RX emits CRSF to the FC | SX1280 2.4 GHz FLRC |
| [`lora/`](lora/) | Point-to-point LoRa template for long-range, low-rate telemetry or a backup command channel | SX1276/SX1278/RFM9x (433/868/915 MHz) |
| [`gps/`](gps/) | GPS bring-up and telemetry module (NMEA + PMTK + UBX) | UART to GPS module |

These carry RC channels and telemetry. They are airframe-agnostic: they do not
assume a particular vehicle, and mixing, allocation and control laws are not
here.

### Airframe demonstrations — not flight controllers

| Project | Purpose | Output |
|---------|---------|--------|
| [`twin_wings/`](twin_wings/) | Twin Motor Flying Wing bench sketch: 2 motors through ESCs, 2 elevon servos, MPU6050 + NMEA GPS | ESC/servo PWM |
| [`single_wing/`](single_wing/) | Single Motor Flying Wing bench sketch: 1 motor through ESC, elevon/aileron/elevator servos, MPU6050 + NMEA GPS | ESC/servo PWM |

**These two are not supported flight configurations and must not become the
baseline for the airframes they are named after.** Board names in their
`platformio.ini` files are build targets, not qualified hardware. Each one
contains its own mixer, attitude controller and GPS guidance — so unlike the
projects above, they *do* assume a vehicle, and they duplicate safety-relevant
logic that is not shared with anything else.

What each one lacks, with the line that shows it:

| Missing | Where it shows |
|---------|----------------|
| A bounded loop period. `loop()` ends in `delay(20)`, so the real period is 20 ms *plus* execution time, while the controller is handed a hardcoded `0.02f` dt | `twin_wings/src/main.cpp:99`, `:91` |
| Any handling of IMU failure beyond silence: the raw stick demands go straight to the mixer with no attitude limit and no annunciation | `twin_wings/src/main.cpp:93` |
| An arming state machine. `grep -rn 'arm' firmware/twin_wings/src/ \| grep -v '//'` produces no output | whole project |
| Bounded input buffering. The serial line and the NMEA line both accumulate into an Arduino `String` with no length cap | `twin_wings/src/control_input.cpp:37`, `sensors.cpp:151` |
| Integrated attitude. Roll and pitch are `atan2` of accelerometer axes, so they are invalid under acceleration | `twin_wings/src/sensors.cpp:128` |
| A correct CRSF length/CRC span. The length byte is read as excluding the CRC (`2 + frameLength + 1`) and the check covers `frameLength` bytes | `twin_wings/src/rc_input.cpp:81`, `:89` |
| Receiver failsafe inspection before SBUS channels are taken | `twin_wings/src/rc_input.cpp` |

These are meant to be checked rather than believed. Each is one command away —
for the first row, `grep -n '0.02f\|delay(20)' firmware/twin_wings/src/main.cpp`
prints four lines: the two code lines (`:91`, `:99`) and the two header-comment
lines that name them. The `main.cpp` line numbers are from the revision that
carries this table, which is `57e1811` plus the header-comment correction in
`main.cpp`; the header grew by nine lines, so any earlier citation of `:92`/`:84`
refers to the same two statements at `57e1811`. `sensors.cpp`, `rc_input.cpp` and
`control_input.cpp` are unchanged from `57e1811`, so those numbers hold there too.

`single_wing/` is a near-duplicate of the same implementation — its
`controller.cpp` differs only by the yaw terms a single motor cannot use — so
every row above applies to it at the line numbers in its own `src/`.

They are kept because a bench sketch that moves a servo is genuinely useful for
wiring and mixer-sign checks. Nothing here should be flashed to an aircraft.
Flight-control development for these airframes is not part of this repository.

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

To build every firmware environment in one pass:

```bash
bash scripts/build_firmware.sh
```

The current firmware target matrix is:

| Project | Environments | Kind |
|---------|--------------|------|
| `twin_wings` | `twin_wings_esp32c3`, `twin_wings_esp32`, `twin_wings_f411`, `twin_wings_f405` | bench demonstration |
| `single_wing` | `single_wing_esp32c3`, `single_wing_esp32`, `single_wing_f411`, `single_wing_f405` | bench demonstration |
| `elrs` | `elrs_tx`, `elrs_rx` | link |
| `lora` | `lora_433`, `lora_868`, `lora_915` | link |
| `espnow` | `transmitter`, `receiver` | link |

## Airframe support

The link and telemetry projects above are airframe-agnostic. Airframe-specific
behavior (mixing, allocation, control laws) is not part of them — see the
airframe table in the [root README](../README.md#supported-airframes).

`twin_wings/` and `single_wing/` are the exception, and are the only projects
here that are not covered by that statement: they carry their own mixer,
attitude controller and guidance, they are bench demonstrations, and they are
not a supported flight configuration for any airframe. See
[Airframe demonstrations](#airframe-demonstrations--not-flight-controllers)
above.
