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
| [`wing/`](wing/) | Flying wing bench sketch in two profiles — `single_wing` (1 motor) and `twin_wings` (2 motors, differential thrust) — 2 elevon servos, MPU6050 + NMEA GPS | ESC/servo PWM |

**This is not a supported flight configuration and must not become the baseline
for the airframes its profiles are named after.** Board names in its
`platformio.ini` are build targets, not qualified hardware. It contains its own
mixer, attitude controller and GPS guidance — so unlike the projects above, it
*does* assume a vehicle. Its two profiles now share that logic instead of
duplicating it, which is why they are one project: the duplication is gone, the
safety problem is not.

What it lacks, with the line that shows it:

| Missing | Where it shows |
|---------|----------------|
| A bounded loop period. `loop()` ends in `delay(20)`, so the real period is 20 ms *plus* execution time, while the controller is handed a hardcoded `0.02f` dt | `wing/src/main.cpp:82`, `:74` |
| Any handling of IMU failure beyond silence: the raw stick demands go straight to the mixer with no attitude limit and no annunciation | `wing/src/main.cpp:75` |
| An arming state machine. `grep -rn 'arm' firmware/wing/src/ \| grep -v '//'` produces no output | whole project |
| Bounded input buffering. The serial line and the NMEA line both accumulate into an Arduino `String` with no length cap | `wing/src/control_input.cpp:49`, `sensors.cpp:151` |
| Integrated attitude. Roll and pitch are `atan2` of accelerometer axes, so they are invalid under acceleration | `wing/src/sensors.cpp:128` |
| A correct CRSF length/CRC span. The length byte is read as excluding the CRC (`2 + frameLength + 1`) and the check covers `frameLength` bytes | `wing/src/rc_input.cpp:81`, `:89` |
| Receiver failsafe inspection before SBUS channels are taken | `wing/src/rc_input.cpp` |

These are meant to be checked rather than believed. Each is one command away —
for the first row, `grep -n '0.02f\|delay(20)' firmware/wing/src/main.cpp` prints
four lines: the two code lines (`:74`, `:82`) and the two header-comment lines
that name them (`:27`, `:28`). The numbers are from the consolidation revision,
which renumbered `main.cpp` and `control_input.cpp`; an earlier citation of
`twin_wings/src/main.cpp:99`/`:91` or `control_input.cpp:37` refers to the same
statements before the merge. `sensors.cpp` and `rc_input.cpp` were not touched by
it, so their numbers are unchanged.

Both profiles compile from these same files, so every row above applies to
`single_wing` and `twin_wings` alike, at one set of line numbers rather than two.

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

That script also runs the firmware's host tests at the end, after the last board
build. Today that is one test — the wing mixer's arithmetic, compiled for the
machine rather than for a board (see [`wing/README.md`](wing/README.md#host-test))
— and it is invoked there so that it runs whenever anyone builds the firmware
rather than only when someone remembers to.

The current firmware target matrix is:

| Project | Environments | Kind |
|---------|--------------|------|
| `wing` | `wing_single_esp32c3`, `wing_twin_esp32c3`, `wing_single_esp32`, `wing_twin_esp32`, `wing_single_f411`, `wing_twin_f411`, `wing_single_f405`, `wing_twin_f405` — four board ports × two vehicle profiles | bench demonstration |
| `elrs` | `elrs_tx`, `elrs_rx` | link |
| `lora` | `lora_433`, `lora_868`, `lora_915` | link |
| `espnow` | `transmitter`, `receiver` | link |

## Airframe support

The link and telemetry projects above are airframe-agnostic. Airframe-specific
behavior (mixing, allocation, control laws) is not part of them — see the
airframe table in the [root README](../README.md#supported-airframes).

`wing/` is the exception, and is the only project here that is not covered by
that statement: it carries its own mixer, attitude controller and guidance, it is
a bench demonstration, and it is not a supported flight configuration for any
airframe. See
[Airframe demonstrations](#airframe-demonstrations--not-flight-controllers)
above.
