# single_wing

> **Bench demonstration. Not a flight controller, and not a supported flight
> configuration.** This sketch exists to check wiring, PWM rates, servo
> directions and mixer signs on a bench. Do not flash it to an aircraft. See
> [`../README.md`](../README.md#airframe-demonstrations--not-flight-controllers)
> for what it lacks, with file and line.

PlatformIO firmware for a **Single Motor Flying Wing**:

- 1 motor through ESC
- Aileron/elevator or elevon servos
- MPU6050 IMU over I2C, NMEA GPS over UART

This is a near-duplicate of `twin_wings/`: the two share their sensor driver, RC
parsers, navigation and PWM layer, and their controllers differ only by the yaw
terms a single motor has no authority for. Every warning in the twin-wing
directory applies here, at the line numbers in this project's own `src/`.

## What this is not

An earlier revision of this file called the project a scaffold whose control loop
and sensor drivers were "the next layer to add". Those layers were added, so the
sentences that said so have been removed rather than left to age. What replaced
them is the honest version of the same warning: the layers are present, they are
unvalidated, and nothing here is a path to a flight-capable vehicle. The
attitude controller, the guidance and the CRSF/SBUS parsers are this project's
own and are shared with nothing; flight-control development for these airframes
is not part of this repository.

The gains in `src/controller.cpp` and `src/guidance.cpp` are placeholders, not
tuned values. The comments there say "tune before flight" because that is the
conventional phrasing; read it as "these numbers are unmeasured", not as an
indication that tuning them here would produce a flight-capable vehicle.

## Targets

| Environment | Board | MCU |
|-------------|-------|-----|
| `single_wing_esp32c3` | `esp32-c3-devkitm-1` | ESP32-C3 |
| `single_wing_esp32` | `esp32dev` | ESP32 WROOM |
| `single_wing_f411` | `blackpill_f411ce` | STM32F411CE |
| `single_wing_f405` | `genericSTM32F405RG` | STM32F405RG |

## Build and upload

```bash
cd firmware/single_wing
pio run -e single_wing_esp32c3
pio run -e single_wing_esp32c3 -t upload
```

STM32 targets use the STM32duino Arduino core and may require selecting an upload protocol
for your board (ST-Link, DFU, serial). Adjust `platformio.ini` for your wiring.

## PWM output

`src/pwm_output.cpp` provides a small board-aware PWM layer:

- ESP32: LEDC at 50 Hz for servos and 400 Hz for the motor, 12-bit duty.
- STM32: `analogWrite` with 50 Hz / 400 Hz setup via `analogWriteFrequency`.

The servo neutral, travel, and ESC arming/calibration values still need to be tuned for
your actual hardware before the sketch moves a real control surface. Tuning them makes the
servos move correctly; it does not make this a flight controller.

## Bench control input

`src/control_input.cpp` accepts a simple serial command for bench testing:

```text
roll,pitch,throttle
```

Example: `0.2,-0.1,0.35` followed by Enter. This is the input the sketch is
built to exercise on a bench, and it is the only one whose behaviour has been
observed. CRSF and SBUS parsers exist alongside it (`src/rc_input.cpp`), but the
CRSF length and CRC span are wrong and SBUS ignores the receiver failsafe flags —
so they are not a route to flight, and neither is this.

## CRSF RC input

`src/rc_input.cpp` provides `rcInputReadCrsf(Stream&, ControlInput&)` for parsing
CRSF `RC_CHANNELS_PACKED` frames. The channel mapping is CH1 roll, CH2 pitch, CH3
throttle. Assign a receiver UART in `main.cpp` when ready.

## SBUS RC input

`src/rc_input.cpp` also provides `rcInputReadSbus(Stream&, ControlInput&)`. Set
`RC_INPUT_PROTOCOL` to `2` in `src/config.h`; the init helper uses inverted UART on
ESP32. On STM32, SBUS may require an external inverter circuit.

## Sensors

`src/sensors.h` defines the IMU and GPS interfaces. `src/sensors.cpp` now contains a
dependency-free MPU6050 driver over I2C (address `0x68`, ±8 g accel, ±500 deg/s gyro) that
populates `ImuData`, plus a minimal NMEA parser for GPS (`$GPGGA` fix/lat/lon/alt and
`$GPRMC` groundspeed). IMU I2C and GPS UART pins are in `src/config.h`.

## Controller

`src/controller.cpp` contains a proportional attitude controller with gyro-rate damping and
integral trim (`kpRoll`/`kpPitch`, `kdRoll`/`kdPitch`, `kiRoll`/`kiPitch`). It maps
roll/pitch errors to normalized actuator commands before `applySingleWingMix()` converts
them to motor/servo outputs. The gains are placeholders. Attitude comes from
accelerometer `atan2` (`src/sensors.cpp:128`), so it is invalid under
acceleration, and the loop runs on a hardcoded `0.02f` dt while the real period
is set by `delay(20)` in `main.cpp`. Neither is a tuning problem.

## GPS guidance

Set `ENABLE_GPS_GUIDANCE` to `1` in `src/config.h` and fill `WAYPOINT_LAT` /
`WAYPOINT_LON` / `WAYPOINT_ALT_M`. When a GPS fix is available, `guidanceToTarget()` in
`src/navigation.cpp` will override the bench serial input with L1/TECS commands to the
configured waypoint.
