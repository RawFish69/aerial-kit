# wing

> **Bench demonstration. Not a flight controller, and not a supported flight
> configuration.** This sketch exists to check wiring, PWM rates, servo
> directions and mixer signs on a bench. Do not flash it to an aircraft. See
> [`../README.md`](../README.md#airframe-demonstrations--not-flight-controllers)
> for what it lacks, with file and line.

PlatformIO firmware for a **flying wing**, in two vehicle profiles:

| Profile | Actuators | Yaw |
|---------|-----------|-----|
| `single_wing` | 1 motor through ESC, 2 elevon servos | none — one motor has no yaw authority |
| `twin_wings` | 2 motors through ESCs, 2 elevon servos | differential thrust |

Both run the same sources. The profile is a compile-time flag that supplies the
pin map, the motor count and whether a yaw axis exists; everything else — sensor
driver, RC parsers, navigation, guidance, attitude controller, PWM layer and
mixer — is one implementation.

`single_wing/` and `twin_wings/` used to be separate projects. Each tracked 21
files, 16 of them in `src/`; 11 of those 16 differed and 5 were byte-identical.
`diff` over the whole project reported 207 changed lines, 140 of them in `src/`
(`diff -r firmware/legacy/single_wing firmware/legacy/twin_wings | grep -c '^[<>]'`). The
airframe difference underneath all of that came to one actuator and one mixer
term: `platformio.ini` differed by environment names alone, and `rc_input.h`,
`guidance.h` and `guidance.cpp` by comments alone. They are now one project with
two profiles, so the safety-relevant logic exists once and cannot drift between
airframes.

## Profiles

`src/profiles/airframe.h` requires exactly one profile to be defined and errors
out otherwise:

```c
#if defined(AK_AIRFRAME_SINGLE) && defined(AK_AIRFRAME_TWIN)
  #error "define only one of AK_AIRFRAME_SINGLE / AK_AIRFRAME_TWIN"
#elif defined(AK_AIRFRAME_SINGLE)
  #include "single_wing.h"
#elif defined(AK_AIRFRAME_TWIN)
  #include "twin_wings.h"
#else
  #error "no vehicle profile: define AK_AIRFRAME_SINGLE or AK_AIRFRAME_TWIN"
#endif
```

Each profile header defines the actuator pins per board family, the motor count,
whether the profile has yaw, and how many fields the bench serial command
carries:

| Macro | `single_wing` | `twin_wings` |
|-------|---------------|--------------|
| `AK_AIRFRAME_NAME` | `"single_wing"` | `"twin_wings"` |
| `AK_MOTOR_COUNT` | `1` | `2` |
| `AK_HAS_YAW` | `0` | `1` |
| `AK_CONTROL_VALUES` | `3` | `4` |
| ESP32 pins | motor 12, elevons 14/15 | motors 12/13, elevons 14/15 |
| STM32 pins | PA0, PA1/PA2 | PA0/PA1, PA2/PA3 |
| other pins | 3, 5/6 | 3/5, 6/9 |

Pins are placeholders until the real flight controller board is chosen. Board
names in `platformio.ini` are build targets, not qualified hardware. The pin
names were unified in the merge: the old `MOTOR_PIN` / `SERVO_L_PIN` /
`SERVO_R_PIN` are now `AK_MOTOR_PIN` / `AK_ELEVON_L_PIN` / `AK_ELEVON_R_PIN`.

### What the merge picked, and what it did not

Where the two copies differed, the merge kept the twin-motor one and made the
difference a compile-time one. Three files carry a yaw difference, and under
`single_wing` all three now compute a yaw the mixer discards:

| File | What `single_wing` gained |
|------|---------------------------|
| `src/controller.cpp` | `kMaxYawRateDegPerSec`, `kpYaw`/`kdYaw`/`kiYaw`, `yawIntegral`, `yawError`, and the `out.yaw` line |
| `src/rc_input.cpp` | CRSF and SBUS now decode CH4 into `out.yaw`; the single-motor copy left it unset |
| `src/navigation.cpp` | `guidanceToTarget()` now writes `out.yaw = 0.0f`; the single-motor copy did not write it |
| `src/guidance.cpp` | nothing — two comment lines, no code |

The actuator outputs are unchanged, and that is by inspection rather than by
hope. In `controller.cpp`, `yawError` and `yawIntegral` feed `out.yaw` and
nothing else, and the roll and pitch lines are byte-identical to the
single-motor file's; in `mixer.cpp` the single-motor path never reads `in.yaw`;
and the navigation write is a zero. Each file is one diff away:

```bash
for f in controller.cpp rc_input.cpp navigation.cpp guidance.cpp; do
  diff -u <(git show <base>:firmware/legacy/single_wing/src/$f) firmware/legacy/wing/src/$f
done
```

A merge that picked the copy with more channels *and* let one of those channels
feed a surface would be a different story; this one does not.

`src/control_input.cpp` is the other shape of the same question, and there the
merge kept both: each profile keeps its old parse, three fields for the
single-motor wing and four for the twin, chosen by `AK_CONTROL_VALUES`. The
single-motor field order is unchanged, so `0.2,-0.1,0.35` still means
roll, pitch, throttle and not roll, pitch, yaw.

## Mixing

`src/mixer.cpp` holds the one mixer. `akActuatorBegin()` starts every actuator
the profile uses; `akMix()` maps a `ControlInput` to PWM:

| Command | Effect |
|---------|--------|
| throttle | thrust (one motor, or both in common) |
| yaw | differential thrust — `AK_MOTOR_COUNT == 2` only |
| pitch | symmetric elevon deflection |
| roll | asymmetric elevon deflection |

Under `single_wing` the yaw term is compiled out rather than asserted, so the
single motor profile has no code path that could act on it.

## Host test

The arithmetic lives in `src/mix_math.h`, which has no Arduino dependency, so the
same functions the firmware compiles can be compiled and checked on the host:

```bash
firmware/legacy/wing/host_test/run.sh      # 430 checks, 0 failures on this revision
```

[`scripts/build_legacy_firmware.sh`](../../../scripts/build_legacy_firmware.sh) runs it too, after
the last board build, so the one command that builds every target also runs the
one test there is. It needs a host C++ compiler, which the board builds do not;
without one the script says so rather than passing quietly.

It checks the shape of the mix — which way each surface moves, that the elevons
mirror in roll, that nothing escapes its limits — and not the tuning. Three
seeded mixer bugs were planted to confirm it fails when it should: swapping the
roll term on one elevon (154 failures), reversing differential thrust (4), and
dropping a clamp (40). See [`host_test/README.md`](host_test/README.md) for why
it is not a `pio test`.

## Targets

Eight environments: four board ports × two profiles. Every one of them compiles
the same `src/`; only the profile flag differs.

| Environment | Board | MCU |
|-------------|-------|-----|
| `wing_single_esp32c3` | `esp32-c3-devkitm-1` | ESP32-C3 |
| `wing_twin_esp32c3` | `esp32-c3-devkitm-1` | ESP32-C3 |
| `wing_single_esp32` | `esp32dev` | ESP32 WROOM |
| `wing_twin_esp32` | `esp32dev` | ESP32 WROOM |
| `wing_single_f411` | `blackpill_f411ce` | STM32F411CE |
| `wing_twin_f411` | `blackpill_f411ce` | STM32F411CE |
| `wing_single_f405` | `genericSTM32F405RG` | STM32F405RG |
| `wing_twin_f405` | `genericSTM32F405RG` | STM32F405RG |

## Build and upload

```bash
cd firmware/legacy/wing
pio run -e wing_single_esp32c3
pio run -e wing_twin_esp32c3
pio run -e wing_single_esp32c3 -t upload
```

STM32 targets use the STM32duino Arduino core and may require selecting an upload protocol
for your board (ST-Link, DFU, serial). Adjust `platformio.ini` for your wiring.

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

## PWM output

`src/pwm_output.cpp` provides a small board-aware PWM layer:

- ESP32: LEDC at 50 Hz for servos and 400 Hz for motors, 12-bit duty.
- STM32: `analogWrite` with 50 Hz / 400 Hz setup via `analogWriteFrequency`.

The servo neutral, travel, and ESC arming/calibration values still need to be tuned for
your actual hardware before the sketch moves a real control surface. Tuning them makes the
servos move correctly; it does not make this a flight controller.

## Bench control input

`src/control_input.cpp` accepts a simple serial command for bench testing. The
field count follows the profile, so a three-field wing cannot silently read a
four-field line:

```text
single_wing:  roll,pitch,throttle        e.g. 0.2,-0.1,0.35
twin_wings:   roll,pitch,yaw,throttle    e.g. 0.2,-0.1,0.0,0.35
```

followed by Enter. This is the input the sketch is built to exercise on a bench,
and it is the only one whose behaviour has been observed. CRSF and SBUS parsers
exist alongside it (`src/rc_input.cpp`), but the CRSF length and CRC span are
wrong and SBUS ignores the receiver failsafe flags — so they are not a route to
flight, and neither is this.

## CRSF RC input

`src/rc_input.cpp` provides `rcInputReadCrsf(Stream&, ControlInput&)` for parsing
CRSF `RC_CHANNELS_PACKED` frames. The channel mapping is CH1 roll, CH2 pitch, CH3
throttle, CH4 yaw. Assign a receiver UART in `main.cpp` when ready.

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

`src/controller.cpp` contains a proportional attitude/rate controller. It maps
roll/pitch/yaw errors to normalized actuator commands with gyro-rate damping and
integral trim (`kp*`/`kd*`/`ki*` gains) before `akMix()` converts them to
motor/elevon outputs. The gains are placeholders. Attitude comes from
accelerometer `atan2` (`src/sensors.cpp:128`), so it is invalid under
acceleration, and the loop runs on a hardcoded `0.02f` dt while the real period
is set by `delay(20)` in `main.cpp`. Neither is a tuning problem.

## GPS guidance

Set `ENABLE_GPS_GUIDANCE` to `1` in `src/config.h` and fill `WAYPOINT_LAT` /
`WAYPOINT_LON` / `WAYPOINT_ALT_M`. When a GPS fix is available, `guidanceToTarget()` in
`src/navigation.cpp` will override the bench serial input with L1/TECS commands to the
configured waypoint.
