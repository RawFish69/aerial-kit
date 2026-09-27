# Fixed-wing firmware control notes

> **These notes describe bench demonstrations, not a flight controller.** They
> document the control/mixing conventions used by `firmware/wing`, with PX4,
> ArduPilot and Betaflight as references rather than as code to copy. Nothing
> here is a supported flight configuration. See
> [`../firmware/README.md`](../firmware/README.md#airframe-demonstrations--not-flight-controllers)
> for what that project lacks, with file and line.

This is the working reference for `firmware/wing`. It documents the control/mixing
conventions to use, with PX4, ArduPilot, and Betaflight as references rather than
as code to copy.

## Conventions

- Body frame: forward `+X`, right `+Y`, down `+Z` (FRD), matching the rest of
  the repo where practical.
- Roll input positive = right wing down / right roll.
- Pitch input positive = nose up.
- Yaw input positive = nose right.
- Throttle is 0..1.
- Servo outputs are normalized 0..1 with 0.5 neutral until a real PWM driver is
  added.

## Twin Motor Flying Wing — profile `twin_wings`

Reference mixing:

```
motor_left  = throttle + yaw * differential_gain
motor_right = throttle - yaw * differential_gain

elevon_left  = neutral + pitch * pitch_gain - roll * roll_gain
elevon_right = neutral + pitch * pitch_gain + roll * roll_gain
```

This matches the differential-thrust-plus-elevon model used by flying-wing
mixers in ArduPilot/PX4: yaw is produced by asymmetric thrust, while elevons
handle pitch and roll. Keep the elevon sign convention consistent with the
servo installation before the sketch moves a real control surface.

## Single Motor Flying Wing — profile `single_wing`

Reference mixing:

```
motor = throttle

elevon_left  = neutral + pitch * pitch_gain - roll * roll_gain
elevon_right = neutral + pitch * pitch_gain + roll * roll_gain
```

Single-motor wings have no differential-thrust yaw authority, so yaw is
produced by bank-and-pull (coordinated turns) rather than a mixer term. Both
blocks above are `akMix()` in `src/mixer.cpp`, selected by `AK_MOTOR_COUNT`; the
single-motor profile compiles the yaw term out rather than passing it a zero.

## What was built, and what that does not mean

This list is a record of what was added to the sketch, not a plan whose
completion would produce a flight-capable vehicle. Each item is marked with what
actually exists, at the line numbers of `57e1811` in files that revision still
matches — a revision that predates the consolidation of `single_wing/` and
`twin_wings/` into `wing/`, so a citation below that names one of those
directories is a historical pointer to where the code came from, not a path that
exists now. None of the four is a claim of flight readiness: what is present is
present and unvalidated, and the items that remain are not the reason the sketch
cannot fly.

1. Replace `analogWrite` with a proper PWM driver: 50 Hz for servos, an
   ESC-safe rate for motors, with correct neutral pulse widths. *(Done in
   `src/pwm_output.*`. The neutral and travel values are still placeholders.)*
2. Add RC input (CRSF/SBUS/PPM) or autopilot command input. *(Bench serial input
   exists in `src/control_input.*`; CRSF and SBUS parsers exist in
   `src/rc_input.*`. The CRSF length and CRC span are wrong — at
   `src/rc_input.cpp:81` and `:89` since the two projects were consolidated, and
   at `:80` and `:88` in the `single_wing` copy that no longer exists — and SBUS
   does not inspect the receiver failsafe flags, so neither parser is a route to
   flight.)*
3. Add an attitude/rate controller. *(A proportional-plus-rate-damping-plus-integral
   controller exists in `src/controller.*`; L1/TECS-lite helpers exist in
   `src/guidance.*`; a GPS course/waypoint navigation module exists in
   `src/navigation.*`, and the waypoint source is `WAYPOINT_LAT`/`WAYPOINT_LON`/
   `WAYPOINT_ALT_M` in `src/config.h`, called before the controller from
   `main.cpp`. All gains are placeholders. Attitude is accelerometer `atan2`
   (`src/sensors.cpp:128`), which is invalid under acceleration, and the loop's
   real period is `delay(20)` plus execution time against a hardcoded `0.02f`
   dt — so tuning these against PX4 `fw_att_control`/`fw_pos_control_l1`,
   ArduPlane's TECS or Betaflight's fixed-wing PID would be measuring the wrong
   thing.)*
4. Add IMU and GPS drivers behind a small sensor interface. *(Interfaces exist in
   `src/sensors.*`, and drivers exist too: a dependency-free MPU6050 I2C driver
   and a minimal NMEA parser in `src/sensors.cpp`. An earlier revision of this
   line said hardware drivers were still TODO; that was stale. The IMU driver
   is what supplies the accelerometer attitude noted in item 3.)*
