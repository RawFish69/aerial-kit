# Fixed-wing firmware control notes

> **These notes describe bench demonstrations, not a flight controller.** They
> document the control/mixing conventions used by `firmware/twin_wings` and
> `firmware/single_wing`, with PX4, ArduPilot and Betaflight as references rather
> than as code to copy. Nothing here is a supported flight configuration. See
> [`../firmware/README.md`](../firmware/README.md#airframe-demonstrations--not-flight-controllers)
> for what the two projects lack, with file and line.

This is the working reference for `firmware/twin_wings` and
`firmware/single_wing`. It documents the control/mixing conventions to use, with
PX4, ArduPilot, and Betaflight as references rather than as code to copy.

## Conventions

- Body frame: forward `+X`, right `+Y`, down `+Z` (FRD), matching the rest of
  the repo where practical.
- Roll input positive = right wing down / right roll.
- Pitch input positive = nose up.
- Yaw input positive = nose right.
- Throttle is 0..1.
- Servo outputs are normalized 0..1 with 0.5 neutral until a real PWM driver is
  added.

## Twin Motor Flying Wing (`twin_wings`)

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

## Single Motor Flying Wing (`single_wing`)

Reference mixing:

```
motor = throttle

servo_left  = neutral + pitch * pitch_gain - roll * roll_gain
servo_right = neutral + pitch * pitch_gain + roll * roll_gain
```

Single-motor wings have no differential-thrust yaw authority, so yaw is
produced by bank-and-pull (coordinated turns) rather than a mixer term.

## What was built, and what that does not mean

This list is a record of what was added to the two sketches, not a plan whose
completion would produce a flight-capable vehicle. Each item is marked with what
actually exists, at the line numbers of `57e1811` in files that revision still
matches. None of the four is a claim of flight readiness: what is present is
present and unvalidated, and the items that remain are not the reason the
sketches cannot fly.

1. Replace `analogWrite` with a proper PWM driver: 50 Hz for servos, an
   ESC-safe rate for motors, with correct neutral pulse widths. *(Done in
   `src/pwm_output.*`. The neutral and travel values are still placeholders.)*
2. Add RC input (CRSF/SBUS/PPM) or autopilot command input. *(Bench serial input
   exists in `src/control_input.*`; CRSF and SBUS parsers exist in
   `src/rc_input.*`. The CRSF length and CRC span are wrong — in `twin_wings` at
   `src/rc_input.cpp:81` and `:89`, in `single_wing` at `:80` and `:88` — and
   SBUS does not inspect the receiver failsafe flags, so neither parser is a
   route to flight.)*
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
