# AerialKit - the first flight

The bring-up checklist ends with a board that answers questions on the bench -
[05-bringup.md](05-bringup.md) for the WeAct F405, and
[26-ghf435-bringup.md](26-ghf435-bringup.md) for the wing's own controller -
and everything after that is this page: the
order in which an aircraft that has never flown gets into the air, what to watch
while it is there, and what to do when it does something unexpected.

**Nothing in this firmware has flown.** The gains are the numbers a simulator
liked, no motor has turned, no servo has moved and no part has been read over a
wire - so the first flight is not a test of the firmware so much as the first
measurement of it, and the procedure is built around that: one change at a time,
and the log is what decides.

## Before anything is armed

1. **The bench checklist, all of it.** `make bench-check PORT=/dev/ttyACM0` (or
   `/dev/ttyUSB0` for the ESP32) types [05-bringup.md](05-bringup.md)'s list at
   the console and writes down the answers. It should end `failed: 0`, with the
   items it calls *unverified* being the ones that need a person: the sensor-bus
   loopback jumper and the calibrations.
2. **The preflight.** `preflight` prints the machine's own account of itself.
   The lines to read before a first flight are `outputs: N motors, M servos on
   this board, and the mix needs ...` - a mix this board cannot drive is why an
   aircraft would refuse to arm, and that is the line that says so before the
   refusal - `airframe N: the <mix> mix
   and a <profile> return` - which is a *check*, not a setting, because the
   airframe parameter chooses both - and `gyro bias: measured ...` or `not
   measured yet`, because those are different claims about the same flight
   ([15-preflight.md](15-preflight.md)).
3. **The calibrations, on the aircraft as it will fly** - with the pack in, the
   canopy on and the props off: `calibrate` (gyro, and the aircraft still),
   `calibrate vbat <the multimeter's number>` (the pack, with the meter in
   hand), `calibrate rc` (sticks centred, which is the step that catches a
   receiver a few counts off centre), and `calibrate accel 0..5` (six faces,
   which is what makes the attitude mean something absolute). `save` afterwards,
   or the next power-up loses them.
4. **The log, emptied**: `log flash clear` with the props off, so the first
   flight's log is the first flight's.
5. **The servos, with the horns on and the surfaces moving**: `output` prints
   what each servo is set to, and a hand on the surfaces says whether the
   elevons go the way the mixer thinks. `servo1_reverse` and its two siblings
   are the fix for a linkage that goes the other way or a centre that is not
   where it should be - a wing with one elevon reversed does not turn at all
   ([07-outputs.md](07-outputs.md)).

## On the ground, with props off (or the servo horns disconnected)

These are the four ways an untested aircraft hurts somebody, and each one is
cheaper here than in the air:

| Check | Command | What to see |
| --- | --- | --- |
| The outputs are on the pins the mixer thinks | `output test` | each motor in turn, the two servos, and nothing while armed |
| The four directions of the mixer | `output test` with the aircraft held | [05-bringup.md](05-bringup.md) 3b: stick right rolls right, stick forward pitches down, yaw right turns right, and on a wing both elevons move together for pitch |
| The failsafe by *unplugging the receiver* | arm, throttle up, then pull the receiver's cable | the motors stop (the log's `state` column says `failsafe`); a receiver in its own failsafe is the other half, and `rc` prints the flag |
| The return, held in the hand | `set rth_enable 1`, arm, switch off (or pull the link) | the navigator engages and the outputs move as the airframe page says - [14-navigation.md](14-navigation.md), and it needs the aircraft **armed** to move anything |

Then the pack: `battery` should agree with a multimeter, and `vbat_ratio` should
be corrected until it does ([19-battery.md](19-battery.md)).

## The first flight

Short, and in this order, whichever airframe it is. The point of the first
flight is *the log*, not the flight: everything else is easier to change once
there is a recording of what the aircraft actually did.

### Which motor is which, and which way each turns

A quadrotor's mixer is a claim about *corners*, and getting one wrong is a
first-flight flip rather than a tuning problem - so before anything is armed,
put the frame's own numbers against the wiring. On a four-motor board - which
is every one that flies a quadrotor, and so every board this page is about -
the default frame is `airframe 0` (`quadx`), because that is what those boards
answer `ak_board_default_airframe()` with. A board whose pads are a wing's comes
up on the wing instead, and `preflight` says which frame the flight is on rather
than leaving it to this page. The `quadx` table says, row by row:

| Output | Corner | Turns |
| --- | --- | --- |
| motor 1 | rear right | clockwise |
| motor 2 | front right | **counter-clockwise** |
| motor 3 | rear left | **counter-clockwise** |
| motor 4 | front left | clockwise |

Two rules make that checkable without a diagram: **the diagonal pairs match**
(1 with 4, 2 with 3), and **the pair that turns counter-clockwise is the pair
the mixer raises for yaw-right** - the `+1` in the yaw column of rows 2 and 3.
`ak_mixer.h` states the same thing in the code ("yaw right raises the pair this
table assumes turns counter-clockwise"), and which way a *particular* motor
turns is set by the prop and the ESC wiring, so swapping two props and two
leads is the fix if a corner is backwards.

**And if the frame is numbered the other way** - plenty of boards call the front
left motor 1 - that is not a rewire, it is `airframe 2` (`quadx1234`), which is
the same aircraft with the motors numbered from the other corner and a table of
its own (`ak_mixer_quad_x_1234`). The two tables' numbers are both checked
against Betaflight's own tables in `tests/test_mixer_parity.c`, so which of the
two to select is a question about the *wiring*, not about which one is right.

### A quadrotor

1. Arm with the throttle down (the switch held for `arm_hold_ms`, 500 ms by
   default) and **angle mode** on the mode channel: the stick asks for a tilt
   and the aircraft levels itself, which is the mode to find out whether the
   directions are right. If the switch does nothing, the console says why, in
   one line - `arm: refused - the throttle is at 30 per cent`, or the tilt, or
   the receiver - and `status` prints the same line without touching the
   switch ([15-preflight.md](15-preflight.md)). **A refusal that is about the
   tilt is the one to stop on**: it means the aircraft is not level, or the
   board is not mounted the way the alignment parameters say, and that is a
   bench fix rather than a flight one.
2. Hover a metre up for ten seconds. Nothing else - no translation, no
   altitude hold, no navigator.
3. Land, disarm, pull the log: `log` (the fast ring, 1.5 seconds at 250 Hz) and
   `log flash` (the whole flight at 5 Hz). The fast ring is the one to read for
   the control loop; the flash log is the one that has the altitude, the yaw
   *and the position* of the flight ([11-blackbox.md](11-blackbox.md)).
4. What the log is for: the `torque_*` columns show what the loop asked for and
   the `motor*` columns show what the mixer could give, so a loop that is
   fighting something looks like one of them pegged while the other moves. The
   `roll`/`pitch` columns show whether the aircraft was where the stick asked.

### A fixed wing

1. Arm with the throttle down, then raise the throttle to the cruise the
   airframe wants while somebody holds it nose-up: the mixer drives both motors
   and the elevons should be neutral with the sticks centred. The tilt gate
   applies to this airframe too, so arm it **level** and raise the nose after -
   or, if the launch attitude is genuinely more than `arm_max_tilt_deg` (25
   degrees) from level, raise that parameter deliberately: 180 is the whole sky
   and turns the check off ([04-flight-core.md](04-flight-core.md)).

   Or let the firmware hold it: `set launch_channel 8` (any spare channel),
   arm with the throttle down, and throw the switch - the launch spools the
   motors and holds `launch_climb_deg` for `launch_timeout_s`, and a stick
   takes it back. That is the manoeuvre written for exactly this moment, and it
   is worth using for the first throw because the alternative is holding a
   pitch stick with the hand that is also holding the wing
   ([24-launch.md](24-launch.md)). **Check it on the ground first, with the
   props off**: throw the switch and watch the motors run up and the elevons
   pitch up, then take it back with the pitch stick and watch them stop.
2. Launch it straight and level and let it fly a circuit at altitude with the
   sticks. No navigator, no return, no fence.
3. Land, disarm (or the throttle down and catch it), and pull the log as above.
4. The wing's own first-flight question is the *elevon signs* under power: a
   wing that pitches up when the loop asks for down is a handful, and it looks
   exactly like a stabilised aircraft with the gain too high.

## Tuning, in one order

Every number below is a parameter, so every change is `set <name> <value>` at
the console, a flight, and a log - and the order matters because a later loop
cannot be judged while an earlier one is wrong:

1. **The rate loop's P** (`rate_kp_*`): fly in *rate* mode and step the sticks.
   The airframe should reach the rate that was asked for and stop there; a
   wobble that grows is P too high, and a rate that arrives late and sags is P
   too low. The simulator's rate-mode session (`aerialkit-fw-sim 18 rate`) is
   the rehearsal for this, and the numbers it flies are the defaults.
2. **The rate loop's D** (`rate_kd_*`, filtered by `d_cutoff_hz`, 80 Hz): stops
   a fast bounce that the P alone cannot. Raise it until the bounce stops, then
   back off - noise in the gyro columns is D too high, and it is the term that
   heats motors.
3. **The angle loop** (`angle_kp_*`, `angle_rate_limit`, `max_tilt_deg`): fly in
   angle mode and command a tilt. It should reach it and hold it without
   wandering.
4. **The altitude and the navigator** (`rth_*`, `quad_*`, `rth_alt_ki`,
   `quad_alt_ki`): only once the aircraft flies well by hand. The return is the
   last thing to trust, and its own page has the profiles and what each gain
   does ([14-navigation.md](14-navigation.md)).

One change per flight, and the log from *that* flight, or the next change is
being tested against two of them.

### What the aircraft's own tune has that this firmware does not

The wing flies on INAV today, and its configuration is in this repository
(`projects/twin-wings/ghf435-inav/`). Reading it against this firmware's loop is
the cheapest way to know what the first flight's log will look like, because
every row below is something INAV does that we do not - and each one makes the
*same* airframe quieter or smoother than ours will be:

| The aircraft's INAV configuration | This firmware |
| --- | --- |
| `gyro_anti_aliasing_lpf_hz = 250` - a low-pass on the gyro itself | `gyro_lpf_hz` - the same job, a first-order filter, and **0 by default**: at 250 the loop's closed-loop check stops settling, because 0.6 ms of lag in the *proportional* path is not free and this aircraft's gains are not tuned. `set gyro_lpf_hz 250` is the first thing to try once a log shows the noise |
| `dterm_lpf_hz = 110`, `dterm_lpf_type = PT2` - a second-order low-pass on D | `d_cutoff_hz` (80 Hz, first order) - the same idea, one order and one knob less |
| `fw_ff_pitch = 50`, `fw_ff_roll = 50`, `fw_ff_yaw = 60` - feed-forward from the sticks | none. The P and I do all of the work, which is why the ladder starts with P and why a stick step is the interesting test |
| `iterm_windup = 50` | `rate_i_limit` (0.3 of an axis) - the same protection, written as a clamp rather than a decay |
| `airmode_type = STICK_CENTER`, `airmode_throttle_threshold = 1150` | the mixer's `motor_idle` keeps a quadrotor's props turning with the throttle at zero, which is what Airmode is *for*; there is no separate mode ([07-outputs.md](07-outputs.md)) |
| `rc_smoothing = ON` | none: a stick step reaches the setpoint as one step |

**So the first flight has less margin than the aircraft had on INAV**, and the
ladder is written for that: P first and low, watch for the oscillation the D
filter cannot hold back, and read the *fast* log (250 Hz) rather than the long
one - the numbers in it are the loop's own, and the noise the reference's
filters would have removed is what it shows. The gyro filter is *there* and off
(`gyro_lpf_hz`): the log is what says whether to turn it on, and the reason it
starts off is in its own row above rather than in a hunch - a first-order 250 Hz
filter at a kilohertz loop is 0.6 ms of lag in the loop's most sensitive path,
and this aircraft's gains have never been tuned.

**One difference that is history rather than current**: the firmware the board
*shipped* with (Betaflight 2025.12.2, captured in the same directory) had
`dshot_bidir = ON` - motor RPM telemetry and the RPM filter built on it. INAV
9.1.0's configuration for this board does not, and this firmware implements
neither: the DShot frame *asks* for telemetry (`ak_dshot_pack`'s telemetry bit)
and nothing reads the answer. Worth knowing because RPM filtering is the quiet
fix a person might expect to find here, and it is not.

## When something goes wrong

- **Stop first.** The arm switch is the pilot's; the firmware's own failsafe -
  no link, no attitude, a flat pack with `battery_rth` on, the fence - stops the
  motors or brings the aircraft home, and all of them are described where they
  live. None of them needs the pilot to diagnose anything in the air.
- **Then pull the log before powering off.** The fast ring is in RAM and does
  not survive a power cycle; the long ring survives a reset and not the battery;
  the flash log survives everything, which is why it exists
  ([11-blackbox.md](11-blackbox.md)).
- **Then read the state column**, which is the first question. It is a number,
  and it is the same one `status` prints as a name: `failsafe` is the firmware
  having stopped it, `returning home` and `on autopilot` are a navigator flying
  it (one because something failed, one because somebody asked), `armed` is the
  pilot. The `flags` column is which measurements were alive when the record was
  written, so "the gyro stopped answering" and "the receiver went away" are
  different rows rather than different guesses - the bits are in
  [11-blackbox.md](11-blackbox.md).

## What the simulator has and has not rehearsed

Every session in [18-software-in-the-loop.md](18-software-in-the-loop.md) is a
rehearsal of one of the steps above, and `make test` flies all of them on every
build: the sticks and the mixer, the return with wind and with a dead barometer,
the GPS going quiet, a flat pack, a bad fix, a fix that lies, the fence, the
mission on both airframes, a gyro that has warmed up, and noisy sensors. What it
cannot rehearse is everything in [01-plan.md](01-plan.md)'s "what is not
measured": a real ESC, a real servo, a real gyro, a stall, a gust that is not
uniform, and an airframe whose signs are somebody else's wiring.

That list is the reason the first flight is short. It is also the reason the
firmware keeps a log.
