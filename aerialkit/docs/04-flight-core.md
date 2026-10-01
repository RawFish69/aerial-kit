# AerialKit - the flight core

`src/core/flight/` is the part of this firmware that would still make sense if
the aircraft were made of different silicon. It includes no MCU header, does no
I/O, allocates nothing, and calls into libc nowhere - not even `sqrtf`. That is
what lets the same code run in three places:

| Where | How | What it proves |
| --- | --- | --- |
| Host tests | `make test` | control laws, state machine, maths, against libm |
| The board, at boot | `ak_flight_selftest()` in `main()` | the same checks pass on the target, printed on the console |
| The board, later | the flight loop, once sensors exist | nothing yet - this is M2 onwards |

## Frames and units

Fixed here once, so that a sign error is a bug rather than a convention:

    body frame   x forward, y right, z down
    gyro         rad/s
    accel        g; z reads +1 with the board level and the right way up
    sticks       roll/pitch/yaw -1..1, throttle 0..1
    motors       0..1, servos -1..1 with 0 at centre
    angles       radians, rates rad/s

## The modules

| File | Job | Deliberately not there |
| --- | --- | --- |
| `ak_math.*` | `sqrtf`, `atan2f`, both ours, with accuracy stated and checked against libm on the host | sin/cos, exp, anything not yet needed |
| `ak_estimator.*` | complementary filter for roll and pitch | magnetometer yaw, EKF, GPS fusion, and any filtering of the rate the loop flies on - the gyro arrives through `ak_flight`'s own first stage (below) |
| `ak_altitude.*` | height above take-off from a barometer and a GPS | feed-forward, wind, an INS-grade filter |
| `ak_pid.*` | PID with derivative on measurement and a clamped integral | feed-forward, autotune, gain scheduling |
| `ak_rc.*` | receiver counts to sticks and switches | CRSF decode, SBUS, failsafe detection |
| `ak_crsf.*` | CRSF frame parsing, CRC-8/DVB-S2, 16 channels unpacked from 11-bit fields | telemetry frames beyond the envelope check |
| `ak_output.*` | motor value to DShot frame, servo value to pulse width | the timer and DMA that put edges on a pin |
| `ak_mixer.*` | mixer tables: quad-X, twin-motor elevon wing | curves, thrust linearisation, per-output limits |
| `ak_flight.*` | the loop: **the gyro's low pass (`gyro_lpf_hz`, off by default)**, the estimator, control, mixing, arming, failsafe, return and managed flight | position control, and the guidance profiles themselves; a second filter stage, feed-forward, or anything the reference's sharper filter chain does ([23-first-flight.md](23-first-flight.md) has what the aircraft's own tune had and what happened when this default was 250) |
| `ak_selftest.*` | the checks that run on the board and in the tests | anything needing a sensor |
| `ak_log.*` | the blackbox record, and the two RAM rings that hold it | where the records go, and how often |
| `ak_flashlog.*` | the same record, written into flash sectors that outlive the battery | erasing anything by itself - that is the caller's decision, on the ground |

Two host tools sit beside the tests, both built from the same core:

```bash
make host                                            # tests, replay, trace generator
build-host/aerialkit-gen-trace > docs/evidence/trace-hover.csv
build-host/aerialkit-replay quad-x < docs/evidence/trace-hover.csv
```

The input format is the shape a blackbox log will have, so that when the
blackbox exists, "replay the flight" is this program rather than a new one.

## Two kinds of test, on purpose

**Scripted input** (`tools/gen-trace.c` → `docs/evidence/trace-hover.csv`)
walks the state machine through arm, throttle up and a roll command at 250 Hz.
It answers "does the loop do the thing, in the right order, with the right
signs" and it is a file anyone can regenerate and diff.

**A closed loop against a toy plant** (in the host tests) is the only test that
can tell a stable loop from an unstable one. One number per axis - torque
becomes rate through a 30 ms lag - which is crude, but a controller that cannot
hold 15 degrees of roll against it has no business on an aircraft. Measured
result: commanded 15.0 deg, reached 15.1 deg, settled. That is a statement
about the controller and the toy plant, and about nothing else.

**Both flight laws, and for a long time only one of them.** The mode channel
picks between angle mode - the stick asks for a *tilt*, and an outer loop turns
that into a rate for the inner one - and rate mode, where the stick *is* the
rate and nothing levels the aircraft. Every host test passed angle mode and
every simulator session held the mode channel high, so the branch a pilot
actually flies a quadrotor with by hand had no evidence beyond the code
compiling. It has both kinds now: `test_flight_rate_mode` in the host tests
(the commanded rate on all three axes, in the direction the stick is pushed,
zero when the stick is centred even with the aircraft tilted, and a rotation
nobody asked for damped), and `aerialkit-fw-sim 18 rate` in the loop, where a
full roll stick asks for 400 deg/s and the airframe reaches 389, centring the
stick stops the rotation within a second - and the aircraft keeps the attitude
it ended at, which is the whole difference between the two modes.

## Mixing, and the authority limit

The mixer table says what each output gets. What it does not say is what happens
when the torque the control loop asked for does not fit under the throttle -
which is most of the time near the ends of the stick.

Clamping each motor on its own is the obvious thing and the wrong one: it throws
away the thrust the pilot asked for and hands back a different aircraft. At full
throttle a roll command becomes asymmetric thrust; at idle it becomes no yaw
authority at all. What this firmware does instead is reduce the *differential*:

```text
base   = clamp(throttle, motor_idle, 1)
d_i    = each motor's torque contribution
k      = min(1, (1 - base)/max(d), (base - motor_idle)/-min(d))
output = base + k * d_i
```

The thrust survives exactly - the mean of the motors is the throttle - and the
authority is what gets given up. At full throttle with a roll demand the mix
flattens to full throttle, which is what every flight controller does and nobody
enjoys.

Two details in that are airframe decisions, not arithmetic:

- **The idle floor is per airframe.** A quad's motors turn at 5% while armed:
  yaw is differential thrust, and stopped motors have none. A wing's motors
  stop, because there the throttle really is a throttle and its yaw comes from
  the elevons. `motor_idle` lives in the mixer table for that reason.
- **Servos are not scaled with the motors.** A wing at full throttle saturates
  its motors while its elevons still need full travel - they are what keeps it
  upright. Scaling them because the thrust ran out would be exactly backwards.

The idle floor is a mixer behaviour, not a safety feature: disarmed, the flight
core writes zeros itself and nothing in the mixer can talk it out of that.

## Parity with the reference implementations

Every sign in a mixer table is a claim about a physical aircraft, and the one
place a wrong one is invisible until it is airborne: a quad with its yaw column
inverted flies perfectly and turns the wrong way, and a quad with two rows
swapped pirouettes on take-off. The authority limit cannot catch either - there
is nothing malformed about a mix that is simply backwards.

So `tests/test_mixer_parity.c` compares our quad-X table against the reference
two ways, and they catch different things:

- **The tables, coefficient by coefficient**, against
  `tests/mixer_reference.h` - Betaflight's `mixerQuadX` transcribed with its
  revision, cross-checked against the same signs in INAV's target configs.
  Exact, and it would catch a swapped row, a flipped sign or a different motor
  order.
- **The arithmetic, over a grid of inputs** - our mixer's outputs against the
  reference's one-line formula, which is what would catch the table being right
  while the code that consumes it is not.

**And it found the yaw column inverted.** Our table had yaw `+1` on rear-right
and front-left; both references have `-1` there. Physically either works - which
diagonal spins counter-clockwise is decided by the props and the ESC wiring -
but a person who has built a quad before wires it the way every other flight
controller expects, and a table built for the other convention yaws the
aircraft backwards on its first flight. That is now the common convention, and
the check is what keeps it.

**And then it found the pitch column inverted, which the check could not see.**
The table was a verbatim copy of Betaflight's, and the convention underneath it
is not: Betaflight's pitch is positive nose *down* (its `attitude.values.pitch`
is computed that way, and `flight/imu.c` says "negative is backwards"), while
this firmware's pitch is positive nose *up* - its estimator computes it from
the accelerometer that way and its elevon mix uses it that way. A numeric
comparison against a table written for the other convention cannot see that;
it passed while the axis was inverted, and the sentence next to it said "nose
up raises the rear pair", which is not what a nose does when you speed the rear
motors up.

The simulator found it the moment its quad plant modelled thrust honestly -
rear motors up, nose down - as a **1,861 degree runaway** in pitch: the angle
loop drove the nose one way while the motors pushed it the other. What catches
it now is a check written in *physics* rather than in numbers: "nose up raises
the front pair", and its mirror "a nose-down command raises the rear pair".
Both are in `tests/test_mixer_parity.c`, next to the numeric comparison, which
now carries the convention explicitly (`MIXER_REFERENCE_PITCH_SIGN`) and says
why. The lesson is worth keeping: a parity test is a *convention* test as much
as a numbers test, and a convention that is only written down in prose is a
convention that can be wrong in every line of code at once.

### The frames, and what each one is for

The `airframe` parameter picks a row order and a yaw convention. What it does
*not* change is anything else: every frame below flies the same PID, the same
angle loop and the same mixer arithmetic, and each table is checked against the
reference implementation it came from, row for row, with the reference's
convention accounted for.

| # | `airframe` | Rows, in motor order | What it is |
| --- | --- | --- | --- |
| 0 | `quadx` | rear right, front right, rear left, front left | the common X, motor 1 at the rear right |
| 1 | `elevonwing` | left motor, right motor, left elevon, right elevon | the twin-motor flying wing - the one frame with no reference table, because it is this project's airframe |
| 2 | `quadx1234` | front left, front right, rear right, rear left | the same aircraft as #0 with the motors numbered 1..4 from the front left, which is how plenty of boards are wired |
| 3 | `quadp` | rear, right, left, front | a plus frame: one motor per arm |
| 4 | `y4` | rear top, front right, rear bottom, front left | two motors up, two down |
| 5 | `vtail4` | rear right, front right, rear left, front left | the rear arms swept into a V |
| 6 | `tri` | rear, right, left, then the tail servo | three motors and a servo that tilts the tail rotor for yaw |
| 7 | `elevonwingsingle` | motor, left elevon, right elevon | the same wing as #1 with one motor and **no rudder** - the frame whose yaw column is empty because nothing on the aircraft can produce yaw |

Which of these a board starts on is the board's answer and not this table's:
`ak_board_default_airframe()` supplies the `airframe` parameter's default, so
the Adafruit Feather F405 - two motor pads, two servo pads - comes up on 7 and
refuses to arm on anything needing four motors, while the four-motor boards come
up on 0. The list is in [07-outputs.md](07-outputs.md), beside the pad counts
that make it true.

Two deserve a word. `quadx1234` is not a different aircraft from `quadx`: the
check that says so is that the same command moves the same *physical* corner,
which is the only thing a person who has wired a quad before can rely on. And
`tri` is the one frame where a servo is part of the mix: the reference carries
the tail's tilt in its servo mixer and this firmware carries it as a fourth
row, so the yaw column reaches the servo and nothing else - and how far it
moves is the board's `servo1_travel_us`, because the travel is the linkage's
business rather than the frame's.

**And the single-motor wing is the frame whose empty cells are a statement.**
Every other zero in every other table means "this output is not used for that
axis", which is unremarkable - a quadrotor's motor rows carry no roll. This
one's yaw column means "this aircraft has no actuator that can produce yaw":
the twin above yaws with differential thrust, one motor is one row, and nothing
takes its place. Course changes are bank and pull, which is roll and pitch.

So the control law cannot simply multiply by the zero. It asks
`ak_mixer_has_axis()` whether the axis exists, and when it does not it zeroes
the setpoint, zeroes the torque and **resets the PID**, every tick
(`ak_flight.c`, `control_armed`). The reason is the integral: a loop handed a
demand it can never satisfy does not return zero, it returns a torque the mixer
discards *while keeping the error it accumulated producing it*, so a pilot
holding rudder winds up a term that is invisible for as long as the axis stays
unauthorised. The output would be identical either way - the column is zero -
which is why `tests/test_aerialkit.c` checks the loop's state rather than the
pulses, and flies the twin beside it so that "no yaw torque" cannot be confused
with a stick that was never moved.

What is deliberately missing is anything with more than four motors.
`AK_MAX_MOTORS` is four, so a hexacopter or an octocopter needs the output
count, the DShot channels, the blackbox record's motor field and the telemetry
frame widened *together*; that is a change of its own rather than a table, and
four motors is what this project's aircraft are.

The wing has no reference table to compare against: a twin-motor flying wing
with elevons is this project's airframe and neither upstream ships one. What is
checked there is that its conventions are the ones its own documentation claims
- elevons opposite for roll, together for pitch, thrust for yaw - and that they
agree with the quad on the axes they share.

## What the first run of the tests found

Found: four bugs, all in code that looked right and would have been expensive to find
on a bench:

1. **`ak_atan2f` was wrong on the left half of the circle** - by up to 4.7 rad,
   not by 2*pi, because folding `x` and adding `pi` does not work when `y` is
   also negative. The fix is to fold both axes into the first quadrant and undo
   the fold afterwards. The host test compares against libm over the whole
   circle, which is how it surfaced: worst error is now 7.7e-6 rad.
2. **The mixer indexed servos by row number**, so every servo row of a table
   that lists its servos before its motors landed on the wrong output - which
   is exactly the elevon-wing table. Rows are now consumed in order per kind.
3. **The CRSF parser had no way out of a half-finished frame.** Nothing in the
   byte stream says "that frame is over"; the only signal is silence, so the
   parser now abandons a partial frame after 5 ms of it. Without that, a
   receiver rebooting mid-frame leaves the parser waiting forever and the link
   stays dead until the aircraft is power cycled.
4. **The DShot crc folded the wrong value.** It has to cover the twelve bits
   `(throttle << 1) | telemetry`; folding the shifted frame instead drops the
   `>> 8` term and every crc is wrong, which an ESC ignores as a corrupt frame -
   a motor that simply never answers. Caught by hand-derived frames in the test
   (0xFFFF at full throttle with telemetry, 0x7D0A at throttle 1000) and then
   confirmed against Betaflight's `prepareDshotPacket` at `6dbc4218`, which
   builds the frame the same way.

None would have shown up as a build failure, and all four would have looked
like "the aircraft does something strange" on a bench.

## Who is flying

Six states, and the state says who is in charge:

| State | Who is flying | What ends it |
| --- | --- | --- |
| `disarmed` | nobody | the arm switch, throttle down, for half a second |
| `armed` | the sticks | the arm switch; a lost link with no navigator; a sensor that stops answering |
| `on autopilot` | the navigator, with the link still up | a stick moved past a fifteenth of travel, `mission stop`, the arm switch, disarming, a breach of the fence, or a pack gone critical |
| `returning home` | the navigator, because the link is gone | the link coming back, or the arm switch |
| `failsafe` | nobody, and it stays that way | the arm switch cycled off and on again |
| `circling down` | the flight core, flying a fixed wing down | the ground (the landing detector disarms it), or the arm switch cycled off and on again |

The rule underneath them is the one that makes a navigator an autopilot instead
of a hazard: **guidance never overrides a live link unless the pilot has handed
the aircraft over.** The navigator cannot decide that for itself - `managed` is
set by whoever owns the mission, and cleared by arming, by disarming, by the
failsafe, and by the pilot's own sticks. The same rule read the other way is why
a return is safe: with the link up, nothing the navigator says is listened to.

Losing the link on a mission is the interesting case, and it does not change who
is flying - the navigator was already flying the aircraft, so it keeps flying
the list. The console says `on autopilot` rather than `returning home` for
exactly that reason: two different things are happening, and a state name that
confuses them is a state name that costs somebody an aircraft.

There are three ways an aircraft ends up `on autopilot` while somebody is
holding the transmitter, and they are the same rule read three times: a mission
the pilot started, a fence the aircraft left, and a hand launch the pilot asked
for. The launch is the shortest of them - a climb attitude and a throttle for
two seconds - and it is the same guidance pointer with `managed` set as the
other two, which is why a failsafe can take it away: see
[24-launch.md](24-launch.md).

### And what "safe" means for each airframe, which the airframe answers

The row above is right about the *state*, and the outputs behind it are one
line: `outputs_safe()` sets every motor to zero and every servo to zero. For a
quadrotor that is the whole answer - a stopped motor is a stopped aircraft -
and it is what the bench checklist checks by pulling the receiver's cable.

**For a wing it is only half an answer, and this is the one place in the flight
core where the two airframes want different things.** Motors to zero is right
(a wing in a failsafe should not keep flying away), and centred elevons leave
the aircraft gliding rather than diving - but "gliding with nobody flying it"
is not the same as a landing, and the reference firmware for fixed wings is
explicit about it: INAV's failsafe for a wing holds the aircraft level and cuts
the throttle rather than leaving the surfaces centred, and its options are named
for what a wing can do (circle, drop, land) rather than for what a quadrotor
does. What this firmware does today is written in the code above.

**And it is implemented now, because the owner answered it.** The rule, chosen
on 2026-09-18, is *land as it circles down* - not the quadrotor's answer, and
not "cut the throttle and hope": a bank the attitude loop holds, a nose a little
down, and enough throttle to keep air over the elevons, circling until the
ground. It is flown through the same control law the pilot's sticks go through,
because a failsafe that flew a path of its own would be a second autopilot to
prove.

Which behaviour a lost aircraft gets is the *airframe's* answer, not a build
option and not the pilot's: `ak_mixer_t.fixed_wing` is what the failsafe reads,
so it cannot disagree with what the outputs are actually driving - a quadrotor
stops, a fixed wing circles. The three numbers that shape the circle are
parameters rather than constants, because the airframes do differ (a flying wing
with no rudder and a twin-motor elevon ship are not the same animal):
`wing_descend_bank_deg` (30), `wing_descend_pitch_deg` (8) and
`wing_descend_throttle` (0.15 - the one that matters most, because a wing at
zero throttle has no working control surfaces at all).

It is checked twice, and the two are different kinds of evidence. The boot
self-test - the same function a board runs at every power-on - arms a wing,
pulls its link and asserts that the state is `circling down` with the motors
still turning and the elevons still moving; and the simulator flies the whole
thing: `make test`'s thirtieth session, `docs/evidence/sil-wing-lost.txt`, has
the receiver come out over a wing with RTH off, and reports the console's own
"circling down", twelve metres of descent, a held bank of 30 degrees and a
held nose-down of 8, with the motors still turning when it reaches the ground.

## What has to be true before it arms

Arming is a list of gates, in the order a pilot can do something about them,
and one function - `ak_flight_arm_check()` - is the whole of it: the flight
core's arming path, the `arm:` line in `status`, the same line in the preflight
report and the sentence printed when a switch is thrown and nothing happens all
ask that function, so there is one answer to "would this aircraft arm".

| Gate | What it means |
| --- | --- |
| the board drives what the mix needs | the selected airframe's mixer against `ak_board_output_shape()`: a wing's two motors and two elevons need a board with two servos, and a quadrotor's four motors need four. **First in the list**, because it is the one gate a pilot cannot clear from the sticks |
| the link is up | a receiver that has spoken inside `rc_timeout_ms`; a board with none never arms |
| not latched in failsafe | coming back from a failsafe is the arm switch cycled off and on, not a link that reappeared |
| the arm channel is asking | channel 6 in this firmware's own channel order (`AK_RC_ARM`), above 1300 counts: the pilot's request, and not a gate that can be removed |
| the throttle is down | below `throttle_low`, 5 per cent by default |
| the attitude estimate has seen the accelerometer | `est.converged`; without it there is no attitude to fly on |
| the aircraft is the right way up | within `arm_max_tilt_deg` of level, 25 degrees by default, 180 for the whole sky |

**The first one is AerialKit's own**, and it exists because of the shape of this
project rather than the shape of an aircraft: the airframe is a *parameter* and
the board's outputs are a *build*, so the two meet for the first time on a
bench. A wing's mix on a board whose servos took the timer channels a
quadrotor's four motors would need is four motors that do not exist, and the
first symptom would otherwise be a wing that will not turn. The counts come from
the mixer table itself - its rows counted by kind, never written down beside it -
and from the board, once, at boot; a board that never says leaves the aircraft
unable to arm, because a gate a board can skip by staying quiet is not a gate.

The next four are as old as this firmware. **The last two are not, and the tilt
one was missing until it was measured**: a quadrotor held at 40 degrees of roll
with the throttle down and the switch on armed and then flew the correction with
the motors on their stops. The gate is the one both reference implementations
have - Betaflight refuses with `ARMING_DISABLED_ANGLE` against `small_angle`
(`src/main/fc/core.c`, `src/main/flight/imu.c`, default 25 degrees) and INAV
calls the same condition `ARMING_DISABLED_NOT_LEVEL`
(`src/main/fc/fc_core.c`) - and the number is theirs, because 25 degrees is
what everybody's props and pads are assumed to survive.

The link gate is worth a sentence of its own, because putting the gates in one
function made arming stricter in a way that had been invisible. A frame that
arrived and then went stale - the receiver unplugged, or its own failsafe
holding the last positions - still *decodes*, so the old arming test read the
switch position the sticks had three hundred milliseconds ago and could arm on
it. The gate reads `link_live`, which is the same freshness the failsafe uses,
and a stale frame is now `refused - the receiver has gone quiet`. Both
references have the same gate (`ARMING_DISABLED_RC_LINK` in INAV); AerialKit
had the freshness computed and did not ask it here.

Two things about it are deliberate. It is the check that catches **a board
alignment parameter that is wrong**, which is the failure a first flight cannot
survive and a bench can: a board mounted at 40 degrees reads 40 degrees of roll
while sitting still, and this is where that stops being a number in a parameter
table and becomes a refusal. And it is a *refusal*, not a warning, because the
alternative is an aircraft that lifts off with the estimator already disagreeing
with the ground.

What a pilot sees, in the loop and in the transcript
([evidence/sil-tilt.txt](evidence/sil-tilt.txt), session `20 tilt`):

```
sim:    812 ms  the aircraft is held at 60 degrees of roll
sim:   4001 ms  arm switch on, throttle down
arm:       refused - the aircraft is 60 degrees from level, and arming wants 25
state:     disarmed
motors:    0 0 0 0 per-mille
sim:   9001 ms  the switch goes off
sim:  10001 ms  put down level
sim:  12001 ms  arm switch on again, throttle down
sim:  14001 ms  throttle up, still held
state:     armed
motors:    552 552 546 546 per-mille
```

The refusal is printed **once per attempt and once per new reason**, not once
per pass: it appears when the switch goes on, and again if the reason changes
while it is held - so a pilot who lowers the throttle and is then told about the
tilt hears both. `status` prints the same sentence while the aircraft is
disarmed, which is how somebody who has not touched the switch finds out.

## Known limits, stated rather than implied

- **Yaw drifts, and the only correction is the track the aircraft makes through
  the air.** No magnetometer: yaw is gyro integration, and what pulls it back is
  the GPS course - corrected for the wind, because the module measures motion
  over the ground and in wind that is not the way the aircraft is pointing.
  That correction is what the navigator's standing term is for: it is the
  velocity the position loop has learned to fly at to hold station, which *is*
  the wind in the world frame, and subtracting it leaves the track the airframe
  actually made. Measured in the simulator before it existed: a quadrotor's
  mission ran with the yaw estimate 50 to 136 degrees away from the nose, and
  the aircraft chased its own tail with the motors on their stops. The
  remaining limits are honest ones: a drift alone no longer earns any
  alignment (its track through the air is nothing), a heading has to be earned
  by moving, and the estimator is still a complementary filter rather than a
  state estimator with the wind in it - see [14-navigation.md](
  14-navigation.md) for the nudge a quadrotor uses to earn one, and
  [ak_estimator.h](../src/core/flight/ak_estimator.h) for the interface.
- **The gains are not tuned**, and no sign in a mixer table has been checked
  against a real airframe. Every one of them is provisional until a bench test.
- **Failsafe is a stop unless a navigator is flying.** Motors to zero, servos to
  centre. With guidance available the flight core hands over instead and
  returns - see [14-navigation.md](14-navigation.md). A wing whose return has
  never been tested still gets the stop, because the stop is the default.
- **No attitude is not a state to fly in**, and that rule is now flown rather
  than only unit-tested: `aerialkit-fw-sim 20 noimu`
  ([evidence/sil-no-imu.txt](evidence/sil-no-imu.txt)) kills the gyro in flight
  at 24 m, and the motors stop **in the same millisecond** - the loop reads the
  IMU every pass, so the only delay is the loop itself. The part then comes
  *back*, and the state stays `failsafe`: the latch is the half of a failsafe
  that matters, because one that clears itself when the fault does is one that
  starts flying again on its own. What clears it is the arm switch, which is
  the same rule the console has always documented.
- **The navigator can be handed an aircraft whose link is up**, which is a
  different thing from a failsafe: the fence does it when the aircraft leaves a
  ring the pilot drew, and the pack does it when a cell goes below the critical
  threshold with `battery_rth` on ([19-battery.md](19-battery.md)). Both are off
  until somebody asks for them, both are visible on the console and the
  handset, and the flight core's rule is the same for both: guidance that
  somebody asked for is `on autopilot`, and guidance that appeared because
  something failed is `returning home`.
- **No thrust linearisation.** Motor commands are treated as linear, which is
  not true of an ESC and a prop, and it shows up as a slightly wrong mix at
  partial throttle.
- **A mission is started from a console *or* a switch.** `mission start` works
  over the UART or the network, and `mission_channel` names a receiver channel
  that starts and ends the same mission from the transmitter - which is the
  half that matters in the air, because a mode that can only be selected from a
  laptop is a mode nobody selects in the air. The channel wins while it is set,
  and the switch is what the mission session in the simulator uses. (This
  bullet said the switch was still missing; it landed with the mission, and the
  line stayed behind.)
- **The flight state lives on the stack** in the selftest (472 bytes of stack
  there, the largest single frame in the image). Once a scheduler exists, that
  state becomes static and the loop allocation-free by construction.
- **The CRSF and SBUS decoders are in the image, and the receiver UART is what
  calls them.** This bullet used to say the opposite - that nothing called the
  CRSF decoder, so `--gc-sections` dropped it and `nm` showed no `crsf` symbols
  at all. That was true when it was written and stopped being true when the
  receiver arrived. `nm` on the shipped F405 image now shows `ak_crsf_feed`,
  `ak_crsf_unpack`, `ak_sbus_feed` and the telemetry builders as defined text
  symbols, which is the check to run if this is ever in doubt again: a decoder
  that is dropped by the linker is a decoder the aircraft does not have.
- **Sign conventions are self-consistent, and the pitch axis was not.** Every
  angle, rate and mixer sign is *supposed* to agree with every other one, and
  the parity tests check it - but the quad's pitch column disagreed with the
  estimator for as long as the table was a copy of a table written for the
  opposite convention, and nothing in the numbers could see it (above). Roll
  and pitch are now checked in physical terms; **yaw is still a wiring
  question**: which diagonal of a quad spins which way is decided by the props
  and the ESC, and a wing's differential thrust is decided by its motors. The
  bench check is M2/M3: tilt the aircraft right and watch the roll estimate,
  then arm with the props off and check that each stick moves the surfaces the
  way [05-bringup.md](05-bringup.md) says. Until then, treat a yaw sign as
  unverified.
