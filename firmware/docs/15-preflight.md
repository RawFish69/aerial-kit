> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - preflight

The boot banner says what the firmware intends. `preflight` says whether the
machine agrees.

```text
ak> preflight
ok    console attached to the board's own port
ok    clock on the crystal, system 168 MHz, apb1 42 MHz
ok    tick advanced after 41250 spins
ok    dshot 300 kHz, ARR 279, ccr 98/196, 1204 frames sent
ok    saved configuration: none stored
ok    no fault recorded since power on
ok    outputs: 4 motors, 2 servos on this board, and the mix needs 4 and 0
--    imu: icm42688p
--    receiver: 8412 bytes, 401 frames
--    gps: 41230 bytes, 401 nav-pvt, 1 configure attempts
--    blackbox: 384 records (wrapped)
--    link up, disarmed
--    arm:       ready
preflight: the machine is what the firmware thinks it is
```

## Why this exists

There is a class of fault that a build cannot catch and a host test cannot see,
because the port code is not in the host build: a console pointed at the wrong
pin, a timer that never started, a tick that does not tick, a configuration
sector that is damaged. From the outside they all look the same - the board does
nothing - and each one costs a bench session to find.

**A dead tick no longer *stops* the board, only the clock.** The delay that
waits on the tick is the one wait in the firmware that had no bound (every other
one - the console's UART, the flash controller, the sensor buses, the USB core -
gives up and says so), and an unbounded wait here is a board that puts its USB
device on the bus and then never speaks again, which is exactly what the F405
did on 2026-09-17. It gives up now after about a second of rounds and counts how
often, so the line above reads `FAIL  the millisecond tick is not running (3
delays have given up waiting for it)` and the rest of the boot carries on. The
bound is tested on the host: `tests/test_arch.c` drives `ak_wait_for_ticks()`
with a clock that never moves and with one that counts.

Two of them are not hypothetical. The console check exists because configuring a
UART and choosing the console used to be one operation, so initialising the
receiver left console output pointing at the receiver's pin: the firmware would
have booted silently. That was found by reading code, which is not a method that
scales. Now the boot line says so before anyone gets a serial adapter out.

## What is a failure and what is a fact

The check tells them apart on purpose. **Failures** are the ways the firmware is
wrong about itself: console on the wrong port, no crystal, a dead tick, outputs
configured at a rate nobody asked for, a damaged configuration sector, a
recorded fault. Any of those and `preflight` exits non-zero, so a script can use
it.

**Facts** are printed with `--`: no IMU fitted, no receiver frames, no GPS
messages, an empty blackbox. A board with no sensor on it is a board with no
sensor, and calling that a fault would teach everyone to ignore the output.

### The failure lines, executed

A check nobody can trip is a check nobody can trust, and until 2026-09-18 every
one of those FAIL branches had been **compiled and never run**: the simulator's
board was always the well-behaved one, so the lines existed and no test had ever
watched them come out. `aerialkit-fw-sim 6 badboard` is a board that lies - the
simulator makes one board call answer the way a broken one would, types
`preflight` after each, and the checks look for that failure's own line:

```text
sim: ok   a console on a port the board does not have is caught FAIL  console is on 0x40004800, but the board's console is 0x00000001
sim: ok   a crystal that never started is caught     FAIL  the clock is not on its crystal
sim: ok   a configuration whose record will not read is caught FAIL  the saved configuration is damaged (a bad length, or a checksum mismatch)
sim: ok   a crash from the run before is caught, with its own pc FAIL  a fault is recorded: pc 0xdead0f00 cfsr 0x00000082
sim: ok   timers running at the wrong rate are caught FAIL  outputs are at 302 kHz, 300 asked for
sim: ok   a dead millisecond tick is caught rather than hung on FAIL  the millisecond tick is not running (0 delays have given up waiting for it)
sim: ok   and an honest board passes the same command the clean line three times: at boot, and after each clean run
sim: ok   one wrong board fact, one refusal - and no more six SOMETHING IS WRONG lines for six lies
```

The first and last stages arm nothing, which is the other half of it: the same
command has to come back clean when the board is honest, or "it complained"
would be all the session could show. The stopped tick is worth naming - the
preflight's spin is bounded at five million reads, so a dead millisecond clock
is a **failure it reports**, not a board that hangs the check.

Two of the lies are facts about boot rather than switches a running command can
read, so they are words of their own:

```text
sim: ok   a board whose outputs never came up says so --    outputs: none on this board
sim: ok   and the rate is not asked of a timer that is not running
sim: ok   which is a fact rather than a fault, so the machine checks out
sim: ok   and nothing was refused
sim: ok   a two-motor board is told the mix does not fit it FAIL  this mix needs 4 motors and 0 servos; the board drives 2 and 2
sim: ok   and the arming gate refuses on the same two numbers
sim: ok   and that is the only thing wrong with it
```

The two-motor one is the ESP32-C3's shape - two RMT transmit channels - against
a quadrotor's mix, and it is the arming gate as much as the preflight: the
sentence a pilot reads is the same two numbers, which is what makes "it will not
arm" a diagnosis instead of a mystery. The no-outputs run lies about the DShot
rate as well, so the check that matters there is the line that is *absent*: a
timer that is not running is not asked what rate it is running at.

**A board whose outputs did not come up is a refusal, not a fact** (2026-09-18).
The `--    outputs: none on this board` line above is honest, and on its own it
was not enough: the boot used to state the board's *declared* shape whatever the
board's timers had done, so an aircraft whose outputs never came up reported the
right numbers, armed, and wrote DShot frames into channels that do not exist.
The ESP32 is where that could happen - its `ready` answer counted the servo
timer, and a quadrotor has no servos - so `ak_esp_output_ready()` now means "the
channels this board declares are up", and `main.c` states the shape only when
that is true. A board that is not ready states zero, and the two lines that
matter then say so:

```text
--    outputs: none on this board - the timers did not come up
FAIL  this mix needs 4 motors and 0 servos; the board drives 0 and 0
refused - this airframe's mix needs 4 motors and 0 servos, and the board drives 0 and 0
```

The boot says `preflight: 1 problem - type 'preflight'` before anybody types
anything, and the simulator's `badboard noout` session walks the whole chain.

And one thing the session proved by not being able to reach it: the other
outputs branch, `FAIL  the board has not said what its outputs are`, cannot be
produced at all while `main.c` states the board's shape at boot. It is a guard
for a port that never calls `ak_flight_set_board_outputs()`, and the session says
so rather than faking it. The transcripts are
[evidence/sil-preflight-lies.txt](evidence/sil-preflight-lies.txt),
[no outputs](evidence/sil-preflight-no-outputs.txt) and
[two motors](evidence/sil-preflight-two-motors.txt).

Back to the facts, two of them are about the flight rather than the board, and
both were added after the rest: **what the fix is worth to the navigator** (a fix can be
arriving, 3D, and still not one the navigator will use - see
[13-gps.md](13-gps.md)), and **whether the return the pilot may be trusting
would actually happen**. The second one has three ways to be impossible, and
they are the ways a pilot does not think of:

```text
--    fix: type 3, 11 satellites, hacc 1200 mm - usable
--    return: off - a lost link stops the aircraft
--    return: enabled, but no home yet - a lost link would stop it
--    return: enabled, but the fix is not usable - a lost link would stop it
--    return: ready, home set, 11 satellites
```

An aircraft that armed before it had ever seen a fix has **nothing to come back
to** - home is captured from the first fix while disarmed - and an aircraft
whose fix is not one the navigator will use has nowhere to navigate. In both
cases the aircraft would stop on a lost link, which is the right thing to do
and not what the pilot was expecting when they turned `rth_enable` on. The
checklist asks the question with the switch on and off, and the simulator's
return-with-a-bad-fix session asks it in the air: while the fix is degraded
mid-return, the transcript has
`--    return: enabled, but the fix is not usable - a lost link would stop it`.

### What the aircraft is, and what it will do

And two more, because the first flight is the one where the pilot cannot check
them by flying:

```text
ok    airframe 1: the elevon-wing mix and a fixed wing return
--    gyro bias: measured (500 samples): 12 -4 8 mdps
--    gyro bias: not measured yet - the stored bias stands
```

The **airframe line is a check, not a setting**. The `airframe` parameter
chooses two things - the mix in the flight core and the return profile in the
navigator - and they are set together in one function, so a disagreement should
be impossible. That is exactly why it is worth asking: a wing flying a
quadrotor's mix, or returning the way a quadrotor does, is a crash with a
plausible number in it, and the interface that sets them is the kind that grows
a second caller. A disagreement is a **failure**, not a fact: it means the
firmware is wrong about itself. (`ak_mixer_for_airframe()` is the single place
the number becomes a table, and `tests/test_mix.c` pins it.)

The **gyro line is a fact about the number the flight will be flown on**. The
aircraft measures its own gyro offset at power-up while it is disarmed and still
([10-calibration.md](10-calibration.md)); if it could not - somebody picked the
aircraft up before it was armed - the stored bias stands, and the pilot reads
which of the two it is rather than assuming.

### And whether the switch would arm it

The last line of the report is the answer to the question the next thing the
pilot does raises, and it is the same function the flight core's arming path
asks ([04-flight-core.md](04-flight-core.md)):

```text
--    link up, disarmed
--    arm:       ready
--    arm:       refused - no receiver has spoken
--    arm:       refused - the aircraft is 40 degrees from level, and arming wants 25
--    arm:       refused - the throttle is at 30 per cent, and arming wants 5
--    arm:       refused - a failsafe is latched: the arm switch off, then on
```

It is a fact rather than a failure - an aircraft on a bench with no receiver is
not a broken aircraft - but it is the fact that costs an hour when it is
missing: "it just will not arm" is the same sentence for a switch that is off, a
throttle that is up, a board whose alignment parameter is wrong and a receiver
that has never sent a frame. The same sentence is printed *once* at the moment
a pilot throws the switch and the aircraft does not respond, so the answer
arrives without anybody typing anything.

## At boot, and on demand

Every boot runs the checks quietly and prints one line - `preflight: the machine
is what the firmware thinks it is`, or a count of problems and a pointer to the
command. `preflight` prints the whole report.

## What it cannot check

- **Anything that needs a reference from outside.** A tick that advances proves
  SysTick runs, not that it runs at a kilohertz: only a scope or a stopwatch
  settles that (step 5 of the bring-up checklist).
- **Anything electrical.** Pins, solder, a dead servo, an ESC that ignores
  DShot - all invisible from inside the chip.
- **Whether the parts are configured correctly**, only that they answered. An
  IMU that probed correctly can still be misaligned or full of bias, which is
  what `calibrate` and the alignment parameters are for.
