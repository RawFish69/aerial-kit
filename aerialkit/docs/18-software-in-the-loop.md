# AerialKit - software in the loop

`tools/fw_sim.c` runs the *whole firmware* - `main()`, the console, the CLI, the
config protocol, the sensor drivers, the flight core and the navigator - against
a board contract implemented in the same file: virtual time, synthetic CRSF and
NAV-PVT frames, a register file that answers like an ICM-42688-P, and an airframe
model that turns the mixer's output back into motion.

```bash
make sil                    # builds it and prints both flights
make test                   # runs the same flights and checks what they saw
```

Every other harness in this repository tests a module. This one exists because
`main()` is not a module: the wiring between the parts is where an integration
bug lives, and nothing else here covers it.

The same binary is flown forty-two ways, and most of them are flights: the
bench calibration, the tilted-arm one, the board that lies, the output test
that meets the arm switch and the boot after a crash are sessions about an
aircraft that never leaves - or never gets to leave - somebody's hand, and one
of them never had an aircraft at all. The rest are not the same flight as
each other, because the two airframes are not the same aircraft, a mission is
not the same flight, a fence is not a mission, a quadrotor's return is not a
wing's, a return whose GPS has gone quiet is not a return at all, a return a
flat pack asked for is not any of those, a mission a pack going critical
interrupted is not the mission it was, and the *other* flight law is not the
same flight either. Five of the runs are controls rather than flights of
interest: the quadrotor's return with the rangefinder taken off, and the
gps-loss landing with and without one - the same scenario twice with one thing
changed, which is the only way to say what the part is doing - and the
quadrotor's mission flown in still air as well as in wind, which is where the
nudge that earns a heading shows its work - and the return flown with a gyro
that has warmed up since somebody calibrated it, which is where the bias the
firmware measures for itself shows its work
([09-sensors.md](09-sensors.md)).

**And the capture the checks read has a size, which is a fact about the
instrument.** It was 16 KB, which is *smaller* than one `log flash` dump of a
full flight (fifteen kilobytes) before the boot report, the tape and every
report a scenario types are counted - so the longest session's blackbox check
was reading a hundred and fifty records of a parked aircraft instead of four
hundred and eighty-five of a flight, and a perfectly good flight looked like a
fault. It is 128 KB now. That is the third time in this project that a check
was wrong rather than the aircraft.
The mission is flown twice - by the wing in the table
below, and by the quadrotor in the same session with `quad` after it, which is
the same waypoint list, the same switch and a different profile (see
[14-navigation.md](14-navigation.md)):

**Every flying session has wind in it** - five metres a second out of the west -
because the difference between where an aircraft points and where it goes is
what a crosswind *is*, and that difference is the assumption the estimator's
ground-track alignment rests on. It is not a detail: the wing's transect
carries a crab angle of up to 86 degrees, its steering is done on the track the
GPS reports, and it still comes home inside the arrival radius. The quadrotor
returns and lands in it too, which it took three fixes to get: the *plant* was
missing its drag, the firmware's climb rate - the difference between two
passes of a 1 kHz loop on a 32 Hz barometer - read as zero while the aircraft
descended, so the vertical loop commanded *more* descent, and the position loop
needed to learn the wind before holding a station stopped costing a standing
error. See
[14-navigation.md](14-navigation.md) for both, and for the tape: the return now
starts its descent 7.7 m from home, holds a station there worth an average of 1.0 m,
and stops its motors 0.5 m from home and 0.6 m up. The same session with `calm`
as a second word flies it in still air - 46 m out at 4 m per second, motors
stopped 0.8 m from home - which is how the wind term is shown to do nothing when
there is no wind.

| | quadrotor | fixed wing | quadrotor return | gps goes quiet | mission | fence | bench |
| --- | --- | --- | --- | --- | --- | --- | --- |
| airframe | `airframe 0`, the quad-X mixer | `airframe 1`, the twin-motor elevon wing, set by typing at the console | the quad-X, pointing east while the estimator starts at zero | the same quad-X, on its way home | the same wing | the same wing | quad, disarmed, held in the hand |
| what happens | `set rc_protocol 1` at 200 ms, then arm on **SBUS** frames, throttle up, the stick forward, roll right, yaw right, the pack sags at 10.5 s, then the receiver is pulled | arm, throttle up, climb out and fly 118 m away, lose the receiver, come home, and get the receiver back | arm, climb, fly out east on the stick, lose the link at 136 m and 22 m up, and let the navigator bring it home, down and disarmed | arm, climb, fly out east, lose the link, and then the module stops answering 12 s into the return - the console is asked before and after | both waypoints typed at the console, started from a transmitter switch, and the link stays up throughout | fly out past a 250 m fence with the link up, and a 30 m floor underneath | six orientations, a `calibrate accel` each, then `imu` and `params` |
| what it proves | the angle loop reaches the commanded 17.5 deg **on a receiver whose protocol was changed mid-session from the console**, the nose goes down when the stick goes forward and right when the yaw stick does, the pack it is holding is reported as three cells and `ok` before the sag and `critical` after it, losing the link stops the motors, and `output test` refuses to drive an armed aircraft | the navigator takes over, turns round, comes back inside the 60 m arrival radius holding the altitude it captured, circles, and hands control back to the sticks - **in five metres a second of wind**, with its nose up to 86 degrees off the track it was making | the quadrotor's own return, flown **in the same wind**: the yaw estimate finds the ground track it is making, the return engages, it comes home at the altitude it held, **starts down 7.7 m from home** instead of at a wing's arrival radius, holds its station there, and **lands and stops its motors on the ground** 0.5 m from home and 0.6 m up - staying disarmed when the receiver comes back - and sends RTH and then DISARM to the handset over telemetry, and answers the handset's device ping once per ping with the name and the addresses a transmitter lists ([08-receiver.md](08-receiver.md)) | **the return does not stop when its sensor does**: with the fix gone the motors keep turning (lowest collective 0.53 against a hover of 0.55), the altitude is held at 20.2 m for the rest of the flight, the mode stays RTH on the handset and the console reports no valid fix and the steps it has been holding for | a navigator flying an aircraft whose pilot is still there: both waypoints reached and counted, the flight core reporting `on autopilot`, and the captured altitude held to the end | the fence takes the aircraft off a pilot whose link is up, climbs it to the floor on the way home, and gives it back inside | the six-position calibration (its arithmetic, its face mapping, the parameters it writes, the corrected sample the flight loop flies on), and `output test` moving every motor and every servo and leaving them at zero |
| length | 20 s | 40 s | 100 s | 70 s | 75 s | 60 s | 16 s |
| transcript | [evidence/sil-flight.txt](evidence/sil-flight.txt) | [evidence/sil-wing-flight.txt](evidence/sil-wing-flight.txt) | [evidence/sil-quad-return.txt](evidence/sil-quad-return.txt) | [evidence/sil-gps-lost.txt](evidence/sil-gps-lost.txt) | [evidence/sil-mission.txt](evidence/sil-mission.txt) | [evidence/sil-fence.txt](evidence/sil-fence.txt) | [evidence/sil-bench-calibration.txt](evidence/sil-bench-calibration.txt) |

The eighth flying scenario is not in that table because it is the only one whose
trigger is the *energy*: `aerialkit-fw-sim 110 battery` flies the quadrotor out
with the link up and a pilot on the sticks, takes its cells below the critical
threshold, and watches the aircraft take itself home and land - including a
recovery to 12.00 V partway, which must not cancel the return. Its transcript is
[evidence/sil-battery.txt](evidence/sil-battery.txt) and the behaviour is in
[19-battery.md](19-battery.md).

The ninth is not there because it flies the *other flight law*:
`aerialkit-fw-sim 18 rate` throws the mode channel low and puts a stick on each
axis, which is the shape of flying a quadrotor by hand. It flies the same
airframe through the same loop, and what it measures is what a pilot feels:
a full roll stick asks for 400 deg/s and the airframe reaches 389, the three
axes turn the way their sticks are pushed, centring the stick stops the
rotation inside a second, and the aircraft keeps the attitude it was left at
rather than levelling - which is the whole difference from angle mode. Its
transcript is [evidence/sil-rate-mode.txt](evidence/sil-rate-mode.txt) and the
control-law side is in [04-flight-core.md](04-flight-core.md).

And the tenth run is the quadrotor's return with a *sensor* failure:
`aerialkit-fw-sim 100 quadrth nobaro` stops the barometer answering as the
descent begins - the moment the aircraft is leaning on the height for the only
thing this firmware stops its own motors for. Measured before it was fixed: the
navigator's height froze at twenty metres, the landing rule never fired, and the
aircraft ended the session on the ground with its motors still running. After:
the estimate falls back to the GPS, the return lands at the same second as the
healthy flight, and the console has said why. Transcript:
[evidence/sil-quad-return-nobaro.txt](evidence/sil-quad-return-nobaro.txt), and
the sensor side is in [09-sensors.md](09-sensors.md).

The eleventh run is the same quadrotor session with a *receiver* that is still
alive: `aerialkit-fw-sim 20 quad rxfailsafe` switches the transmitter off but
leaves the receiver sending, with its own failsafe flag up and the channels
holding the last stick positions - a hovering throttle and centred controls,
which is what a good command looks like. The aircraft stops its motors 236 ms
after the transmitter goes off, 450 of those frames notwithstanding; the host
test calls the parser, and this is the wiring between the parser and the
motors. Transcript: [evidence/sil-rx-failsafe.txt](evidence/sil-rx-failsafe.txt),
and the flag-by-flag account is in [08-receiver.md](08-receiver.md).

The twelfth is the one sensor the flight core will not fly without:
`aerialkit-fw-sim 20 noimu` stops the gyro answering at 24 m, in flight, with
the sticks where the pilot left them. The motors stop in the *same
millisecond* - the loop reads the IMU every pass, so the only delay is the loop
itself - and when the part comes back four seconds later the state is still
`failsafe`: the latch is the half of a failsafe that matters, because one that
clears itself when the fault does is one that starts flying again on its own.
Transcript: [evidence/sil-no-imu.txt](evidence/sil-no-imu.txt), and the rule is
in [04-flight-core.md](04-flight-core.md).

And the thirteenth is a *bad* fix rather than a missing one:
`aerialkit-fw-sim 170 quadrth badfix` has the module keep sending - five frames
a second, a 3D fix it calls ok - while its satellite count drops to four, twenty
seconds into a return. The navigator will not navigate on that, and the console
says so (`usable:    no - 4 satellites, and the navigator wants 6`); the
aircraft holds 23.0 m with its motors never below 0.53 of a hover, and drifts
**from 96 m to 176 m from home** while it does, because a hold with no position
worth using is a hold in the wind. The count comes back twenty seconds later,
the return picks up where it left off, and it lands 2 m from home. Transcript:
[evidence/sil-quad-return-badfix.txt](evidence/sil-quad-return-badfix.txt).

The fourteenth is the fix that *lies*: `aerialkit-fw-sim 100 quadrth gpsjump`
jumps the position 50 m north for two seconds while the aircraft is 20 m from
home, with the fix type, the satellite count and the accuracy figure all
unchanged - the one bad fix the navigator has no way to refuse. Measured: the
return stops closing for those two seconds (the distance sits at 17-18 m
instead of falling), then picks up and lands 2 m from home. That is the cost of
a glitch, and [13-gps.md](13-gps.md) has how the two reference implementations
answer the same case (neither has a per-fix rule either).

And the fifteenth is a mission with a pack that fails under it:
`aerialkit-fw-sim 130 mission quad flatpack` flies the quadrotor's waypoint list
with `battery_rth` on and sags the pack to 3.25 V a cell 46 s in, 219 m from
home. It is a *precedence* run, and it found a real one: the mission block ran
first and returned, so the pack's return never engaged, and the aircraft went on
visiting waypoints with a critical pack until the pilot's switch came off at
118 s - measured at 275 m from home and still armed at the end of the session.
Now the fence and the pack are judged before the mission and stop it, the
aircraft has stopped its motors 1 m from home by 103 s, and it stays there: the
switch is *still on* until 114 s - sixty-eight seconds after the pack took the
mission away - which is the half of the fix that is a mode rather than a
trigger. A mission something else stopped must not come back because the switch
was never moved. The report the simulator types mid-flight counts
`1 started, 1 taken back`, and the last `status` says `disarmed`. Transcript:
[evidence/sil-quad-mission-flatpack.txt](evidence/sil-quad-mission-flatpack.txt).

And the sixteenth is an aircraft that is not the right way up:
`aerialkit-fw-sim 20 tilt` holds a quadrotor at 60 degrees of roll from before
its first sample, throws the arm switch, and then puts it down level and throws
it again. It is the session for a gate that was *missing* rather than broken -
AerialKit checked the switch, the throttle and the estimate and never checked
whether the aircraft was upright, which both references do
([04-flight-core.md](04-flight-core.md) has the citations) - and what it
measures is three things at once: the console names the tilt and agrees with the
plant about how far out it is (60 degrees, against a limit of 25), **no output
ever moved** while the aircraft was held over (`motors: 0 0 0 0` and the first
frame the mixer produced came after it was put level), and the same switch arms
it and runs the motors once it is. The refusal is printed once per attempt,
which is why the transcript reads like a conversation rather than a log:

```
sim:    812 ms  the aircraft is held at 60 degrees of roll
sim:   4001 ms  arm switch on, throttle down
arm:       refused - the aircraft is 60 degrees from level, and arming wants 25
state:     disarmed
motors:    0 0 0 0 per-mille
sim:  10001 ms  put down level
sim:  12001 ms  arm switch on again, throttle down
state:     armed
motors:    552 552 546 546 per-mille
```

Transcript: [evidence/sil-tilt.txt](evidence/sil-tilt.txt).

And the seventeenth is the fixed wing's first two seconds: `aerialkit-fw-sim 14
launch` arms a wing with the launch switch still off, throws it, and watches the
climb the launch holds while the pilot's hands are busy.

```
sim:   3001 ms  launch switch on - the aircraft is in a hand
launch: 18 degrees of climb at 70 per cent - a stick gives it back
state:     on autopilot
motors:    699 700 0 0 per-mille
sim:   7001 ms  the pilot takes it: 20 m up, nose 18 deg
launch: over - a stick
state:     armed
motors:    599 599 0 0 per-mille
```

What it measures is what a launch promises and the one thing it must not do: the
motors went to the launch throttle (0.71 of full, against 0.70 asked for), the
aircraft flew the attitude it was given (18 degrees of nose-up when the pilot
took it, against the 18 it was asked for), it left the ground while the launch
was flying it (20 m), the mid-launch `status` says `on autopilot`, and the stick
gave it back with the later `status` saying `armed`. What it caught on its first
run was a real bug rather than a wrong number: the switch was read as a *level*,
so a launch that ended on its timeout was followed by another one, and another,
for as long as the switch was held - the same class of mistake as the mission's
switch, in the mode with the least time to notice it. Transcript:
[evidence/sil-launch.txt](evidence/sil-launch.txt), and the manoeuvre itself is
[24-launch.md](24-launch.md).

And the eighteenth and nineteenth are the two ways a *bench* interacts with
arming - both of them safety rules that had never been executed.

`aerialkit-fw-sim 6 testarm` starts an `output test` sweep on a held aircraft
(props off, which is what the command is for) and throws the arm switch under
it. The test stops, with the firmware's own sentence, and the check is
two-sided: the sweep has to have moved a motor before it stopped - "it stopped"
is also true of a test that never started - and the four motors are then left
at the aircraft's armed idle (0.072 each in the simulation) rather than one of
them still at the sweep's 0.150. That is the interlock giving the outputs to
the pilot who just took them: [evidence/sil-output-test-arm.txt](evidence/sil-output-test-arm.txt),
and [07-outputs.md](07-outputs.md) has the other two rules of that command.

`aerialkit-fw-sim 10 armnow` is an aircraft somebody is holding: the simulator
wobbles it with the small rotation a hand cannot help, and the arm switch goes
up while it is still being held. The gyro bias the firmware measures for itself
needs five hundred samples that agree with each other, and a hand never gives
it five hundred in a row, so the aircraft arms without one - and says so, once:

```
gyro: no bias measured before arming (0 samples, 2198 rejected as moving); the stored bias stands
```

What the checks read back is the firmware's own numbers (zero samples, two
thousand two hundred rejected as moving), that the *normal* line ("bias
measured while disarmed") is absent - or the session would have proved nothing
about which path it took - and then that the aircraft is put down and flies,
because an aircraft whose stored bias stands is an aircraft expected to fly.
Transcript: [evidence/sil-armed-while-held.txt](evidence/sil-armed-while-held.txt);
[10-calibration.md](10-calibration.md) has the numbers that make it a race, and
why the *other* way in - arming faster than the measurement - cannot happen.

And the twentieth is the mission the *pilot* ends: `aerialkit-fw-sim 130
mission quad takeback` flies the same waypoint list and, after the second
arrival, moves a stick while the switch stays **on**. That is the state
`main.c` has a branch of its own for - the switch is still asking for a mission
and the mission has already been cancelled - and the rule is that it must
*stay* cancelled until the switch is cycled, because a mission that restarted
itself under a switch the pilot never touched is a mode nobody is flying. The
two checks read the navigator's own report twice: `1 started, 1 taken back`
both times, with the status between them saying the pilot has the aircraft
([evidence/sil-mission-takeback.txt](evidence/sil-mission-takeback.txt)).

Two more endings that had no session were closed in the same piece, in the
sessions that already flew them: the **launch's third way out** - the switch
going down under it, which `24-launch.md` describes and no script had thrown -
and the **fence report**, which is the sentence a pilot reads in the air and
which nothing had ever asked this firmware to print. The fence session now
types `gps` twice: once above the lid and once back under it, so both sides of
`ceiling: N m above home, over it` and the count of how many times the fence
has fired are on the tape.

And the twenty-first is the last sensor failure with no session: `aerialkit-
fw-sim 100 quadrth rangefail` stops the **rangefinder** answering as the
return's descent begins and brings it back four seconds later. The scenario
word had been in the simulator since the part was written and nothing had ever
run it, let alone checked it - and its first run printed the firmware's two
lines *alternating once per poll*, with the count of failed reads stuck at one.
`ak_rangefinder_read()` returned 0 for both "not asked this pass" and "answered,
and the reading was not usable", so a dead part looked like one answering every
other pass. The service returns four values now, the session passes, and
[09-sensors.md](09-sensors.md) has the story and the transcript:
[evidence/sil-quad-return-rangefail.txt](evidence/sil-quad-return-rangefail.txt).

And the twenty-second is the launch that meets a **failsafe**: `aerialkit-fw-sim
45 launch pack` throws a wing with `battery_rth` on and sags the pack four and a
half seconds later, seven metres up and mid-launch. A launch is a *mode*
somebody asked for and a critical pack is a *reason* to stop flying, and the
rule is that the reason wins - the code comment above that block says why ("a
launch that outlived a failsafe would be a mode that switched one off", which
is the bug it exists because of), and the two checks are that the console says
both sentences (`battery: ... bringing it home`, `launch: over - the pack`) and
that the return - not the launch - is the thing flying the aircraft afterwards.
[evidence/sil-launch-pack.txt](evidence/sil-launch-pack.txt).

And the twenty-fourth is the **bench's mistakes**: `aerialkit-fw-sim 14 refuse`
and `refuse rcquiet` type the calibrations in the states that refuse them -
four of them on an *armed* aircraft, a pack that is not a pack, a divider with
nothing on it, a face that is not a face, a receiver that has never spoken, and
a gyro calibration somebody interrupts by picking the aircraft up. Every line
it produces is a diagnosis a person reads at a bench, and the first check found
a defect rather than a missing sentence: **`calibrate accel` had no "not
disarmed" guard where the gyro, receiver and pack calibrations all do**, so an
armed aircraft could be held in six attitudes and have `accel_bias_*` written
under it. Removing the guard again fails the check with "3 of the four"
([10-calibration.md](10-calibration.md), and the two transcripts
[evidence/sil-calibrations-refused.txt](evidence/sil-calibrations-refused.txt),
[evidence/sil-calibrations-no-receiver.txt](evidence/sil-calibrations-no-receiver.txt)).

And the twenty-fifth is the *board* a devkit out of the bag is: `aerialkit-
fw-sim 100 quadrth bare` flies the return on a machine with **no barometer, no
rangefinder and no pack divider** - an inertial sensor, a receiver and a GPS,
which is every ESP32 this project has and the bench F405 before the parts were
soldered on. Two things are asked. The first is that the machine says what it
is not, on the lines a person reads first:

```
--    imu: icm42688p
--    baro: none fitted
--    rangefinder: none fitted
--    altitude: gps only
--    battery: none fitted on this board
baro:      none fitted - altitude comes from the gps alone
battery:   none fitted on this board
```

and the second is that it **flies anyway**: the return engages, comes home and
lands on the gps's altitude where the barometer would have been and on the
barometer where the rangefinder would have been - on the ground at 0.0 m,
motors stopped 0.1 m up, 0 m from home, with ten checks of the same return the
fitted board flies ([evidence/sil-bare-board.txt](evidence/sil-bare-board.txt)).
That is the firmware's own claim ("a board without one still flies"), and it is
now a flight rather than a comment.

### And the last one is not a flight at all: the boot after a crash

Everything above judges a flight. `aerialkit-fw-sim 3 fault` plants the record a
hard fault would have left in the RAM startup does not clear, boots, and reads
what the firmware makes of it - because the boot that follows a crash is the
only place that record is ever read, and it is what somebody at a bench copies
into `arm-none-eabi-addr2line`:

```
fault: 2 recorded, last pc 0x08001b42 lr 0x08000a55 psr 0x61000000
       cfsr 0x00000082 hfsr 0x40000000 mmfar 0x00000000 bfar 0x00000000
       r0 0x11111111 r1 0x22222222 r2 0x33333333 r3 0x44444444 r12 0x55555555
       frame 0x2001ffc0 (the stacked registers are here)
```

Six checks, and each is a whole line rather than a substring: the count and the
pc, the fault status registers, the registers the faulting code was using, that
the boot *consumes* the record afterwards (so the next thing that asks is not
told about the same crash twice), and that the preflight - which runs after the
report - agrees, and where the frame was. That last one is the field a reader
cannot rebuild from the others: the record holds *copies* of the stacked words,
and the originals are still at that address. A line that prints the wrong field
sends the reader to a line of code that was fine, which is worse than printing
nothing at all.

What it found on the way in: the simulator's own stand-in for the fault record
answered "no fault" to every question, whatever was in it, so no session could
ever have exercised the boot's fault lines. It reads the magic now, the way both
ports do. Transcript: [evidence/sil-fault.txt](evidence/sil-fault.txt); the
record's own host test is `tests/test_fault.c`, and the page a person reads at a
bench is [05-bringup.md](05-bringup.md) §6.

### And the board is the one that is wrong: the preflight's failures

The fault session above plants a *crash*. Three more plant a **board that is
wrong**: `aerialkit-fw-sim 6 badboard` arms one lie at a time - a console on a
port this board does not have, a crystal that fell back to the internal clock, a
saved configuration whose record will not read, a fault from the run before,
timers two kilohertz off what was asked, a millisecond tick that stopped - and
types `preflight` after each. Eight checks, and the first and last stages arm
nothing: the same command has to come back clean before the first lie and after
the last, because a check that cannot pass is not a check either.

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

Until this existed every one of those branches had been compiled and never run:
the simulator's board was always the well-behaved one, so the lines existed and
no test had watched them come out. A check nobody can trip is a check nobody can
trust - and this is the page of diagnosis a person reads first, on the bench,
when a fresh board does not behave.

Two of the lies are facts about boot rather than switches a running command can
read, so they are words of their own. `badboard noout` is a port whose timers
never came up - and it lies about the DShot rate as well, so what the checks
pin is the line that is *absent*: a timer that is not running is not asked what
rate it is running at. `badboard twomotor` is the ESP32-C3's shape, two motors
against a quadrotor's mix, and the check is the sentence a pilot reads when the
switch does nothing - `refused - this airframe's mix needs 4 motors and 0 servos,
and the board drives 2 and 2` - as much as the preflight's own line.

What the session could not do is worth as much as what it did: the other outputs
branch, `FAIL  the board has not said what its outputs are`, cannot be produced
at all while `main.c` states the board's shape at boot, so it is a guard for a
port that never calls `ak_flight_set_board_outputs()` and the session says so
rather than faking it. Transcripts:
[evidence/sil-preflight-lies.txt](evidence/sil-preflight-lies.txt),
[no outputs](evidence/sil-preflight-no-outputs.txt),
[two motors](evidence/sil-preflight-two-motors.txt).

### And the module that is there and not talking

A u-blox out of the box speaks NMEA at 9600, which this parser cannot read, so
the firmware sends it a UBX configuration frame at boot - up to four times, two
seconds apart, and then gives up and flies without a fix. Nothing in this file
reached that path before, because every other session's module is talking from
the first second and the firmware's first question is "has it said anything
yet?". `aerialkit-fw-sim 10 gpsquiet` is the case a person actually meets on a
bench: the module is wired, powered, and silent.

```text
sim: ok   the module is asked to configure itself four times, and no more
sim: ok   and the console's own report says so
sim: ok   and what it sends is a UBX configuration frame
sim: ok   asking for a measurement every 200 ms, which is 5 Hz
sim: ok   and for NAV-PVT once per solution on the port
sim: ok   and the aircraft knows it has no fix
sim: ok   with the parser having read nothing off the wire
```

What is checked is the *content* of the frame, not just that one went out: a
configuration that asks a module for the wrong rate is a module that answers at
the wrong rate, which reads as a fix that lags the aircraft. The walk through the
key/value pairs is also where the check earns its keep - the two rate keys carry
sixteen-bit values and the message keys one byte each, so a walk that assumed one
size read every later key off by one byte. The first draft of this check did
exactly that, and what it reported was "the NAV-PVT key is missing" rather than a
wrong number. Transcript: [evidence/sil-gps-quiet.txt](evidence/sil-gps-quiet.txt).

### And the two long ones, where the drift would show

Every session above is under three minutes, and the failures that take minutes
to appear - a position loop with a slow walk in it, an altitude that loses a
metre a minute, an integrator winding up - have nowhere to show in three
minutes. Two sessions close that: the same mission as above, with the switch
left **on** for five minutes, so the aircraft is still holding the station (the
quadrotor) or still circling it (the wing) at the end of the run.

```text
sim: ok   it is still at the last waypoint minutes later   1 m from the last waypoint after 300 s, still holding station there
sim: ok   and at the altitude it has held all along        started at 3 m, still at 3 m
sim: ok   with the mission still flying it                 the status says on autopilot
```

The wing's numbers are the same shape - 50 m from the waypoint it has been
circling for five minutes, out of the sixty metres the navigator calls arrived,
and the altitude it started with.

**Getting this right took one attempt and one throw-away.** Running the ordinary
mission for six minutes is *not* the same test, which is what trying it showed:
the switch goes off on a scripted timeline, so the last four minutes are flown
on whatever the script's final stick positions were - and the aircraft climbs
away, ending 2.2 km from its first waypoint, while the run still reports PASS,
because every check it has was frozen when the mission ended. The long sessions
hold the mission on and read the *live* distance and altitude instead, which is
why their bounds are the arrival radius and the mission's own altitude
tolerance: those are what "holding" already means here.

### And the sensors as a real part leaves them

Everything above runs on ideal sensors: the gyro reports the plant's exact rate,
the accelerometer the exact gravity vector, the barometer the exact pressure
(quantised, which is the one real error the tape ever had) and the GPS the exact
position. A real part is not that good, and the loops that fly on those numbers
had never been asked whether they tolerate one.

`noisy` is a word any flying session takes, and it puts a datasheet's worth of
noise on all four:

| Sensor | One sigma | Where the number comes from |
| --- | --- | --- |
| gyro | 0.06 deg/s at 1 kHz | ICM-42688P, 2.8 mdps/√Hz |
| accelerometer | 0.0016 g at 1 kHz | ICM-42688P, 70 µg/√Hz |
| barometer | 0.5 Pa, about 4 cm of height | DPS310 noise, *underneath* the 0.39 Pa quantum the firmware already has |
| GPS fix | 1.5 m horizontal, 1° of course | the accuracy a good fix reports |

The generator is a seeded LCG with no libm in it, so a noisy session produces
the same tape on every host - a noise run nobody can re-run is a noise run
nobody can compare against. Three sessions are re-flown with it:
`quadrth noisy` (the quadrotor's return), `wing noisy` (the wing's, which needs
55 seconds rather than 40 because a noisy fix makes it take longer to get home -
33 s against 22) and `quad noisy` (the sticks).

What it found is worth saying plainly: **the flight is unaffected**. The
quadrotor's return lands 1 m from home with the same station-keeping as the
clean run, the wing's arrives and hands back, and the stick session reaches the
same 17.5° and turns its 93.5°. What it did *find* were two faults in the
session's own instruments, which is what a new stimulus is for:

- the wing's pilot check read the aircraft's roll at the **end of the run**
  rather than a fixed time after the pilot asked for the left roll, so a return
  that took 11 s longer put the pilot's phase outside the session - it measures
  three seconds after the command now;
- the quadrotor's yaw check subtracted the plant's headings **without wrapping
  them**, and a noisy run happened to cross north: a right turn of 93° came out
  as a left turn of 266°. The aircraft was right and the instrument was wrong,
  which is the second time in this project that has been the case.

In both, somebody types `status`, `rc`, `gps` and `preflight` at the console
while the aircraft flies. The transcript therefore carries two views of the same
aircraft at the same timestamps - the simulator's (attitude, position, and the
numbers the mixer handed to the timers) and the firmware's own (what `status`
believes it is doing) - and the two disagree exactly where a bug is.

### And the ground is measured rather than inferred

Every session above, including the noisy ones, lands on an *inference*: a
descent that has stopped while the navigator is still asking for it was stopped
by something, and at the end of a descent that something is the ground. A
TOF10120 on the I2C bus changes that to a measurement, and the sessions that
prove it come in pairs - the same flight twice with the part taken off, because
one run cannot say what a sensor is doing:

| Run | What it shows |
| --- | --- |
| `aerialkit-fw-sim 100 quadrth` | the return lands and stops its motors on the *part's* verdict: the reading reaches the ground and stops falling, with the barometric rule underneath it as a fallback rather than as the reason |
| `aerialkit-fw-sim 100 quadrth norange` | the same return with no part fitted: the barometric rule lands it, motors stopped at 0.6 m, which is the path every earlier flight took |
| `aerialkit-fw-sim 130 gpslostland` | the fix gone, the link down, `quad_hold_land_s 8`: the aircraft comes down where it is - 20.2 m to the ground, lowest collective 0.51 against a 0.55 hover - and stops, 438 m downwind because with no position there is nothing to hold it anywhere |
| `aerialkit-fw-sim 130 gpslostland norange` | the same pilot setting with no part: it holds at 20.2 m and the motors keep turning, which is the negative control for the whole feature |

And one more session exists because a quadrotor could not start a mission in
still air at all: `aerialkit-fw-sim 150 mission quad calm` flies the same
waypoint list in no wind, where the navigator has to earn a heading before it
can fly anywhere. It reaches both waypoints (4 m and 0 m), counts both and
holds station over the last one - [14-navigation.md](14-navigation.md) has the
two-and-a-half fixes it needed.

And one because a gyro's offset moves with temperature: `aerialkit-fw-sim 100
quadrth gyrobias` gives the part five degrees a second that nobody has
calibrated, and the return flies on the bias the firmware measured for itself
at power-up. The check is the heading the aircraft reports while it is parked
at the end of the flight, where nothing else can move it: **zero degrees** with
the measurement, 109 without. [10-calibration.md](10-calibration.md) has the
three wrong turns that took.

The parts are in [09-sensors.md](09-sensors.md) and [14-navigation.md](
14-navigation.md), and the noise word applies here too: `range_refresh()` in the
simulator adds 40 mm of a real part's noise when a session asks for it, and the
filter, the period and the decision are all in the firmware.

The recordings are captured from the commit before the commit that adds them -
they are recordings, like the M1 trace, not build outputs, and the `rev: ...-dirty`
line at the top of each is the honest part of that: it is the revision the tree
was based on, with the recording's own edit in it. `make sil-trace` re-records
all six from whatever is built now, which is what was run to refresh them when
the console gained its `ak> ` prompt; the revision and build stamp inside each
file say which build it came from, and a recording that does not match the
current console is stale rather than wrong.

### And the simulator can be typed at

```bash
build-host/aerialkit-fw-sim 0 console
```

runs the same firmware with no scenario: the flight loop flies, the sensors and
the radio and the GPS carry on being modelled, and the console's bytes come from
stdin instead of from a script. It is how [tools/bench_check.py](../tools/
bench_check.py) is tested without a board - the same checklist that
[05-bringup.md](05-bringup.md) types at a real one - and it is the only way to
exercise the console *as* a conversation, which is what found two things: the
firmware printed no prompt at all (so a board that was listening and a board
that had stopped looked alike), and this console treats CR and LF as separate
end-of-line, so a script that sends `command\r\n` runs the command and then an
empty line.

### Why a re-recording moves numbers nobody changed

The simulated clock charges 1 microsecond for a clock read that comes with no
board call behind it - it is modelling a spin-wait, which is what waiting is in
this firmware - and it charges the loop's millisecond once per pass in
`sim_step()`. The consequence is that **adding a clock read to the flight loop
shifts the second hand**: the battery sampling below cost exactly one pass out
of 59940, which moved `status` by one iteration and changed the last printed
digit of two or three lines in the fence and wing transcripts.

It is a real property of the harness rather than a flight that changed, and it
is worth knowing before spending an afternoon on a difference that is not one.
The way to tell the two apart is the check at the top of every run - "one pass
of the loop is one millisecond" prints the totals - and the flight checks
themselves, which are what the recording is for. The mechanism is load-bearing:
an earlier version charged a millisecond for every clock read, the firmware's
own sense of time then ran at twice real speed, and the radio frames were
scheduled onto multiples of twenty that almost never came round.

The same accounting decides how fast the board *sounds*. A pass is a millisecond
of its time and a few microseconds of ours, so the simulated clock runs at
roughly **600x wall time**: in the 3 s a `timeout 3` allows it reaches
`alive: 1810811 ms` and prints **181 heartbeat lines**, one every ~17 ms. Anything
on the host that treats a quiet wire as a signal is therefore wrong about this
board - the 10 s heartbeat is a 17 ms event here. A reader that waits for
silence waits for ever, which is what stalled `tools/bench_check.py` at its first
command until its drain grew a second, absolute bound (traps 200). The board on
the bench is the opposite: over USB it says nothing at all unless it is asked.

## The quadrotor

The recorded run is [evidence/sil-flight.txt](evidence/sil-flight.txt), captured
from the commit before this file's own - it is a recording, like the M1 trace,
not a build output. The short version, over 20 s of simulated time:

```
sim:   1501 ms  arm switch on, throttle down
sim:   2500 ms  status: state armed, motors 50 50 50 50 per-mille
sim:   3002 ms  throttle up to hover
sim:   5003 ms  roll right, half stick
sim:   6631 ms  roll   17.6 deg    motors 0.56 0.56 0.56 0.56
sim:   7004 ms  receiver unplugged
sim:   7631 ms  roll   17.5 deg    motors 0.00 0.00 0.00 0.00
sim:  10500 ms  status: state failsafe, motors 0 0 0 0 per-mille
```

Half stick in angle mode is 0.5 x 35 deg of tilt, and the airframe settled at
17.5 deg: the stick, the CRSF parser, the estimator, the angle loop, the mixer
and the plant are one closed loop, not a pile of parts that each pass their own
tests. Pulling the receiver out stopped the motors 238 ms later.

The same session flies with a flight pack, because the simulator's board has an
ADC and the firmware does not know the difference. It holds a 3S at 12.0 volts,
`battery` is typed at 8.5 s, and at 10.5 s the pack sags to 9.75 volts - a 3S at
3.25 volts a cell - and `battery` is typed again:

```
> battery                                     # at 8.5 s
battery:   3S, 12.00 V pack, 4.00 V a cell - ok
> battery                                     # at 13.5 s, after the sag
battery:   3S, 9.75 V pack, 3.25 V a cell - critical
```

The sag is a step rather than a discharge curve on purpose: what runs here is
the arithmetic that turns a pin voltage into a cell count and a level, through
the real loop, and a battery model would be a second thing that could be wrong.
The three cells are the interesting half - 9.75 volts divided by 4.30 is 2.2,
and a count that rounded down would call this a healthy 2S at the end of a
flight. [19-battery.md](19-battery.md) has the rest.

This is also the session that flies on **SBUS**. It is the only one that does;
the wing, the mission and the fence all fly CRSF, which is what the wing
actually carries. The console is told `set rc_protocol 1` at 200 ms - before
anything is armed - and from that moment the board's line settings change, the
parser that sees the bytes changes with them, and the simulated receiver starts
sending the other protocol. The aircraft then arms and flies, which is the
proof that matters: a firmware reading the wrong protocol would have counted a
stream of rejected bytes and never armed. The simulator builds its SBUS frames
itself rather than calling the firmware's packer, for the same reason it builds
its CRSF frames itself - two implementations that agree are evidence, and one
that calls the other can only prove it agrees with itself.

```text
> rc                                        # in the quadrotor's session
receiver:  sbus, 7875 bytes, 315 frames
sbus:      315 good, 0 failsafe, 0 with lost frames, 0 rejected
link:      framing
```

## The fixed wing

The same firmware, told by the console that the airframe is `1`, flies a wing:
out to 118 m on a cruise throttle, up to 13 m, and then the receiver comes out.

```
sim:  11003 ms  receiver unplugged, 118 m out
sim:  12631 ms  roll   36.0  pitch  -0.5 deg  alt  15 m   142 m out  course  30.6
sim:  15631 ms  roll   35.1  pitch  -0.1 deg  alt  15 m   161 m out  course 110.8
sim:  21631 ms  roll    4.1  pitch  -0.1 deg  alt  15 m    91 m out  course 207.9
sim:  23739 ms  home again, 60 m out - loitering
sim:  27631 ms  roll   12.3  pitch  -0.0 deg  alt  15 m    12 m out  course 241.3
sim:  31740 ms  receiver back, left roll - has the pilot got it?
sim:  33631 ms  roll  -18.4  pitch  -0.0 deg  alt  15 m    95 m out  course 262.0
```

It banks to the angle that steers it home, turns through south, arrives inside
the 60 m arrival radius at 12 m, and circles there - 12 m to 67 m, the far side
of a loiter the navigator holds when it has nowhere else to be. The altitude it
captured is the altitude it kept, to a metre, for the whole forty seconds, which
is the whole of the altitude-hold behaviour and is not something the unit tests
were checking.

Then the receiver comes back and the pilot asks for something no part of the
return would ever do - a roll to the left, away from the circle - and the
aircraft banks to exactly the -17.5 deg that half stick in angle mode means and
flies off. That last line is the check that matters most: an aircraft that
returns but will not give the sticks back is more dangerous than one that never
left.

### And the same wing with nothing to bring it home

RTH off is what a board ships with, so the wing above is the *equipped* case.
The thirtieth session is the other one: a wing, no navigator, and the receiver
comes out. What it does then is the owner's rule - land as it circles down -
and the session is the evidence that the rule is a landing rather than a fall:

```
sim:  11003 ms  receiver unplugged, 125 m out
sim:  12811 ms  roll   30.5  pitch   -8.7 deg  alt   13 m   145 m out  course  45.3  motors 0.11 0.23
sim:  17811 ms  roll   29.8  pitch   -8.1 deg  alt    7 m   159 m out  course 195.4  motors 0.10 0.24
sim:  21827 ms  12 m lower at 2 m - the circle came down
```

The console's own view of it, from the `status` the script types after the link
went:

```text
state:     circling down
motors:    60 239 0 0 per-mille
servos:    1 2 per-mille
```

Four things are asserted, and they are what separates this from the quadrotor's
answer (the same event with a different airframe: motors to zero): the firmware
*says* it is circling down; it came down - twelve metres of it, from the
fourteen it was at when the receiver came out; it was still being flown on the
way, holding the bank and the nose-down it was told to hold (30 and 8 degrees,
to a tenth); and the motors were still turning, which is what keeps air over the
elevons. A wing that arrived with centred servos and stopped motors would be
somewhere in the field with nothing having flown it.

One limitation, written down because the numbers depend on it: the simulator's
plant has no ground under a *wing* (the quadrotor's vertical model is the one
with a floor), so the session ends at the point the descent has happened rather
than waiting for an altimeter to read zero.

## What it asserts

`make test` runs both flights and reads their exit codes. Each check is something
the board contract can see for itself, so a regression in the flight core, the
parser or the wiring between them fails the build instead of printing a
transcript nobody reads:

| Check | Quadrotor | Fixed wing |
| --- | --- | --- |
| One pass of the main loop is one millisecond | 1.0010 ms over 19980 passes | 1.0010 ms over 39960 passes |
| Every radio byte reached the parser | 8294 of 8294, ring dropped 0 | 24232 of 24232, ring dropped 0 |
| The radio sent frames throughout | 319 frames | 932 frames - the receiver goes away and comes back |
| The GPS stream reached the parser | 97 frames, 0 dropped | 197 frames, 0 dropped |
| The position the GPS sent is the position the firmware has | `52.1234567, 4.9876543` | the same, and the same check |
| The aircraft armed and the mixer drove the motors | motors turning 2013..7242 ms | motors turning 3012..40000 ms |
| The elevons moved | - | to 0.78 of full travel |
| The angle loop reached the commanded roll | commanded 17.5 deg, reached 17.5 deg | - |
| Losing the radio stopped the motors | 238 ms after the link went | - |
| The wing flew away from the take-off point | - | 169 m out at its furthest |
| Return-to-home brought it back inside the arrival radius | - | closest approach 12 m (arrive 60 m) |
| And it circled there instead of wandering off | - | circled between 12 m and 67 m before the pilot took over |
| The altitude it captured is the altitude it kept | - | held 14 m, ended at 15 m |
| The navigator says it took over | - | the console reports the return engaged |
| The pilot got it back when the receiver did | - | asked for a left roll, ended at -17.5 deg |

## The mission

The fourth session is the newest rule in the firmware: a navigator flying an
aircraft whose pilot's link is *up*. The simulator types two waypoints and a
start command at the console, and then watches where the aircraft goes - the
only place a navigation claim can be judged.

```
sim:   4001 ms  typed: mission add 52.1254304 4.9876548
mission: waypoint 0 is 52.1254304, 4.9876548 - 'save' keeps it
sim:   4501 ms  typed: set mission_channel 7
sim:   5501 ms  typed: mission add 52.1254304 4.9916792
sim:   6001 ms  mission switch on, at 4 m
status
state:     on autopilot
mission
mission:   2 waypoints, flying
  [1] 52.1254304, 4.9916792  <- flying here
  1 reached, holding 120000 mm
```

| Check | Result |
| --- | --- |
| The mission reached its first waypoint | closest approach 32 m (arrive 60 m) |
| And then the second | closest approach 1 m |
| The navigator counted both arrivals | the console says two reached |
| The flight core says the navigator is flying | `status` says `on autopilot` |
| And the switch that started it ends it | the later `status` says `armed` |
| And it stayed at the last one instead of flying on | 65 m from the last waypoint when the switch went off |
| Holding the altitude it had when the mission started | started at 4 m, ended at 4 m |

The last two are the ones a flight would notice: a mission that arrives and then
keeps going is a mission that has left the aircraft somewhere, and one that
climbs or descends through the whole thing is one nobody asked for the altitude
of.

## The fence

The fifth session is the one automatic behaviour that takes an aircraft off a
pilot whose link is still up. A wing is flown out north with everything
connected, and a 250 m fence brings it back - climbing, on the way, to the 30 m
floor it was given. Then, with the pilot flying again inside the ring, a
**ceiling** goes on above the aircraft and the pilot climbs through it: the same
fence, one dimension up, and the same question - does somebody else take the
aircraft, and does the console say so.

```
fence: 253 m from home, bringing it back
sim:  19631 ms  the fence has it
status
state:     on autopilot
sim:  28631 ms  roll 11.7 deg    <- the pilot has it back, inside the fence
status
state:     armed
sim:  51002 ms  a ceiling goes on at 43 m above home
set fence_ceiling_m 43
fence_ceiling_m = 43
sim:  51003 ms  climbing for the ceiling, 28 m above home
fence: 43 m up, bringing it back
sim:  55023 ms  the ceiling has it, 43 m above home
state:     on autopilot
```

| Check | Result |
| --- | --- |
| The fence noticed the aircraft leaving | the console says the fence brought it back |
| And it brought it back inside | closest approach 31 m (fence 250 m) |
| The flight core says the navigator was flying | `status` says `on autopilot` |
| And the pilot has it back now | the later `status` says `armed` |
| It climbed to the floor it was given on the way | floor 30 m above home, 28 m up when it handed over |
| And it was inside the ring when it handed over | 226 m from home when the pilot took it back (fence 250 m) |
| The ceiling took the aircraft off the pilot as well | ceiling 43 m, highest afterwards 44 m |
| And it came back down under it | 42 m above home and 68 m from home at the end, with the ceiling at 43 |

**The scenario was wrong before the firmware was.** The first version flew a
12-degree circle inside a 150 m fence, which is a turn radius of about 100 m: the
aircraft crossed its own fence on the far side of the turn, was taken over,
handed back, and taken over again, and the check that the pilot had it back
failed. The firmware was doing exactly what it was told. That is now a note in
[14-navigation.md](14-navigation.md) next to the fence parameters, because it is
true of a real wing and a real fence too.

**And the ceiling had the same disease on the first build.** The navigator held
the ceiling itself, which is the number the trigger fires at, so the aircraft
sat on the line and every metre the vertical loop spent above its own target was
another breach: measured in this session, the pilot and the navigator swapped
control **four times in twenty seconds** - take over, hand back, take over -
which is the fence's own turn-radius hazard one dimension up. The navigator now
holds ten metres under the lid (INAV's `geozone_safe_altitude_distance`), with
the margin taken out of the room between the floor and the ceiling, and the same
phase of the same session says the climb stopped within a metre: `ceiling 43 m,
highest afterwards 44 m`.

## The bench session

And the bench session, whose simulator holds the aircraft on each face in turn
and types the six commands itself. The first check is not about the calibration
at all: it is the boot's own record of how far it got, which is the number a
board with no console has to be able to say, and which every session's boot
walks (`src/core/ak_boot.h`, and `05-bringup.md` §6a for the image that blinks
it):

| Check | Result |
| --- | --- |
| The boot reached all thirteen of the stages it has | the last stage marked was 13, the network and the preflight |
| Six faces produced a correction | the console reports a bias and a scale |
| The bias it measured is the part's bias | worst of the three is 0.0000 g |
| The scale it measured is the part's scale | worst of the three is 0.0004 |
| The corrected sensor reads level and nothing else | the flight loop's own sample reads `0 0 999` per-mille of g |
| A multimeter reading calibrates the pack's divider | the ratio it writes is the pack over the pin |
| And the ratio it wrote is the pack over the pin | `vbat_ratio` 11.550 against 11.550 |

(**491 checks across the forty-two sessions**, on top of the suite's own
checks, and the whole lot costs well under a second.)

The part the bench session is holding is not ideal: the simulator gives it a
bias of 30, -20 and 40 mg and a scale of 0.98, 1.02 and 1.005, whose correction
the unit tests compute on their own. So the last line is not "the arithmetic
works" - it is "the number the flight loop is flying on is the right one",
which is a statement about the CLI, the face mapping, the parameter table and
the loop, none of which the unit tests can reach.

Two of those read the firmware's *own console output* rather than the board
seam - or four, counting the bench session's two: the simulator tees everything
the firmware prints, so "the bytes arrived" can become "the position arrived",
"the nav is engaged", and "the sensor is corrected". That is what catches a
parser that accepts the frame and reads the wrong field out of it, or a
navigator that never takes over, or a result printed with a format specifier the
console formatter does not have - all three of which are bugs this repository
had.

The firmware also passes its own preflight check against the simulated board -
including the clock check, which is what the spin accounting in the simulator is
for: a spin on the clock has to look like a tick.

## The bugs this found

The first version of the simulator advanced a millisecond for *every* read of
the clock that had no other board call behind it, and the main loop reads the
clock four times a pass. The firmware's own sense of time therefore ran at about
twice real speed, and the radio and the GPS - which scheduled themselves with
`virtual_ms % period` - landed on an exact multiple of twenty about one time in
twenty. Two thirds of the frames were never *sent*; nothing was dropping them.

The aircraft never armed, and the honest reading of that was "bytes are being
lost somewhere between the ring, the poll and the parser", which is what the
first draft of this document said. It was wrong: the ring had dropped nothing,
and the receiver had decoded every byte it was given. What was wrong was the
clock, and the fix was to charge a spin a microsecond instead of a millisecond
and to schedule the radio and the GPS on deadlines rather than on a modulo.

The lesson is worth keeping next to the code: a simulator that lies about time
will produce a plausible story about the wrong module.

**Then the wing scenario found two more**, both in the navigator, and both
invisible to every unit test that existed: it could never engage at all, because
it read the pilot's link one step behind the flight core, and its altitude gain
was applied to millimetres while being defined per metre, which made the
altitude loop a bang-bang controller. [14-navigation.md](14-navigation.md) has
both, including why the tests could not see them.

That is four now, and the pattern is the same every time: a number that belongs
to two modules, or an order that only matters when the whole thing runs.

## What it does not prove

- **Nothing here has touched silicon.** The ICM-42688-P is a register file this
  file answers, the receiver is frames this file packs, and the airframe is a
  first-order lag. A green run says the software is self-consistent, not that
  any of it is correct about the world.
- **The models are crude.** The quad is torque to rate through a 50 ms lag,
  rates integrated into attitude, gravity in the accelerometer. The wing is a
  point mass: elevons to a rate lag, bank to a coordinated turn, throttle to
  airspeed, pitch to climb. No aerodynamics, no stall, no wind, no battery sag
  under load (the quadrotor's pack steps down once, which checks the arithmetic
  and models nothing), no ESC, and the differential thrust that steers a wing
  in yaw does not steer this one.
- **A green run is not a tuned aircraft.** The two scenarios show that the loops
  converge and the wiring is right. They say nothing about whether any gain is
  *good* - the plants were chosen to be flyable by the gains this repository
  ships, which is the opposite direction of evidence.
- **It does not cover the interrupt path.** On a board the receiver fills its
  ring from a UART interrupt while the loop drains it; here the simulation
  pushes whole frames in from inside the loop. A race between the two is
  invisible to this harness.
- **Nothing here models a part failing slowly** - a gyro whose bias walks, a
  barometer that drifts, a pack whose internal resistance climbs. The failures
  this harness has are steps, not slopes.
