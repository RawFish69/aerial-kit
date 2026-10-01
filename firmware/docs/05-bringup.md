> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - F405 bring-up (M2)

The first time this firmware meets real hardware. Written to be done in order,
to stop at the first surprise, and to leave a record of what was seen.

**This is a dev board on a desk.** No props, no servos, no ESC, no battery.
The board is a WeAct STM32F405RGT6; nothing here is an aircraft.

## Before starting

| Need | Where |
| --- | --- |
| The image | `scripts/fw build aerialkit-f405` in `fc-firmware-workspace`, run on **the machine the board is plugged into** - and then flash the **fitted** one of the two it writes, `dist/aerialkit-f405/aerialkit-f405-fitted.bin`, because this board has its barometer and its pack divider soldered on (§1). Record which machine as well as the hash: the same revision built on the laptop and on this Pi is the same source and a different image (the two GCC builds emit different code), and `scripts/compare-firmware.sh` is what tells the two apart afterwards |
| `dfu-util` | `projects/twin-wings/third_party/tools/dfu-util/usr/bin/dfu-util` |
| The USB cable the board is already on | the console comes out of it as `/dev/ttyACM0` - see step 3a. A 3.3 V USB-TTL adapter on PA2/PA3 at 115200 8N1 still works and is the fallback if no USB device appears |
| A scope, or failing that a stopwatch | for the clock check in step 5 |

## The same list, typed by a script

Everything below can be done by hand, and the hand version is the one to read
first - it is what tells you what "normal" looks like. But most of it is also
one command, which types the same things, reads the firmware's own answers
back, and writes down what it saw:

```bash
make bench-check PORT=/dev/ttyACM0 \
    BENCH_ARGS="--out docs/evidence/bench-f405-$(date +%F).txt"
```

It prints three lists rather than one verdict, because they are three different
things: **verified** (the board answered and the answer was checked), **not
fitted** (the firmware says the hardware is not on this board - a bench board
with nothing soldered to it has no barometer and no receiver; this one has the
barometer and the divider, so its not-fitted list is shorter), and **unverified** (the jumper, and the
calibration, which needs a steady hand and no props). Only a board that does not
answer, or a preflight that reports a problem about the firmware itself, fails
the run - `make test` runs the same checklist against the simulated board,
where it is a test of the checklist.

What it does not replace: the scope on the motor pads (step 3b), the two
calibrations (3e and 3e-bis), the clock check that needs a stopwatch, and the
power cycle (3j) - which is the one that has to come *after* this run rather
than during it, because it takes the console away and the transcript is what it
would be taking it from. Those are the parts that need a person, and they are
why this file still exists.

**It works on a board that is already running, and that took a test to find.**
The first thing the tool does is wait for a prompt, which the *boot report*
ends with - and a board that was switched on before the command was typed has
already printed that report, into a port nobody had open. Neither a USB CDC
device nor a pty replays it, so the run used to fail at the first check with
"no prompt after the boot report" on a board that was perfectly well. It now
asks the smallest question a console answers - a bare newline - and says in the
note which of the two the transcript is. **If you want the boot report in the
artifact, reset the board while the tool is waiting** (it waits
`--timeout` seconds, 20 by default); if you just want the checklist, plug in and
run. The serial path itself is exercised on this machine before any board is:
`tools/bench_port_check.py` puts the simulator on the far end of a pty pair and
drives the tool through `--port`, including the case above and the one where
nothing is on the other end. `make test` runs it.

## 1. Record what you flashed

```bash
cd fc-firmware-workspace
scripts/fw build aerialkit-f405
sha256sum dist/aerialkit-f405/aerialkit-f405-fitted.bin
```

Write down the hash and the revision the build printed. The image carries both
its revision and its build stamp, so a later comparison can tell a rebuild from
a different build - `scripts/compare-firmware.sh` is the tool.

**The harness publishes two images, and the name says which board each one is
for.** `aerialkit-f405.bin` is the bare board, with the barometer and the pack
divider compiled as absent. `aerialkit-f405-fitted.bin` is the same sources
compiled a second time with `AK_BOARD_BARO_FITTED=1 AK_BOARD_VBAT_FITTED=1` -
the board that has the parts of §8 soldered to it:

| File in `dist/aerialkit-f405/` | Which board | What it says about the barometer and the divider |
| --- | --- | --- |
| `aerialkit-f405-fitted.bin` | barometer and divider fitted | they are there, and are read: the preflight's pack line and the navigator's altitude have a source |
| `aerialkit-f405.bin` | bare board | none fitted - reported as missing, with the ADC's counts still shown so the conversion is checkable |
| `aerialkit-f405-telltale.bin` | barometer and divider fitted | the same as `-fitted`, plus the boot's stage on the LED - the image of §6a, for when the console never speaks |

And the wing's board publishes the same two: `aerialkit-ghf435.bin` and
`aerialkit-ghf435-telltale.bin` in `dist/aerialkit-ghf435/` - the tell-tale
there is the answer to the same failure on a board whose console has never
enumerated either, and whose way back into DFU is the awkward one
([26-ghf435-bringup.md](26-ghf435-bringup.md) §2a).

The WeAct board on the bench has both parts on it, so **the image to flash is
the `-fitted` one**; the bare image is what a board out of the bag takes, and
both are kept because the difference is exactly the mistake that would
otherwise be invisible. One `scripts/fw build aerialkit-f405` makes all three -
the fitted pass is a second compile plus `make test check`, which is what makes
the preflight's fitted lines *run* rather than merely compile - and that pass
reports **2,270 checks, 0 failed** against the bare build's 2,269, the extra one
being the check the fitted arm has and the `#else` does not.
**Only the last of those was true before 2026-09-29.** This sentence used to say
the fitted pass "ran the same host checks with 0 failures", and it did - because
it was running the *bare* checks, the flags having reached the image and never
the host build. A count that agrees with the other configuration is not
agreement, it is the same run twice. Measured, fixed, and traps 205.
Only the bare build writes an Intel `.hex`; a person flashes the flat `.bin`
either way.

## 2. Flash it

BOOT0 to 1, tap NRST, then:

```bash
dfu-util -l          # expect 0483:df11
dfu-util -a 0 -s 0x08000000:leave -D dist/aerialkit-f405/aerialkit-f405-fitted.bin
```

That is the **fitted** image, because this board has its barometer and its pack
divider soldered on (§1, and §8 for the two flags behind the name). Flashing
`aerialkit-f405.bin` here would boot a firmware that reports the barometer as
missing and the pack as nothing fitted, which is the wrong answer about hardware
that is present - and the preflight would say so, on the line a person reads
first.

`Warning: Invalid DFU suffix signature` is expected: a flat `.bin` carries no
DFU suffix. The bench work in `projects/f405-bench/` saw the same warning on
every write and the writes verified.

**If the banner never appears, §6a is the next move** - the same board, the
same cable, an image whose LED says how far the boot got.

**And once AerialKit is running, the way back into that bootloader is a
command**: `dfu` on the console hands the part to the ROM, which is the
alternative to reaching for BOOT0 and NRST every time. Expect to unplug and
replug the cable afterwards - the host sees the bootloader's device then, not the
console - and note that the ROM ignored a standalone `DFU_DETACH` on the wing's
part, so the same is likely here.

On a Linux host the bootloader needs permission to be opened at all - one udev
rule is enough:

```
# /etc/udev/rules.d/45-dfu-stm32.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666"
```

`dfu-util -l` printing nothing while the board is *in* the bootloader is a
permissions problem, not a wiring one. The other thing that has bitten this
setup: a hub in between - including the one inside a Raspberry Pi - fighting
the ROM bootloader's full-speed enumeration (`error -22`, `invalid context
state`). The same cable carries a running firmware's USB device happily, so a
bootloader that will not appear on one port is worth trying on another.

## 3. What should happen

The first line to read is the boot's own verdict - `preflight: the machine is
what the firmware thinks it is` - and `preflight` prints the detail behind it.
Anything marked `FAIL` there is a fault in the firmware rather than a missing
part, and it is worth reading before anything else in this file:
[15-preflight.md](15-preflight.md).

On the console, in this order:

```text
AK Firmware
  product:  aerialkit-f405
  board:    AERIALKIT_F405 / WeAct STM32F405RGT6
  rev:      <hash>
  built:    <stamp>
  clocks:   168 MHz sysclk (HSE 12 MHz x PLL), apb1 42 MHz, apb2 84 MHz
  state:    no sensor drivers - the flight loop runs in failsafe
faults: none recorded
selftest:
  ... sixteen ok lines ...
selftest: passed
console: type 'help'
no saved configuration
boot: ok
ak>
```

The status pin blinks at about 1 Hz, and every ten seconds the console prints an
`alive: <ms>, <loops>` line, so a quiet prompt still shows the loop is running.

## 3a. The console over USB, which is the cable already plugged in

The board presents itself as a USB serial device on the same cable that flashed
it - `0483:5740`, the ST virtual COM port, so a Linux host binds `cdc_acm` to it
with no rule of its own. Watch the kernel rather than guessing, because the two
ways this can fail look nothing alike:

```bash
journalctl -k -f          # then press reset on the board
# expect: "new full-speed USB device number N using ..."
#         "idVendor=0483, idProduct=5740"
#         "cdc_acm 1-1.x:1.0: ttyACM0: USB ACM device"
ls -l /dev/ttyACM0
```

Then read it. The baud rate is irrelevant over USB - the line is not a UART and
the firmware answers whatever it is told - so `screen`, `picocom` or `minicom`
at any rate will do, and `cat` alone is enough for the banner:

```bash
cat /dev/ttyACM0
```

**The wing's board does the same thing with a different id.** Its USB device is
the same core and the same driver (src/arch/at32f435/usb.c), answering
`2E3C:5740` - Artery's vendor id, so `lsusb` on a bench with both boards says
which one is on the cable. Its console is also on USART1 (PA9/PA10, 115200 8N1)
for the adapter case, and its bring-up page is
[25-at32-survey.md](25-at32-survey.md): the same list, on a board that has an
inertial sensor, a barometer and two motors instead of none.

What it means when it does not work:

| What you see | What it is |
| --- | --- |
| **nothing** in the kernel log at all | the device never got as far as connecting: wrong pins, or `ak_usb_init` never ran, or - the one this board actually found on 2026-09-17 - **the core was never taken out of power-down and VBUS sensing was never turned off**. Those are two bits in `GCCFG`, and a driver that does not write them is a driver that puts *nothing* on the bus: see the note below |
| a device that appears and then *fails* to enumerate - `device descriptor read/64, error -32`, `Device not responding to setup address`, `error -71` or a timeout | half of it is working: the core connected, and the host's SETUP packets were refused (`-32` is EPIPE, which is a stall - a device that heard the request and said no). **This is what the board did on 2026-09-17, and the cause was a real bug that is now fixed**: the driver read `bRequest` out of the wrong byte of the setup packet, so every request a host makes decoded as a number it did not recognise and was stalled ([01-plan.md](01-plan.md), "The console's other door"). If it happens again, the same two candidates are left - the descriptors or control transfers, **or the firmware is not in its loop any more**, which looks identical from the host side - so check the LED: a running firmware blinks it at 2 Hz, and a steady LED with a device on the bus means the fault is in the boot, not in USB. §6a is the instrument that says how far the boot got |
| `ttyACM0` appears, and `cat` shows nothing | the device is fine and the banner was written before the host was listening - type `status` and press enter, and read the answer |
| the banner, but typing does nothing | the bulk OUT direction, which is the half that carries input. `help` should answer |

**The two bits, because they are the whole difference between silence and a
device.** This part's Synopsys core comes out of reset **powered down** and
**sensing VBUS**, and it holds D+ down while either is true. ST's own driver
writes `GCCFG` immediately after the core reset, and so does every other driver
for this core:

```text
PWRDWN      (bit 16) = 1   "deactivate the power down"
NOVBUSSENS  (bit 21) = 1   no VBUS sensing: this board's PA9 is the receiver's
                           TX pin, not a VBUS sense line
```

`src/arch/stm32f405/usb.c` writes both now. It did not until the firmware ran on
a real board, and **no host test could have caught it**: the tests check what
the driver writes against a mapped page, and a register that is never written
is a register the tests never see. The same two bits are in
`src/arch/at32f435/usb.c` for the wing's board, whose comment used to claim the
F405 did not need them.

The banner is queued while the host is still enumerating, so it arrives after
the port opens rather than being lost; if the first thing on the terminal is
nothing at all, press enter and see whether the prompt answers before
concluding anything.

Type at it. The board prints `ak> ` when it is listening - the prompt is new as
of 2026-09-16, and it is the thing to look for, because a board that came up and
a board that stopped before its console did are otherwise the same silence.
`help` first, then `status`, then `params`; the command list is in
[06-console.md](06-console.md). The two lines worth reading on the first boot:

```text
ak> status
state:     failsafe        <- correct: there is no IMU, so the core holds it safe
loops:     12345           <- climbing means the flight core is running here
timing:    0 gaps, 0 catchup pieces, 0 dropped ms, 0 unusable
timing:    0 duplicate samples, 0 clock resets, 0 long loops, longest 1 ms
links:     6 console, 0 net, 26 rc, 32 gps bytes in one pass at most
attitude:  roll 0 mrad, pitch 0 mrad, yaw 0 mrad
```

The two `timing:` lines are what the loop's own clock did, and on a healthy
board every number on them is zero except `longest`. They are the answer to
"why did it fly badly" when the answer is not the tuning: a sensor interval
longer than `max_dt_ms` is a gap and it is integrated anyway, in pieces of at
most `max_dt_ms` each, but a bus that keeps doing it shows `gaps` and `catchup
pieces` climbing together. `dropped ms` is time the loop could not use at all -
a gap past its catch-up budget, or an interval with no usable reading in it.
`unusable` counts samples the driver marked invalid, `duplicate samples` counts
a sensor whose timestamp has stopped advancing, and `clock resets` counts a
timestamp that went backwards. `long loops` and `longest` are the *loop's* own
period rather than the sensor's, so they are the pair that says the aircraft is
being called too slowly rather than that it is being fed badly - see
docs/29-timing.md.

The `links:` line under them is the other half of that answer: the most bytes
each of the four links has taken off the loop in a single pass since boot. The
receiver's is steady at its frame length and the console's is a handful while
nobody is typing; a link sitting at or near 32 is one being drained to its quota
every pass, which is what a flood looks like from inside the loop.

Then `set` something, `save`, `reboot`, and `load` - that proves the
configuration sector and the flash driver, and it is worth doing once with the
props off, because the erase stalls the CPU for about a second.

## 3b. What the outputs are doing, before anything is wired

`ak> output` prints the DShot rate, the timer's period and the two compare
values, and how many frames have gone out. Then put a scope on the pins - with
nothing connected to them:

| Pin | Expected |
| --- | --- |
| PA6, PA7, PB0, PB1 | sixteen short pulses every 3.33 us of bit time, then a low gap, repeating about 1000 times a second. `set dshot_khz 600` should halve all of it |
| the servo pads | a 1500 us pulse every 20 ms - PA0 and PA1 on the WeAct board, PB8 and PB9 on the Feather, because the servo bank is the board's own choice and not the timer's |

`ak> output` names the servo timer it is actually driving, so the banner is where
to check which row above applies before reaching for the scope: a board whose
servo pads are not the ones you probed shows a correct-looking pulse somewhere
you are not looking.

`docs/07-outputs.md` has the table of what each mistake looks like. This is the
first thing in AerialKit whose correctness a scope can decide, and it decides it
in about a minute - which is worth more than any amount of reading the code.

Then make the pins *move* rather than sitting at zero:

```text
ak> output test
output test: motors to 15%, servos to half travel, one at a time - props off
```

One output at a time, round and round, until `output test stop`. Motor 1 first,
then 2, 3, 4, then the two servos - so this is also how the pads get identified
and how a servo's direction is checked before anything is bolted to it. It
refuses to start while armed and stops by itself if the aircraft stops being
disarmed. **A servo that moves the wrong way is a parameter, not a re-mounted
arm**: `servo1_reverse` (or `servo2_reverse`), with `servoN_trim_us` and
`servoN_travel_us` beside it for a centre and a throw that the linkage got
wrong - [07-outputs.md](07-outputs.md) has what each one is for, and `output`
prints what they are set to.

**And the walk is how a motor number becomes a corner.** The quad-X mixer's
rows are in this order, which is the order every reference target uses and the
reason a person who has built a quad before wires it this way without thinking:

| `output test` says | the pad that must move | spins |
| --- | --- | --- |
| motor 1 | **rear right** | one of the two diagonals |
| motor 2 | front right | the other diagonal |
| motor 3 | rear left | the same diagonal as motor 1 |
| motor 4 | front left | the same diagonal as motor 2 |

Two things about the `spins` column are worth being exact about. The table of
coefficients is in `src/core/flight/ak_mixer.c` with the corner written beside
each row, and `tests/test_mixer_parity.c` holds it against Betaflight's; what
the table does *not* decide is which diagonal turns clockwise, because that is
the props' and the ESC wiring's business ([04-flight-core.md](
04-flight-core.md)). What it does require is that **each motor turns the way
its own prop pushes air downwards**, and that the two diagonals are opposite.
So the check before any prop goes on is: `output test`, identify each pad by
the number that is calling it, and watch the bell (a scrap of tape on it makes
this a three-second job) turn the way its position and its prop need. A quad
whose motors are all spinning the same way does not fly badly - it flips on the
pad - and a motor spinning the wrong way is a swapped pair of wires on that ESC,
not a parameter.

## 3c. The receiver, if one is on the bench

Wire a CRSF receiver to PA10 (its TX to ours), power it, and:

```text
ak> rc
receiver:  8412 bytes, 401 frames, 3 crc errors, 0 rejected
link:      framing
channels:  992 992 172 992 992 1811 172 172
```

Frames should climb at about the rate the receiver sends them, and the channels
should move with the sticks. If bytes climb but frames do not, the baud rate or
the protocol is wrong - [08-receiver.md](08-receiver.md) has the table.

Arming is impossible until the IMU exists, so a receiver on the bench cannot
make anything move. That is a property of this build, not a safety feature to
rely on later: the arming path is meant to be tested on purpose, with the props
off, when there is an attitude estimate to arm with.

### The four directions, which are the check a sign error fails

Once there is an IMU and a receiver, and **with every prop off**, this is the
ten-second check that catches a sign error - and there was one to catch: the
quad's pitch axis was inverted for as long as its mixer table was a copy of a
table written for the opposite convention ([04-flight-core.md](
04-flight-core.md)).

Arm with the throttle at the bottom, then hold each stick and read `status`:

| Stick | What has to happen |
| --- | --- |
| pitch forward | the **rear** motors speed up (the nose goes down). `sticks:` shows a negative pitch |
| pitch back | the **front** motors speed up |
| roll right | the **left** motors speed up (the right side drops). `sticks:` shows a positive roll |
| yaw right | the diagonal pair this firmware speeds up: front-right and rear-left |
| throttle up | all four together |

On the wing the same list reads: pitch forward moves both elevons **down** (the
nose goes down), pitch back moves them up, roll right moves the left elevon up
and the right one down, and yaw right pushes the left motor harder. A servo horn
that faces the other way makes every one of those read backwards, which is why
this is checked before a hinge is connected to anything.

## 3d. The sensor bus, which needs one jumper

There is no inertial sensor on this board, but the bus under it can still be
checked. Jumper PB15 (MOSI) to PB14 (MISO) and:

```text
ak> spi
spi:       sent    55 aa 00 ff 5a a5
spi:       read    55 aa 00 ff 5a a5
spi:       loopback matches - MOSI reaches MISO
```

A match proves the SPI clock, the three pins and the transfer path. `imu` with
no sensor attached should say so rather than inventing numbers - and once an IMU
*is* attached, `imu` naming the part is the moment this firmware can arm a motor
at all. See [09-sensors.md](09-sensors.md).

## 3e. Alignment and gyro bias, once there is a sensor

With an IMU fitted, set the mounting rotation first and calibrate second,
because the calibration is taken in the airframe's frame:

```text
ak> set align_yaw_deg 90        # if the board is mounted sideways
ak> calibrate
gyro bias: roll 0.412, pitch -0.088, yaw 0.155 dps (200 samples, 0 rejected)
ak> save
```

Then tilt the aircraft by hand and watch `status`: rolling it right should move
the roll estimate the same way. If it moves the wrong way, the alignment sign is
what to change - [10-calibration.md](10-calibration.md) says what the angles
mean and what a wrong one looks like.

Then the accelerometer, six positions, because an angle loop flies to whatever
the accelerometer says "down" is:

```text
ak> calibrate accel 0        # level
ak> calibrate accel 1        # inverted
ak> calibrate accel 2        # nose down
ak> calibrate accel 3        # nose up
ak> calibrate accel 4        # right side down
ak> calibrate accel 5        # left side down
accel bias:  0.0300 -0.0200 0.0400 g
accel scale: 0.9801 1.0204 1.0054
ak> save
```

Each face wants about a second of stillness and says which faces are still
missing, so it can be stopped and come back to. If the last one fails with "not
a plausible gravity", one of the six was not flat - start from 0 rather than
trying to work out which. Biases in the tens of milligrams are normal; a scale
outside 0.95..1.05 is a part to look at twice.

## 3e-bis. Receiver centres, once there is a receiver

```text
ak> calibrate rc
rc:        50 frames, centre 1012 (was off by 20, -3, 1 on roll, pitch, yaw)
calibrate rc: done - 'save' keeps it
```

Sticks centred, aircraft disarmed. A receiver 20 counts off centre is a
permanent stick input as far as the flight core is concerned, so the aircraft
drifts on the bench - and that looks exactly like an accelerometer problem. The
four stick channels are measured; the switches are reported rather than averaged
in, because a switch held on is not a centre, and throttle does not vote on
where the middle is because it is being held at the bottom.

## 3f. Return to home, held in your hand

This is the one bench test that needs the aircraft **armed**: a disarmed
firmware does not move an output for anybody, navigator included (that rule has
its own check in `tests/test_nav.c`). It also needs an IMU, because a navigator
with no attitude estimate is a failsafe and not a return. Props off, aircraft
held, a GPS fix, `home` set, and the airframe's mixer selected:

```text
ak> set airframe 1          # 0 is the quad-X, 1 the elevon wing
ak> set rth_enable 1
# transmitter: arm switch on, throttle down, and only then switch it off
ak> status
state:     returning home
ak> gps
fix:       type 3 (ok), 11 satellites
return:    engaged, 1 engagements, holding 120000 mm (fixed wing: bank and circle)
```

With the wing mixer the elevons deflect toward home and the motors go to
cruise; with the quad mixer the motors take a lean toward home instead. Switch
the transmitter back on and control returns to the sticks, which is the half a
pilot cares about.

Before any of that, read one line of the same report: `airframe 1: the
elevon-wing mix and a fixed wing return`. The `airframe` parameter chooses the
mix *and* the return profile, and the preflight checks that the two still agree
- a wing flying a quadrotor's mix is a crash with a plausible number in it. The
same report says which gyro bias the flight will be flown on, measured by the
aircraft at power-up or left over from the bench
([15-preflight.md](15-preflight.md)).

Two ways this step can look broken when it is not. A **disarmed** aircraft: the
arm switch has to be on *before* the transmitter goes off, because a link that
was never up cannot be lost. And a fix the navigator will not use - `gps` says
`usable:    no - ...`, and `preflight` says
`return: enabled, but the fix is not usable`. Both are the firmware being
right; [15-preflight.md](15-preflight.md) is where those lines are explained.

## 3g. Before and after anything that moves

Two of the firmware's own behaviours can be checked on a bench without any
aircraft moving, and both are worth seeing once before they ever see a wing:

```text
ak> set rth_enable 1
ak> set rth_min_alt_m 30
# aircraft held, props off, armed (switch on, throttle down), then the
# transmitter switched off, GPS on the window sill:
ak> gps                 # engaged, holding at least 30 m
```

**Armed**, for the same reason the return test above is: a disarmed aircraft is
disarmed, and the navigator does not engage one - which is a rule, not a gap,
and the reason both of these bench tests look dead on a board that was never
armed.

The floor is the altitude the navigator will hold, so with the aircraft on a
table the pitch command should climb rather than level. And a fence announces
itself - `fence: 253 m from home, bringing it back` - which is the line to look
for while walking the aircraft (or its GPS antenna) out of range.

For a mission, the two things to see on the bench are that the switch starts it
and that the switch stops it:

```text
ak> set mission_channel 7     # the channel you put the switch on
ak> mission add 52.1254304 4.9876548
# switch off -> status says armed
# switch on  -> status says on autopilot, and the elevons move toward the waypoint
# switch off -> status says armed again, and the sticks move the elevons
```

If the switch does nothing, check `rc` for the channel count and the channel's
range: a switch below the arm threshold is off, and one that never goes above it
is a switch the firmware cannot see.

`log reset`, then do the thing, then `log`. The blackbox holds 1.5 seconds at
250 Hz and dumps as CSV, which is the difference between "it twitched" and
eleven loops of gyro, sticks and motor outputs to look at. It is in RAM, so dump
before powering off - [11-blackbox.md](11-blackbox.md).

And if the board *does* reset instead of running, press the reset button and
watch the boot output before anything else:

```text
long log: 372 records from the run before, to 14980 ms into it - 'log long'
boot: ok
```

That is the long log - fifteen seconds at 25 Hz in memory startup does not
clear - and `log long` dumps it after the reset that lost the fast one. It is
the single most useful thing on this bench: `faults: ...` says *that* it
crashed and the long log says what it was doing at the time. It survives a reset
and not a power cycle, so read it before unplugging anything.

## 3h. The log in flash, which survives the power going away

The third log is the one that would still be there after a crash in a field,
and the bench is where it is cheap to find out that it is not writing. It lives
in flash sectors 5 to 9, so it is still there after a power cycle, a reflash of
the *image* would not touch it, and `log flash clear` is the only thing that
erases it deliberately:

```text
ak> status
   ...
   flash log: 4210 records
ak> log flash
# aerialkit blackbox (flash), 4210 records in 2 sectors
time_ms,gyro_x,...
...
```

Three things to look for on the first bench session, in this order:

1. **At boot, the log is empty and says nothing.** A never-used part has six
   erased sectors, so `status` reports `flash log: 0 records` and there is no
   "no room" line. If the boot instead says `flash log: no room - will erase a
   sector on the ground`, the region holds something - another firmware's
   bytes, most likely - and the first sector is erased while disarmed.
2. **Records accumulate while disarmed.** Five a second: leave it a minute and
   `status` should show about 300 more than it did. Nothing about this needs
   the aircraft to arm, which is the point of testing it on a bench.
3. **`log flash` dumps what the RAM logs never could.** Pull the power, put it
   back, and the records from before are still there with `log flash` - where
   `log` and `log long` are empty or stale. That single round trip is the whole
   feature.

What this cannot tell you is how long a *sector erase* really takes. On a part
this firmware has never logged from, all five sectors are erased, so the first
erase is not needed until the ring comes back round - about thirty-six minutes
of logging at five records a second. When it comes it is a one-second stall
with the aircraft on the ground, which is where it is supposed to happen; a log
that would need one in the air stops instead, and `status` says so.

## 3i. The rangefinder, if one is fitted

A part under the fuselage that measures the ground is the one sensor a landing
leans on, and it is the one whose *mount* this firmware cannot know: how far it
sits above the ground while the aircraft is resting on its gear is exactly the
number `range_land_mm` exists to absorb rather than guess. So this check is a
ruler and a console line, not a flight:

```text
ak> range
range:     tof10120 at 0x52, up to 2.00 m
ground:    0.28 m below, measured 30 ms ago
readings:  41 kept, 0 out of range, 0 impossible, 0 failed
landing:   the ground counts as reached within 400 mm, and only while the two heights agree within 3.0 m
```

- Hold the aircraft level at a height somebody measured - a ruler under the
  part, a table, a doorway - and `ground:` is that height plus the mount.
- Put it on the ground where it will land, and `ground:` is the mount on its
  own. `set range_land_mm` to a little above that figure. It is the slack the
  landing rule uses and not a measurement of the gear, and the rule still waits
  for the reading to *stop falling* before it stops the motors.
- `nothing in range` while the aircraft is standing on the floor means the part
  is aimed wrong, the surface under it is not returning enough light, or it is
  not at the address in `src/boards/AERIALKIT_F405/board.h`.
- A part that is fitted and wrong is worse than no part, because the landing
  rule prefers it to the barometer - which is why a reading that *disagrees*
  with the height estimate refuses the landing rather than stopping the motors
  in the air. `range` on the bench is how that is found out before a flight.
- No part fitted is not a fault: the aircraft lands on the barometer, which is
  what every flight in this repository did before there was one, and `range`
  says so in one line.

## 3j. The fault record across a power cycle, and what "none recorded" does not say

Step 3h is the half of this that works: the blackbox is in flash, sectors 5 to
9, so it is still there after the pack comes out. The other half is the one that
misleads.

The fault record - `ak_fault`, the line `status` prints in step 6 - is **in
RAM**, in the `.noinit` section (`src/arch/arm/cortex-m4/fault.c`), which is RAM
that startup does not clear. That is exactly right for what it is for: a reset
does not clear RAM, so a hard fault's pc survives the reset it caused and the
next boot prints it. But a reset is not the only way a board starts again:

| the board is running again because | RAM kept its contents | what `status` says |
| --- | --- | --- |
| `reboot`, the watchdog, or the fault itself | yes | `fault: N recorded`, and N went **up** |
| the pack came out and went back in | **no** | `fault: none recorded` |

The last row is the trap. `fault: none recorded` after a power cycle is not a
finding that nothing faulted; it is what a board with no RAM left from the last
boot says, and the two read identically on the console. A bench session that
power-cycles the board and writes down "no faults" has written down nothing.

**Telling a restart from a fault is one line.** `loops:` on the same `status`
counts from boot and the `timing:` counters under it reset with it, so a
`fault:` line unchanged while `loops:` has gone back near zero is a board that
restarted without recording - and a `loops:` still in the tens of thousands is a
board that did not restart at all, whatever the aircraft appeared to do.

**What the firmware cannot say, measured rather than assumed.** Nothing in
`src/` reads `RCC_CSR`, so no boot can report whether it followed a brownout, a
watchdog, a debugger attach or a clean power-on; those are one silence. The
cause therefore has to come from outside the firmware - a meter or a scope on
the pack, and the line in the transcript where the console stopped. On a board
with `AK_BOARD_VBAT_FITTED` still `0` (the table in step 8), `battery` says none
fitted, so a sag under load is invisible to the firmware as well as to the
record.

**The one power test worth the erase**, with the props off: `set` something,
`save`, then take the pack out and put it back, then `load`. The first two prove
the configuration sector and the flash driver; the third proves that what came
back is the *saved* copy rather than a cached one, which is the distinction
[06-console.md](06-console.md) makes about stale copies. The erase stalls the
CPU for about a second, so it belongs before anything is armed rather than
during.

## 4. The three assumptions

Each one is a single line to correct, and each has a different symptom, which
is what makes them worth telling apart:

| Assumption | Symptom when it is wrong | Check |
| --- | --- | --- |
| 8 MHz HSE crystal (**read off the board**, 2026-09-29 — not the 12 this said for a day; see [02-hardware.md](02-hardware.md) and traps 212) | banner says `HSI, HSE FAILED` and 16 MHz | `src/arch/stm32f405/clk.c` measures it and sets PLLM from the measurement, so there is no number to change; `AK_BOARD_HSE_MHZ` in the board header is the declared fallback the banner prints when the measurement did not fire. That fallback is what a host test can reach, and reaching it is not the same as measuring — and the *measurement* is weaker than it looks too, because `board.c` prints it in whole megahertz: `HSE 8 MHz x PLL` is consistent with a reading anywhere in 7.5–8.5 and did not settle this row. The 8 comes from `PLLM = 8` in the image that printed it with USB enumerating at `PLLQ = 7`, and from a host-timed 168 MHz sysclk |
| Status LED on PC13, active low | banner and `alive` lines are fine, nothing lights | probe the pins, then change `AK_BOARD_LED_PIN` in `src/boards/AERIALKIT_F405/board.h` |
| Console on USART2 PA2/PA3, alternate function 7 | **nothing at all** on the console | adapter RX/TX swapped is the usual cause; then `AK_BOARD_CONSOLE_USART`/pins in the same header for USART1 PA9/PA10 |
| USB on PA11/PA12, alternate function 10 | no `/dev/ttyACM0`, and step 3a's kernel log is empty | the two pins in `AK_BOARD_USB_DM`/`AK_BOARD_USB_DP`; the controller is the part's own OTG FS, so there is nothing to wire |

Two symptoms that look alike and are not:

- **Silence** means no bytes are leaving, so it is the pins or the adapter.
- **Garbage** means bytes are leaving at the wrong rate, so it is the baud or
  the clock, and the clock check in step 5 is the next thing to do.

**One cause of "garbage" was removed before this board was ever flashed, and it
is worth knowing it was there.** The UART divisor was computed as
`(fCK * 16) / baud` where RM0090 30.6.4 wants `fCK / baud`, so every port would
have run at a sixteenth of its rate - a console at 7200 baud against a terminal
at 115200, which is not silence and not readable text. It was found by running
the port's own code against a mapped register block
([21-port-on-the-host.md](21-port-on-the-host.md)) and fixed, and four checks
named for those divisors fail if it comes back. If this board *still* garbles
the console, it is the adapter, the wiring, or the crystal - not that.

## 5. The clock check, because the banner is not a measurement

The banner prints what the *firmware believes*. The SysTick is loaded from that
same belief, so:

> if the `alive` line does not arrive once per second, the real clock differs
> from the assumed one by exactly the ratio you measured.

An LED blinking twice as slowly as it should is a 2:1 clock error, and it is
visible with a stopwatch. A scope on a spare pin (or on the LED) gives the same
answer with less patience: 1000 ms of tick is 1000 ms of tick only if the
clock is what the code thinks it is.

Only after that is a "168 MHz" claim worth writing down.

Both of those need the console, which is the door this page keeps finding shut.
When it is, the same number comes off the status pin instead, in units of
20 MHz: §6b.

## 6. If it resets

The hard fault handler records the fault - status registers and the stacked
registers, including the pc - into RAM that startup does not clear, and the
next boot prints it:

```text
fault: 1 recorded, last pc 0x08000abc lr 0x08000991 psr 0x61000000
       cfsr 0x00000082 hfsr 0x40000000 mmfar 0x00000000 bfar 0x00000000
       r0 0x00000000 r1 0x00000000 r2 0x00000000 r3 0x00000000 r12 0x00000000
       frame 0x2001ffc0 (the stacked registers are here)
```

That line is the whole reason the handler exists instead of a silent loop. Copy
it into the record; `arm-none-eabi-addr2line -e build/aerialkit-f405.elf 0x8000abc`
names the line of code. The last line is where the *originals* of the words
above still are: the record holds copies, and `frame` is the address to dump if
the pc alone is not enough to say what the code was doing.

**That format is checked, not remembered.** `tests/test_fault.c` makes a frame
itself and checks what `ak_fault_record()` does with it - the pc in particular,
which is the number above - and `make test`'s `fault` session plants a record
and boots, so the three lines are asserted whole and the transcript below is
what the firmware actually prints rather than what this page says it prints.
`docs/evidence/sil-fault.txt` is that recording; if a field ever moves, the
session fails before somebody copies the wrong number into `addr2line`.

## 6a. If the LED never blinks: the tell-tale image

A boot that stops before its console is the quietest failure this board has. The
USB core comes up first and connects, so the host sees a device; nothing answers
its SETUP packets; and the LED is steady - which is also what a board nobody
reset looks like, and what the power LED on a WeAct board looks like, since that
one is lit whenever the board has power. On 2026-09-17 the F405 did exactly
this, and the host log could only say *that* it had stopped, never where.

The tell-tale answers where, and it is **the same sources compiled with one more
flag** rather than a second tree that drifts:

```bash
cd fc-firmware-workspace
scripts/fw build aerialkit-f405        # writes all three images
```

`dist/aerialkit-f405/aerialkit-f405-telltale.bin` is the fitted board's
configuration plus `-DAK_BOOT_STAGE=1`, and the harness publishes it on every
build, so it can never be an image of a tree that has moved on. (By hand it is
`make BOARD=AERIALKIT_F405 EXTRA_CFLAGS="-DAK_BOOT_STAGE=1
-DAK_BOARD_BARO_FITTED=1 -DAK_BOARD_VBAT_FITTED=1" all`.)

**Every** image, tell-tale or not, records the stage it is at -
`src/core/ak_boot.h` numbers the thirteen, `ak_boot_mark()` is called by the
code that does each one, and the bench session in `make test` asserts the boot
walked all thirteen. What the flag adds is a *light*:

* it blinks **the stage number three times over** at each of the thirteen
  points, using a spin for the delay rather than `ak_delay_ms` (so the
  instrument cannot hang for the reason it is looking for), and
* a crash blinks **ten quick blinks, a pause, and then the stage it reached**.

```text
1 clock          2 barometer bus   3 console UART   4 USB core
5 tick           6 banner          7 selftest       8 parameter apply
9 outputs       10 battery        11 IMU probe     12 altitude
13 network and preflight
```

| What the LED does | What it means |
| --- | --- |
| a pattern, and then nothing | the boot stopped right after the work that stage does |
| ten quick blinks, a pause, then a stage number | it **crashed**, and that number is how far it got: the record survives in `.noinit`, so unless the crash repeats, the next boot prints the pc and the fault status - §6 above |
| patterns through 13, then 2 Hz | the boot completed and the loop is running: the trouble is in the console, not in the boot - go back to §3a, and if neither console door is open, §6b |

The board has to be in DFU first (BOOT0 to 3V3, tap reset), and the kernel log
is worth watching while it comes back up:

```bash
journalctl -k -f          # in another terminal, before resetting the board
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave \
    -D dist/aerialkit-f405/aerialkit-f405-telltale.bin
```

**The earlier tell-tales are not the instrument any more.** Four of them were
built by hand on 2026-09-17 and kept in `~/mcu-bringup-2026-09-17/`, each a
whole-tree patch against the revision of the moment (`v4` is 102,488 bytes,
sha256 `30b0a810…`). They are worth keeping as a record, and not worth
flashing: they predate the USB decode fix of 2026-09-18, which is one of the
two things this board was waiting for, so a stage read off one of them is a
stage of a firmware that no longer exists. The build above is the same
instrument from the current tree.

**And a dead millisecond tick is no longer one of the patterns.** It used to
be (twenty quick blinks, for ever, from the image that had no other way to say
it), but `ak_delay_ms` gives up and counts the stall now, so a dead tick is a
slow boot that says so in the preflight's stall count rather than a board that
stops - which is why the pattern is gone rather than merely unused.

**And the image is checked for being the instrument rather than only being
named one.** A tell-tale built without the flag blinks nothing, which on a
board with no console is indistinguishable from a boot that stopped before
stage one - so `make check` asserts it in both directions
(`scripts/check-image.sh --telltale`, driven by `EXTRA_CFLAGS`): the
tell-tale's `ak_board_boot_mark` has to be the board's real one with its blink
plan linked, and a flight image's has to be the two-instruction stub with the
plan absent. Both directions were run against real images when the check went
in: with the wrong flag the check fails by name, and with the right one it
prints the size it measured (108 bytes of mark for both ARM boards, against the
stub's two).

What it does not say: nothing in it proves the console works, and a stage
pattern says only that the *step before* it finished. Reading it is worth a
photo - the moment passes, and the answer is three blinks long.

## 6b. When the boot finishes and the console still does not: the USB trace

The tell-tale's last row is where 2026-09-27 left this board. Patterns through
13, then a steady 2 Hz: the boot completed, the main loop is running, and the
trouble is in the console. The console's two doors are the USB port and a UART,
and on this bench both are shut - the host log ends in `device descriptor
read/64, error -71` on every boot, and there is no 3.3 V USB-TTL adapter to read
the other door with. So the banner's own `clocks:` line, which the firmware
computes on every boot and has been computing all along, has never been read.

That line is the first thing worth having, because it is the one hypothesis the
board has never been asked about. A USB device needs a 48 MHz PHY clock. The D+
pull-up does **not** - it is a static current source, and it needs the PHY
powered rather than the clock correct - so a board whose crystal the clock layer
could not use still attaches perfectly, still gets reset by the host, and still
fails every transaction. `docs/02-hardware.md` bounds this board's crystal at
4-26 MHz from the ROM bootloader enumerating; it does not establish 8 MHz, and
`clk.c` divides by `PLLM = 8` on the assumption that it is 8. A 12 MHz or 25 MHz
crystal there asks the VCO for 504 or 1050 MHz, and **as written here on
2026-09-27 this paragraph went on to predict that the PLL would refuse to lock,
that the clock layer would fall back to the 16 MHz HSI, and that USB would
therefore have no clock at all - with a symptom, on the host side, of exactly
`error -71`.** The prediction was half right and the half that failed is the
useful one: the host-side symptom is exactly that, and the clock layer never
noticed. See 6c - the part locks outside its datasheet at 252 MHz with USB at
72, which is why no fallback flag was ever set to read.

The USB trace answers that, and two more questions beside it, out of the status
pin. It is the same sources with one more flag:

```bash
make BOARD=AERIALKIT_F405 EXTRA_CFLAGS="-DAK_USB_TRACE=1" all check
```

It samples the counters **once, at ten seconds**, and then blinks them in this
order, for ever. Ten seconds is the host's budget rather than ours: the four
attempts it makes at a device descriptor take a couple of seconds, so sampling
after them is sampling the whole conversation.

| Group | What it blinks | Read it as |
| --- | --- | --- |
| 1 | the system clock, in units of 20 MHz | **8** = 168 MHz, so the crystal was usable. **1** = the 16 MHz HSI fallback - the PLL never locked, and USB has no 48 MHz clock. That is the answer this instrument was built for |
| 2 | bus resets the core saw | 0 means the host never reset us: the trouble is before the first transaction |
| 3 | SETUP packets handed to the stack | 0 means the receive path is dead; the host is talking and the core is not hearing |
| 4 | control transfers the stack started | 0 with 3 above zero means the core heard and never answered - the transmit end |

Read a group the way §6a reads a stage: **N blinks, three rounds**, 120 ms on and
160 ms off, so a group takes `840*N + 2100` ms - eight blinks is 8.8 seconds and
one blink is 2.9. **Zero is one 1500 ms flash**, because nobody can count a
silence and no group above zero contains a flash that long. Groups are separated
by a 2.5 s dark pause, and 4 s before it starts over.

So the reading is: after the 2 Hz heartbeat stops, count the first group (that
is the clock), and the other three follow it in order. One pass, and then the
board is no longer blinking at you - see 6c, which is where the same four
numbers go and why the pass is not repeated.

**The instrument stays out of a flight image, and is checked for being in the
diagnostic one.** `make check` asserts both directions
(`scripts/check-image.sh --usb-trace`, driven by `EXTRA_CFLAGS`): a flight image
must not link the blink plans, and this one must link *both* the plans and
`ak_usb_trace_get`, since a build that dropped the accessor would still show a
plan and still say nothing about USB. Falsified both ways against real images on
2026-09-27: with `--usb-trace` on the plain image the check fails by name, and
on this one it passes; with no flag on this image it fails, which is the flight
rule still holding.

What it does not say: it is a snapshot at ten seconds and nothing polls USB
after it, so the numbers are the conversation up to that point and not a live
count. It cannot tell a wrong crystal frequency from a PLL that failed to lock
for any other reason - only that the clock layer did not reach 168 MHz. And it
says nothing about the host, the cable, or the port: those stay in the kernel
log.

## 6c. The same numbers, without anybody counting them

Counting 120 ms blinks by eye is a measurement with a person in it, and a
miscount looks exactly like a different number. So the trace image also writes
the numbers into a flash page, in `ak_board_trace_save()`
(`src/boards/AERIALKIT_F405/board.c`), where a host reads them back with no
eyes involved.

It used to do a second thing at the same moment: hand the part to its **ROM
bootloader** with `ak_board_enter_bootloader()`, so that reading the record
needed nobody at the board. **The first shape of that did not work on this
part, and it was measured on 2026-09-27** - see the end of this section. That
shape was a jump from the running firmware; the one in the tree now resets
first and remaps system memory to address 0, and the call is **back in the
ten-second path at HEAD**, put there by `ad766dd`. So the sentence that stood
here - "it has been taken back out of the ten-second path, so no current image
attempts it" - was true for the day between 2026-09-27 and `ad766dd` and is
false at HEAD. `src/core/main.c` carries the call and no longer carries the
comment that denied it.

**The console's `dfu` command reaches that same function, and on 2026-09-30 it
brought `0483:df11` up on the bench board, twice.** What the measurement does
not separate is the two ways that could have happened: the reset-and-remap, or
BOOT0 having been left high, which resets into the ROM whatever the firmware
does. A plain `reboot` at the console sets no backup word, so it is the
experiment that tells those two apart, and it has not been run.

**And the first reading of that DFU - "it did not hand flash back" - was about
the reader and not about the ROM.** From a fresh `dfuIDLE` a reader written for
this bench got exactly four bytes (`00 21 41 92`) and
`LIBUSB_ERROR_PIPE` for every block after it; `dfu-util 0.11`, against the same
ROM entered the same way and arriving in the same state (`status 10 state 10`,
cleared to `dfuIDLE`), read 4096 bytes on the first try. The claim that the ROM
does not serve UPLOAD was a claim about a bespoke program, and it is corrected
rather than kept.

**The read was then taken, on 2026-09-30, with the board entered by hand.**
`0x080C0000`'s first word is `0x414B464C` = `AK_FLASHLOG_MAGIC` and its word 8
is `0x414B4653` = `AK_FLASHLOG_SLOT_MAGIC` - so **the configuration's sector
holds a flash-log header and a log slot: the region's previous owner, still in
it.** That is the mechanism trap 216 reasoned to and could not show. The whole
reading, with hashes and the addressing controls that make it trustworthy, is in
[evidence/f405-config-read-2026-09-30.txt](evidence/f405-config-read-2026-09-30.txt).

Two things it turned up that nobody was looking for. **The trace page at
`0x0801F000` is blank** - all 4096 bytes `FF` - so nothing has been saved there
and 6b is the reading on this board. And **bank 1 at `0x080E0000` is not
erased**: its first 12 KB is written and reproduces byte for byte across two
reads, with all 256 byte values present and none of the three magics anywhere in
it, while the remaining 116 KB of that sector is blank. What wrote it is not
established, and it means trap 216's bench prediction - a save into "the other,
erased bank" - does not describe this board, so what a `save` does here is not
predicted by that paragraph and has not been tried.

The recipe below is unchanged, and is still what to use when the read has to
work first time:

```
# hold BOOT0, tap reset, then:
dfu-util -d 0483:df11 -a 0 -s 0x0801F000:0x1000 -U trace-page.bin
python3 - <<'EOF'
import struct

def fnv(words):                      # the same sum the firmware writes
    h = 0x811C9DC5
    for w in words:
        h = ((h ^ w) * 16777619) & 0xFFFFFFFF
    return h

page = open("trace-page.bin", "rb").read()
for i in range(0, len(page), 64):
    w = struct.unpack_from("<16I", page, i)
    if w[0] != 0x52544B41:           # "AKTR": a slot nobody has written
        continue
    if w[15] != fnv(w[:15]):         # a write that stopped half way
        print("slot %d: torn" % w[1])
        continue
    print("slot %d  sysclk %u Hz  resets %u  setups %u  sends %u" % (
        w[1], w[2], w[3], w[4], w[5]))
    print("        pllcfgr %08x  dsts %08x  dctl %08x  gintsts| %08x" % (
        w[6], w[7], w[8], w[9]))
    print("        passes %u  rx %u = setup %u + data %u + other %u" % (
        w[10], w[11], w[12], w[13], w[14]))
EOF
```

`scripts/trace-page.py` in this repository is that reader, with the register
decoding the shift-and-mask above leaves out; it is the one to use rather than
retyping the block.

**The page is 4 KB at 0x0801F000**, the last page of the sector the image lives
in. It is the only flash on the part nothing else claims: the image below it
ends at 0x0801ADE4 in the trace build and 0x0801AA44 in the flight build, the
blackbox log starts at 0x08020000 above it, and
the two sectors above that are the configuration's banks, each of which is a
ring using all of its slots. Nothing erases this page - the image is written by
whatever erases sector 4 next, and `dfu-util` erases sector 4 with every image
it writes, which is also what makes the page blank again for the next flash.

**A record is sixteen words, 64 bytes, 64 of them to a page:**

| word | what |
|------|------|
| 0 | `0x52544B41` - "AKTR", little-endian, so an unused slot reads all-`FF` |
| 1 | which slot this is: **the newest record is the valid one in the highest slot** |
| 2 | `sysclk_hz` - 168000000, or 16000000 for the HSI fallback |
| 3 | bus resets the core saw |
| 4 | setup packets handed to the stack |
| 5 | control transfers the stack started |
| 6 | `RCC_PLLCFGR`, raw - where the 48 MHz for USB comes from |
| 7 | `OTG_DSTS`, raw - the speed the core settled at |
| 8 | `OTG_DCTL`, raw - soft disconnect, and the global NAKs |
| 9 | every bit `OTG_GINTSTS` was ever seen with, OR-ed together |
| 10 | times the poll ran |
| 11 | entries taken off the receive status queue |
| 12 | ... of which the core called a SETUP |
| 13 | ... an OUT data packet |
| 14 | ... the end of a transfer, or a global NAK |
| 15 | FNV-1a over words 0-14, so a write cut short by a power cut is rejected |

Words 11 to 14 are the ones that separate the two ways a device can be deaf.
`setups` alone says the core was handed no SETUP packet; it does not say whether
the wire never carried one, the core never decoded it, or the queue was never
drained. A `rx` that is not the sum of the three kinds below it is itself the
answer - entries are sitting in the queue unread - and `passes` says whether
anything was polling at all. 11 is deliberately the total: a reader that adds up
12 to 14 and does not get 11 has a record from a build whose counters disagree,
which is worth seeing rather than smoothing over.

Slots fill from the bottom and a record is written only into a slot that is
still blank - programming can only clear bits, so a used slot is never rewritten
and the slot number is a sequence without a clock behind it. A page that holds
something *other* than blanks and valid records is refused rather than
programmed over, and that refusal is also the collision detector: if the image
ever grows past 0x0801F000 the page stops being blank and the save returns -1.

Reading the answer out of it:

- **the page is all `FF`** - the save did not happen. Nothing was written, so
  the lamp is the only instrument and 6b is the reading. (A page whose first
  word is `FF` but which is not all `FF` is a page that has moved.)
- **one valid record, in slot 0** - the first boot wrote it, which is the normal
  case: every flash erases the page.
- **several valid records** - more than one boot got as far as ten seconds
  without being reflashed. **The newest is the valid record in the highest
  slot**, not slot 0; the earlier ones are the boots before it, which is how a
  board that reset itself can be told from one that did not.
- **a slot that is not blank and whose sum does not match** - a write that
  stopped half way. It is reported and skipped rather than decoded, because a
  torn record's fields are a mixture of a record and erased flash.

**The hand-over has now been run, once, and it did not take.**
`ak_arch_bootloader()` sets the ROM's stack pointer and branches to its reset
vector, which is what INAV's own `system.c` does for the same reason. On
2026-09-27 an image carrying it was flashed and left to run: a watcher polled
`lsusb -d 0483:df11` every two seconds for 170 s and **the ROM's device never
appeared**. What appeared instead, 145 s after the flash, was the application's
own device attaching again on port 1-2 and failing to enumerate in exactly the
words it always uses.

So on this part **a jump is not a way in to the ROM's DFU: the ROM reads BOOT0
at reset, and a branch is not a reset.** Finding BOOT0 low it booted from flash,
which is what that second attach is. The consequence for a bench is the
paragraph above - the board does not come back to DFU by itself, and BOOT0 plus
a reset is how it does. Nothing was lost: the record is written before the
hand-over, and the page is only erased by a flash *write* of sector 4, so
reading it does not disturb it. BOOT0 plus a reset gets back to DFU whenever it
is wanted, whatever the ROM decided.

**The reading has now been taken, and this section first read it as closing the
clock question in the crystal's favour. It did not** - the retraction below is
what the reading actually says. The records are in [evidence/f405-usb-trace.txt](evidence/f405-usb-trace.txt), with
the pages themselves beside it, stored as hex in
`evidence/f405-usb-trace-pages.txt` because `firmware/.gitignore` keeps
`*.bin` out of the tree; `scripts/trace-page.py` is the reader. Two builds of the same tree, the same
board, the same cable, differing in one thing - which oscillator the clock tree
hangs off:

| | the HSI build | the HSE build |
| --- | --- | --- |
| `PLLCFGR` | `07005410` PLLM=16 **PLLSRC=HSI** | `07405408` PLLM=8 **PLLSRC=HSE** |
| `setups` / `sends` | 19 / 19 | **0 / 0** |
| `rx` | 50 = 19 setup + 6 data + 25 other | **0** |
| `DSTS` | `00010406`, `SUSSTS=0` | `00000007`, `SUSSTS=1` |
| `GINTSTS|` has | `SOF` `RXFLVL` | neither |
| `passes` | 2786605 / 2870011 | 2866622 |

Both latched `USBRST` and `ENUMDNE`. So both cores saw the bus reset and the
enumeration-done condition; the HSE one then received nothing at all - not one
frame marker - and was suspended by ten seconds, while the HSI one took 50
packets off the queue.

**Neither build enumerates, and this section drew the wrong conclusion from
it.** As written here on 2026-09-27, the argument was: the ROM bootloader is an
HSI-sourced USB device on this same OTG_FS peripheral, on the same pins and the
same cable, and it enumerates on every flash - `0483:df11` appears in `lsusb`
each time `dfu-util` runs - so a 48 MHz HSI clock demonstrably drives working
full-speed USB on this board, our HSI build runs on that same source and still
fails, and **the crystal is exonerated**.

**The premise is false and the conclusion is retracted.** The ROM bootloader is
not HSI-sourced: per AN2606 the F405's ROM measures HSE and runs DFU from it,
which is why it always enumerated and why its success says nothing about HSI.
The reasoning is worth keeping precisely because of how it failed - the ROM's
success was read as the strong half of the argument and the HSI build's failure
as the weak half, and it is the other way round. The HSI build failing was
never evidence about the crystal: HSI is an RC oscillator good to about ±1% and
full speed asks for ±0.25% of the device's transmit, so it would have been
expected to fail either way. What the ROM's enumeration did establish is the
bound `docs/02-hardware.md` still quotes, 4-26 MHz on the crystal - a *range*,
mistaken here for a fact about the source.

The fault was `PLLM = 8` against a 12 MHz crystal, measured 2026-09-28 and
written up in `src/arch/stm32f405/clk.c`. No argument about clock *sources*
could have reached it, because the source was never wrong. `usb.c` was not the
fault either, and that statement was no better tested than this one.

`sysclk 168000000` in both records is not the measurement it looks like:
`clk.c` *assigns* `sysclk_hz` rather than reading it back. Reaching that line at
all does prove `HSERDY` set, the PLL locked and the switch took. **And that is
all it proves: this section first drew a bound from it - "so the crystal is
inside the 2.4-10.3 MHz the VCO maths allows" - and the bound is false.** The
crystal on the board that build ran on is 11.95 MHz, measured the next day, and
it is outside that band. It is a 12 MHz part and **not the WeAct**, whose crystal
is 8 MHz (traps 212) - the bound was false on the board it was drawn from, and it
would have been false on this one too. Every one of those three waits still
passed: a datasheet maximum is a guarantee
inside it and not a cliff at its edge, and the part locked at a 504 MHz VCO
against its own 432 MHz limit. That is why the build ran at 252 MHz and could
not decode a frame, and it is the same mistake as the retraction above, in the
other direction - a real reading asked for more than it measured. `PLLCFGR` in
the table above *is* a raw register read and says which tree was built; it does
not say the tree was a legal one.

**And the lead that was going to need no board - the same clock, two builds, two
different failures - is retired rather than followed.** The HSI *trace* image
above took in 19 SETUPs. The HSI *flight* image - same clock, same USB code,
`build/diag/flight-hsi/` sha256 `4e795eb8…` - died on the host side at the first
descriptor read, `read/8, error -71`, with no device on the bus at all. The two
did behave differently and that was never explained; what retired the lead is
that the fault turned out to be somewhere else - `PLLM` against a 12 MHz
crystal - and measuring the crystal found it without the diff. Both images are
HSI builds and the fault was in the HSE divider: a variable neither of them
carried, so that comparison could not have reached it however it had come out.

## 7. The record

Write it next to the other evidence, `docs/evidence/bringup-<date>.md`:

- the hash and revision flashed,
- the banner as it actually appeared,
- what the LED did,
- anything that disagreed with the assumption table,
- the clock check result, and how it was measured,
- and, plainly, what is still unverified.

**And the six outcomes the qualification is for**, because the list above is
about the board coming up and this one is about what it then does. Each is a
number, a trace or an observed state from the step named, and a step that was
not reached is written down as not reached rather than left out:

| outcome | step | what goes in the record |
| --- | --- | --- |
| boot | 3, 3a | the banner and the `status` lines as printed, with `loops:` climbing |
| sensor axes and freshness | 3d, 3e | the loopback result, the alignment and gyro bias, and the two `timing:` lines - `duplicate samples` and `unusable` are the freshness ones |
| RC loss | 3c, 3g | the four directions, then the transmitter switched off: the status line, and what the outputs did |
| output waveforms | 3b | the pulse train per pin as the scope showed it, and `output test` |
| timing and stack | 3, 6 | the `timing:` lines under load; the stack figure from `make stack-check`, or [evidence/stack-f405.txt](evidence/stack-f405.txt) when the toolchain is on another machine |
| power reset | 3h, 3j | the configuration round trip, what `status` said after the pack went back in, and the outside measurement of any sag |

`tools/bench_check.py` types the boot, sensor and receiver checks at the console
and writes its own transcript next to this record - so the first three rows come
back as text. The other three do not: switching the transmitter off, watching a
pulse train on a scope, and pulling the pack are hands-on, and they are the ones
a transcript cannot stand in for.

Then update [01-plan.md](01-plan.md): M2 is not done until its evidence is in
this repository, and "it printed the banner" is not the same claim as "the
clock is right".

## 8. The four things that are switched off until something is soldered

Four board facts are deliberately `0` in `src/boards/AERIALKIT_F405/board.h`,
because the firmware should not read a pin that has nothing on it and call the
number a measurement. Each is one line, and each is the difference between a
section of this file working and doing nothing:

| Fact | What is missing | What works once it is 1 |
| --- | --- | --- |
| `AK_BOARD_VBAT_FITTED` | a 10k from the pack to PC0 and a 1k from PC0 to ground | `battery` and the preflight's pack line, which otherwise say "none fitted" while still showing the ADC's counts so the conversion itself is checkable ([19-battery.md](19-battery.md)) |
| `AK_BOARD_RC_INVERTER` | one transistor in front of the receiver pin, or an un-inverted SBUS pad on the receiver | SBUS. CRSF needs none of it ([08-receiver.md](08-receiver.md)) |
| `AK_BOARD_BARO_FITTED` | any barometer on I2C1 PB6/PB7, and its pull-ups | the barometer, and the navigator's altitude with it ([20-i2c.md](20-i2c.md)) |
| `AK_BOARD_RANGE_FITTED` | a rangefinder on the same two I2C1 pins at 0x52 - a TOF10120 and its pull-ups | the last metres of a landing, and a quadrotor's descent onto the ground below it when it has no position left ([09-sensors.md](09-sensors.md), step 3i above) |

None of the four is a hazard while it is off: the bus is configured, the pin
is read, and what is missing is reported as missing. Setting one to 1 without
the part is the only way to make them mislead, which is why they are facts in a
board header rather than things the firmware tries to detect.

**And the code behind each of them is compiled on every build**, whether or not
the part is fitted: `scripts/fw build aerialkit-f405` builds this board a second
time with `EXTRA_CFLAGS="-DAK_BOARD_BARO_FITTED=1 -DAK_BOARD_VBAT_FITTED=1"` and
checks the image it produces. The flags are `#ifndef`-guarded for exactly that -
a build can set what a hand can - and the reason is not tidiness: the ESP32's
Wi-Fi half was the configuration no build ever compiled, and it did not compile
for days ([02-hardware.md](02-hardware.md)). Whichever row above gets soldered,
the code that reads it has already been through a compiler.

**And the divider, once it is soldered in, is calibrated rather than computed**:
put a multimeter across a pack, type `calibrate vbat <what it says>`, and the
firmware reads the pin and works out `vbat_ratio`. Doing that arithmetic by hand
is how a pack ends up reading twice as healthy as it is, which is a flight that
ends early and badly - [19-battery.md](19-battery.md).

**Something here has been on the board once, and it is worth being exact about
how little that proves.** An image was flashed through the ROM DFU bootloader on
2026-09-16 - rev `04379e4`, `File downloaded successfully`, the board left the
bootloader - and nothing was read back from it: that image predates the USB
device, there is no adapter on PA2/PA3, and the LED was not reported either. So
everything above is still what the code says it will do, run against fakes, a
simulator and a register block ([21-port-on-the-host.md](21-port-on-the-host.md))
- and a register block is not a chip with a crystal, a bus with capacitance or
an ESC that answers. The flash that matters is the next one, because it is the
first one that can be read back over the cable.

## 9. And then it flies

This page ends where the aircraft can answer questions. What comes next - the
ground tests with the props off, the first hover and the first launch, the order
the gains get tuned in, and what to read in the log afterwards - is
[23-first-flight.md](23-first-flight.md), which assumes this page went green.
