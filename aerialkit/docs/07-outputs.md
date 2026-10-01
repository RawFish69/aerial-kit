# AerialKit - motors and servos

The path from a control decision to a pin:

```text
mixer -> ak_output_encode()      DShot frames (16 bits) + servo pulse widths (us)
      -> ak_board_output_write() the board's timers and DMA
      -> PA6 PA7 PB0 PB1          motors, DShot on TIM3
         the board's own pads    servos, PWM on the board's own timer
```

## Pins

The pins are two different kinds of thing, and the split matters.

**The motors are the timer's.** DShot needs one timer period per bit and one DMA
burst that writes all four compare registers at once, so the four channels are
one unit and there is no second timer to move them to. TIM3 channels 1-4 on
PA6, PA7, PB0 and PB1 is what that costs; none of them is the console
(PA2/PA3), the LED (PC13) or USB (PA11/PA12).

**The servos are the board's.** They are ordinary PWM and nothing ties them to
the motor timer, so which timer and which pads is a board fact and is passed to
`ak_output_init()` rather than chosen by the arch layer - the same rule the
console's pins, the receiver's port and the USB pair follow. On this part it is
not cosmetic: the boards bring out different pads.

| Board | Servo 1 | Servo 2 | Alternate function |
| --- | --- | --- | --- |
| WeAct `AERIALKIT_F405` | PA0, TIM2_CH1 | PA1, TIM2_CH2 | AF1 |
| Adafruit Feather `FEATHER_F405` | PB8, TIM4_CH3 | PB9, TIM4_CH4 | AF2 (Feather pins 9 and 10) |

The Feather is the board where the distinction has teeth: its breakout brings
out neither of the WeAct's servo pads, so a Feather built against the arch's own
choice drove two pulses into nothing while reporting that it had two servos. The
channel is per-servo for the same reason - TIM4's free pair is channels 3 and 4,
so the pulse width goes in `CCR3`/`CCR4` and not `CCR1`/`CCR2`.

Which of those pads reach a header is what `ak_board_output_shape()` reports, and
that is the number the arming gate compares an airframe's mix against. Whether a
real ESC and servo accept what comes out is a bench question. The firmware side
of it is checkable right now.

**And the arming gate has to be given something to pass**, which is the other
half of the same fact and the one the Feather board is the reason for. The
`airframe` parameter starts at whatever the board says it is built into -
`ak_board_default_airframe()`, asked once in `main` between `ak_flight_init()`
and the parameter table's registration, so it is what `defaults` restores and
what a fresh board with no saved configuration flies. A board that answered
nothing would take the core's quad-X, four motors, and refuse to arm against
its own pads:

```text
FAIL  this mix needs 4 motors and 0 servos; the board drives 2 and 2
```

That refusal is correct and it is not the bug - a two-motor board flying a
four-motor mix would be. The bug was that no board could say what it *was*, so
the answer is a board contract entry and not a weaker check:

| Board | `ak_board_default_airframe()` | Why |
| --- | --- | --- |
| `FEATHER_F405` | `7` - the single-motor elevon wing | brings out 2 motor pads (PA6/PA7) and 2 servo pads (PB8/PB9) |
| `AERIALKIT_F405`, `AERIALKIT_GHF435`, the four ESP32 devkits | `0` - the quad-X, the core's own default | four motor pads, which is what the quad-X mix needs |

It is a *default* and not an override: a saved configuration still wins, and
`set airframe 7` moves to the same place from anywhere. The parameter's own
range and numbering are in [04-flight-core.md](04-flight-core.md).

## Motors: DShot on a timer with a DMA burst

One timer period is one DShot bit, so the period *is* the bit time and the
compare value is the duty cycle the ESC reads.

| | DShot150 | DShot300 | DShot600 |
| --- | --- | --- | --- |
| Bit rate | 150 kHz | 300 kHz | 600 kHz |
| Bit time | 6.67 us | 3.33 us | 1.67 us |
| TIM3 ticks per bit (84 MHz timer clock) | 560 | 280 | 140 |
| Compare value for '0' (35%) | 196 | 98 | 49 |
| Compare value for '1' (70%) | 392 | 196 | 98 |

DShot asks for 37.5% and 75%; this uses 35% and 70%, which is what the
reference implementation uses and which leaves both bits unambiguous to an ESC
that samples near the middle of the bit. `set dshot_khz 300` changes the rate,
and `output` prints the numbers that are actually in the registers.

Each motor's frame is 16 bits, most significant bit first, followed by two
blank bit times that hold the line low between frames. That is 18 groups of
four compare values, one per motor channel, interleaved so that a single DMA
burst writes all four channels at once:

```text
entry:   [m1 m2 m3 m4][m1 m2 m3 m4] ... [m1 m2 m3 m4][0 0 0 0][0 0 0 0]
group:         bit 15        bit 14          bit 0       gap      gap
```

Three details in the timer setup are load-bearing, and each one is a failure
mode worth naming:

- **Compare preload is on.** A write to a compare register lands in the preload
  register and takes effect at the next period boundary. Without it, a DMA
  transfer arriving mid-bit can extend or clip the bit it arrives in.
- **The burst trigger is the compare event of channel 1**, so there is exactly
  one burst per bit period. `DCR.DBA` is 13 - `CCR1` is at offset `0x34`, which
  is 32-bit word 13 - and `DCR.DBL` is 3, which is four 16-bit transfers:
  `CCR1`, `CCR2`, `CCR3`, `CCR4`, in the order the buffer is laid out in.
- **The first bit of every frame is loaded by hand.** The DMA's first burst
  lands one period later than the values it carries are meant for, so the
  buffer starts at bit 2 and bit 1 is written directly to the compare
  registers. Getting this wrong shifts every throttle value by one bit.

The DMA stream is DMA1 stream 4, channel 5, into `TIM3_DMAR`. That mapping is
the one `timer_def_stm32f4xx.h` in the INAV checkout carries for TIM3_CH1
(`D(1, 4, 5)`), and the stream's transfer-complete interrupt at vector 15 is
what sets `busy` back to zero so the next frame can be handed over.

## Servos: ordinary PWM

The board's servo timer - TIM2 on the WeAct board, TIM4 on the Feather, both on
APB1 and so both at 84 MHz - counts microseconds (prescaler 83), the period is
20 ms, and a compare value *is* the pulse width: 1000 to 2000 microseconds,
1500 at centre, clamped by the pure encoder in `ak_output.c`. A servo channel is
therefore the same numbers a servo datasheet uses, and the scope should agree
with the register. `output` names the timer from the base the board handed in,
so the banner cannot say TIM2 while the pulses come out of TIM4.

## And the plumbing between a servo and a surface

**And the wing's mixer has an actual reference, which it did not when it was
written.** A twin-motor flying wing with elevons is this project's airframe and
neither upstream ships that table - but the aircraft *flies* one today, and its
saved configuration is in this repository: `projects/twin-wings/ghf435-inav/`
`inav-preset-label-set.txt`, the session that ends in `save`:

```text
mmix 0  1.000  0.000  0.000  0.500     mmix 1  1.000  0.000  0.000 -0.500
smix 0 1 0  50 0 -1    smix 1 1 1  50 0 -1
smix 2 2 0 -50 0 -1    smix 3 2 1  50 0 -1
```

`tests/test_mixer_parity.c` now compares this firmware's `elevon-wing` table
with those numbers, and the two conventions between the tables are what the
comparison has to name before it can pass:

| | INAV's saved mixer | AerialKit's table |
| --- | --- | --- |
| the control axes (roll, pitch, yaw) | half-scale coefficients (±0.5), clamped by its ±500 mixer output | full-scale coefficients (±1.0), clamped by `torque_limit` (0.6 of an axis) |
| the throttle | 1.000, the mean of the two motors | 1.000 - the same, because a doubled throttle coefficient is twice the throttle at full stick |
| the two elevons | both reversed (`-1` on each `smix` row) | the opposite sign - a horn goes on the way the builder mounts it |

The check is a *ratio*: change one of those numbers alone - halve the motors'
yaw coefficients to match INAV's own file, say - and it fails
("and it is the aircraft's own mixer up to its scale and its servo direction"),
because the two tables' conventions have to stay in step or the aircraft is a
different aircraft. The effective authority is the arithmetic that makes them
comparable: INAV can ask for 0.5 of an axis, this firmware for 0.6.

**And the last row is a bench instruction, not trivia.** INAV's saved mixer has
**both** elevons reversed, and this firmware's table has the opposite sign on
both - so with the servo horns where INAV had them, a roll command moves both
surfaces the same way and the wing does not turn at all. That is exactly what
the `servoN_reverse` parameters above are for, and it is why the bench session
sets them (`26-ghf435-bringup.md` §4, step 5) before anything is armed. A
parity to the aircraft's own mixer is worth having *because* the first flight is
where it is checked.

The mixer says how far an elevon should move and which way. Whether the servo
*does* is a question of which side of the aircraft its arm is on and how long
the pushrod is - both of which a person fixes with a servo arm, and when the arm
cannot be moved, with three parameters per servo:

| Parameter | What it is for |
| --- | --- |
| `servo1_reverse` / `servo2_reverse` | a linkage that moves the surface the other way from the one the airframe's table assumes. **This is the one that stops a wing flying**: with one elevon's linkage mirrored, a roll command moves both surfaces the same way and the aircraft does not turn at all |
| `servo1_trim_us` / `servo2_trim_us` | a centre that is not 1500, from -200 to +200 |
| `servo1_travel_us` / `servo2_travel_us` | how far the linkage moves at full stick, 100 to 900 us, which is how two elevons with different geometry are made to deflect by the same amount |

The defaults are the neutral linkage - not reversed, no trim, 500 us each way -
and a quadrotor never changes them, because it has no servos to trim. Whatever
the numbers are, the mapping clamps at 750 and 2250 us: a parameter or an
arithmetic mistake must not be able to hold a servo against its stop, because
that is a servo that burns.

`output` prints all three for each servo, which is the command to read before a
first flight:

```text
ak> output
servo 1:   normal, trim 0 us, 500 us at full stick
servo 2:   reversed, trim -20 us, 480 us at full stick
```

## What the scope should show, and what it would mean

With nothing wired to the board, this is a signal on a pin. With the props off
and an ESC wired, disarmed DShot frames keep it disarmed.

| Check | Expected | If it is not there |
| --- | --- | --- |
| Motor pins idle | low between frames | the timer or the GPIO alternate function is wrong |
| Bit period on any motor pin | 3.33 us at DShot300, 1.67 us at 600, 6.67 us at 150 | the timer clock assumption is wrong: check `output` against the scope |
| '1' and '0' duties | 70% and 35% of the bit time | the burst wrote to the wrong registers, or the preload is off |
| One frame | 16 pulses then a low gap, repeating at about 1 kHz | the DMA is not restarting, or the trigger is wrong |
| Servo pins | a 1500 us pulse every 20 ms | the servo timer or its alternate function is wrong |
| `output` | frames sent climbing, skipped near zero | the loop is running slower than the frames, or the DMA never completes |

A disarmed DShot frame is all zeros, so the motor pins show sixteen short pulses
per frame. That is a weak test of *timing* and a poor test of *data*: to see a
'1' bit, something has to be commanding thrust. Nothing used to be able to,
because the only path to the outputs goes through the flight core and that needs
an armed aircraft - so a board on a bench could not be made to move a pin at
all.

`output test` is that path, and it is the one thing in this firmware that writes
the outputs without arming:

```text
ak> output test
output test: motors to 15%, servos to half travel, one at a time - props off
             it repeats until 'output test stop'; the frame counts are above
ak> output test stop
output test: stopped, everything at zero
```

It walks the outputs in turn - motor 1, motor 2, motor 3, motor 4, servo 1,
servo 2, then round again - each one rising to its value, holding, and falling
back, which is how the pads get identified on a new board and how a servo's
direction gets checked before it is anywhere near a hinge. Three rules make it
safe: it refuses to start while armed, it stops by itself if the aircraft stops
being disarmed, and it ends with everything at zero rather than wherever the
sweep happened to be.

**The first two of those are on tape now.** The refusal is in the quadrotor's
own session (the command typed at an armed aircraft); the interlock is
`testarm` ([evidence/sil-output-test-arm.txt](evidence/sil-output-test-arm.txt)),
which starts a sweep on a *held* aircraft - the bench case, props off - and
throws the arm switch under it. What the check asks for is two-sided: the sweep
has to have moved a motor before it stopped (or "it stopped" would also be true
of a test that never started), the firmware's own sentence has to appear once,
and what the four motors are left at is the aircraft's armed idle - 0.072 each
in the simulation - rather than one of them still at the sweep's 0.150. The
third rule, the explicit `output test stop`, is the line in the transcript
above, and it does end at zero.

So the data path is now a bench check as well as a host test: with the props
off, `output test` moving one motor at a time shows both the timing *and* the
frame - and the check in the simulator is that the same command moved every
motor and every servo and left them at zero, watched from the board side of the
frames.

## Scores on the door

Everything here is unverified on hardware except the pure parts. The host tests
check the bit layout, the duty arithmetic and the register encodings, and none
of them can see a waveform; the bench checks in the table above are what turn
the rest into facts, and `output test` is what makes them possible.

## The other target: DShot out of an RMT

The ESP32 has nothing like a timer with a DMA burst. It has the **RMT**, which
is a small engine for saying "high for this many ticks, low for this many,
next", so the *shape* of the problem is different and the arithmetic is not:
`ak_dshot_edges()` in the core turns a frame into levels and durations, and the
host tests check it two ways - by decoding its durations back into the frame an
ESC would see, and by comparing the bits it sends against the bits the timer
encoder sends for the same frame. The duties are the same 35% and 70% for the
same reason: a scope on either target should show the same waveform.

The tick is 100 ns, because the RMT's clock is the 80 MHz APB divided by a whole
number. DShot600 is 1667 ns a bit - 16.67 ticks, so it is rounded to 17 and the
frame is 0.2% fast, which every ESC has margin for; what none of them has
margin for is a wrong bit, which is why that encoder is a function with tests
rather than a loop in the port.

The servos are **LEDC** channels: 50 Hz, 16-bit duty, and the pulse width from
the same `ak_servo_pulse_us()` the F405 uses.

Two things this cost, both worth knowing before the next port:

- **`rmt_transmit()` blocks forever** unless it is told not to. Its
  `queue_nonblocking` flag is not a tuning knob: without it, a channel with no
  free transaction descriptor *waits*, and on a chip whose RMT never completes
  a transmission - QEMU's, which has no such peripheral - the flight loop stops
  at the first frame it cannot hand over, and the console stops with it. That is
  how it was found: the protocol check timed out mid-conversation.
- **IDF logs an error every time that call fails**, at error level, which is
  the level this port keeps for the console. A thousand frames a second of
  "no free transaction descriptor" is a console nobody can read. The driver now
  tracks a per-channel idle flag from the transmit-done callback and simply
  does not call transmit on a busy channel: a skipped frame is counted, which
  is what a motor command is worth.

Under emulation the RMT never completes, so no frame is ever sent and the
driver says so - "configured, but no frame has completed" - and `output ready`
is 0. That is the honest state of a port that has been compiled and booted and
has never driven an ESC.

## And the third thing, which the emulator could not have said

Everything above is about a driver that had been compiled twice, booted once,
and never run: no environment on this machine completes an RMT transmission, so
the code between "a frame arrives" and "the peripheral has it" was the one part
of this file that no check and no boot touched.

It runs now, against a stand-in for the RMT and the LEDC
(`tests/idf-stub` - IDF's declarations, and a model of what IDF does with the
arguments), and the check reads the symbols back out of the model and decodes
them into the frame an ESC would see. **The frame that came back was four bits
long**: `rmt_transmit()` takes the payload's size in **bytes**, and its copy
encoder walks that many bytes as `rmt_symbol_word_t` - four bytes to a symbol.
The port was passing the *symbol* count, so a seventeen-symbol frame went out as
seventeen bytes, four and a quarter symbols, and every motor command this
target ever sent was a truncated frame. The line says
`sizeof(rmt_symbol_word_t)` out loud now, and the check that found it is
`tests/test_arch_esp32_output.c` - the same method this project used for the
F405's flash controller and its I2C bus, applied to the one driver on the second
target that nothing could run.
