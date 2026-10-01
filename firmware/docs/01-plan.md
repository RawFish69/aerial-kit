> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - plan

The milestone list from the goal, with the state of each one. Milestones are
not "done" because code exists for them; they are done when the proof described
here exists in this repository.

## M0 - skeleton on F405  (in progress)

What must be true:

- `make` builds `aerialkit-f405.elf/.bin/.hex` with the shared ARM GCC.
- The image links at `0x08000000` with the vector table at file offset 0, so
  the STM32F405 ROM bootloader can take it — the path
  `fc-firmware-workspace/projects/f405-bench/` already proved on this board.
- The workspace builds it as `scripts/fw build aerialkit-f405`.
- The image carries the git revision and the build stamp, so
  `scripts/compare-firmware.sh` still separates "same code" from "different
  code".
- On the board: a status pin blinks, and a banner naming the firmware,
  revision and system clock goes out on the console UART.

What is *not* claimed: that the banner has ever been read. An image has been on
the board once - rev `04379e4`, flashed 2026-09-16 through the ROM DFU
bootloader, `File downloaded successfully` and the board left the bootloader -
but that image predates the USB device below and the board has no USB-TTL
adapter on PA2/PA3, so the console has never been seen. The LED *has* been
reported since, and it was **steady** - which is a fact with three readings
rather than a missing one: the pin blinks three times at boot and then toggles
twice a second while the loop runs (`AK_BOOT_BLINKS`, `AK_LED_PERIOD_MS` in
`main.c`), so a steady *status* pin is a boot that never reached the loop; the
power LED of a WeAct board is steady whenever the board has power, which is
the same thing to look at and not the same thing; and the pin is active low, so
"lit" and "off" are the reverse of what a person expects. §6a of
[05-bringup.md](05-bringup.md) localises it, and it is a build of the current
tree now rather than a patch kept outside it.

### The console's other door: USB, flashed, and one bug it found on the bench

The board is plugged into the machine that builds it, so the console has a
second route that needs no adapter: an OTG FS device, polled, presenting itself
as `/dev/ttyACM0` to the host it is already attached to. Both bulk directions
are wired - the banner out, and typing in, where `ak_usb_read()` is what the
console pulls on. What makes it believable before the first flash is that
`tests/test_arch.c` executes it against the mapped register block: enumeration,
the descriptors, the class requests a serial driver makes, a reset, a burst
that overruns the ring and typing at the console, 30 checks. Three bugs came
out of that - a status word read from the wrong register, an endpoint never
armed to accept setup packets, and a banner queued and never sent - and
[21-port-on-the-host.md](21-port-on-the-host.md) has them in the table they
deserve. What the host could not say is whether the silicon agrees - and on
2026-09-17 the board answered, in two steps.

**The first flash put nothing on the bus at all.** Not a device that failed to
enumerate: no `new USB device` line, nothing, on the same cable, port and pins
that had enumerated the ROM's own DFU a minute earlier. The cause is in every
driver for this core and was missing from this one: the OTG core comes out of
reset **powered down** and **sensing VBUS**, and it holds D+ down while either
is true, so `GCCFG`'s `PWRDWN` (bit 16) and `NOVBUSSENS` (bit 21) have to be
written before a host can see anything. `src/arch/at32f435/usb.c` had both bits
from the start and its comment said the F405 did not need them; the F405 needed
them. **The host tests were green through all of it** - they check what the
driver writes against a mapped page, and a register the driver never writes is
a register they never look at.

**The second flash, with those two bits written, put a device on the bus.** The
kernel log shows `new full-speed USB device` and then `device descriptor
read/64, error -32` - the core is powered, connected and being reset by the
host, and nothing is answering its SETUP packets. That was read at the time as
a boot that had stopped before its loop, because the board's LED was steady
rather than blinking - and there were *two* things wrong, which is why it took
two readings.

**The stall was the driver's, and the host tests had agreed with it.** `error
-32` is EPIPE: the device received the request and refused it. `usb.c` computed
the request as `setup[1] * 256 + setup[0]`, reading `bRequest` out of the
little-endian pair rather than out of byte 1 where USB 2.0 section 9.3 puts it,
so a real `GET_DESCRIPTOR` decoded as 0x0680, matched no request the driver
knows, and fell through to the stall. Both USB ports had the line, because the
second is the first re-derived, and no test could see it: the modelled FIFO was
one word, so a setup packet was the same four bytes read twice and the test
wrote the request into byte 0. Driver and test agreed with each other and
neither agreed with a host. The fix is one byte on each port; the test's is a
FIFO that is a queue and packets that are the ones a host sends (`SET_ADDRESS`,
which had never been sent, among them); and the proof is that restoring the old
decode fails sixteen checks. Both `usb.c` files are now at no line the host
suite does not reach.

**The steady LED is the other thing, and it is still open.** A stall is not a
hang, so that evidence does not say where the boot stopped - the tell-tale that
blinks the stage it reached is still the instrument for it, and it is now a
build of the current tree rather than four hand-kept patches
(`05-bringup.md` §6a, `src/core/ak_boot.h`).

**Both halves of that are recorded verbatim** - the flash, the empty kernel log,
the enumeration attempts and their errors, and what is missing from them -
in [evidence/f405-usb-bench.txt](evidence/f405-usb-bench.txt), which now ends
with what the kernel log turned out to mean and what the next session should
see instead: `cdc_acm` and a `/dev/ttyACM*`, then the banner.

What the host still cannot say about USB is whether enumeration *completes* -
that is the bench's, and it now has a board that says exactly how far it gets.

It costs what the two rings cost and no more: 74604 B of flash and 55552 B of
RAM with it in, against 73908 B and 55268 B at the revision before it, built the
same way on this machine - 696 B of flash for the receive path, its ring and the
class requests, and 284 B of RAM for the 256 bytes of ring. Those two numbers
are from 2026-09-16 and the image has grown since (104,008 B of flash and
78,432 B of RAM at the revision this page is being kept in step with, both
counted the way `arm-none-eabi-size` prints them: text + data, data + bss); what has
*not* changed is the shape - two rings, one polled, and everything above them
portable.

### M0 evidence, measured on the NAS 2026-09-14

Everything here is from the artifacts and the build output, not from intent.

| What | Result |
| --- | --- |
| Build | `arm-none-eabi-gcc` 13.2.1, no warnings: 4400 B text + 12 B data + 112 B bss, 0.42% of the 1 MB part |
| Image shape | `scripts/check-image.sh` passes: vector table at file offset 0, initial SP `0x20020000`, reset vector `0x080004f1`, SysTick `0x08000841` distinct from the fault handler, reserved entries zero |
| Reset path | disassembly: CPACR `|= 0xF00000`, VTOR `= 0x08000000`, `.data` copied `0x08001130` → `0x20000000..0x2000000c`, `.bss` zeroed to `0x2000007c`, then `main` |
| Clock | `FLASH_ACR = 0x705` (5 wait states, prefetch, I/D cache), `PLLCFGR = 0x07405408` (M=8, N=336, P=2, HSE, Q=7), `CFGR`: PPRE1 /4, PPRE2 /2, SW=PLL |
| Rebuild | two clean builds seconds apart differ in 3 bytes, offsets 4135..4138 → `scripts/compare-firmware.sh` says `stamp-only` |
| Dead code | `-ffunction-sections -fdata-sections` in `CFLAGS`, which is what makes the linker's own `-Wl,--gc-sections` mean anything: without them a function nothing calls is kept because the function beside it is called. Added 2026-09-18, and the bare image went **106,392 → 103,892 B** - 2.5 kB, 44 symbols, every one of them a function no path reaches (the unused half of several modules, plus the boot tell-tale's own pattern arithmetic in the build that does not blink). The ESP32 port always had these flags, because ESP-IDF sets them |
| Same-second rebuilds | two builds in the same second, from two different build directories on the same host, came out byte-identical (`e9910282…`, 10632 B) - the stamp is the only thing that moves between builds, so two builds that land in the same second are the same file |
| Harness | `scripts/fw build aerialkit-f405` → `dist/aerialkit-f405/` with **three** images, `aerialkit-f405{,-fitted,-telltale}.bin` (bare board; the board with the barometer and divider of `05-bringup.md` §8 soldered on; and that same board with the boot's tell-tale switched on, §6a there), the bare `.hex`, three `.elf`s and a `SHA256SUMS` over all seven; the fitted pass is a second compile plus `make test check`. **Corrected and then repaired, 2026-09-29:** this row said that is "what makes the fitted lines *run* rather than merely compile", and for a day it was not - `$(EXTRA_CFLAGS)` is added to `$(CFLAGS)` and never to `$(HOST_CFLAGS)`, so the fitted pass's host binary was compiled exactly as the bare one's and the 2001 checks were the bare tree's, to the check. The number stands as the bare tree's measurement; the claim about it did not. The repair is `$(HOST_FITTED_CFLAGS)`, scoped by `$(BOARD)` so the flag reaches the F405's two host objects and neither of the Feather's, and the pass now reports **2,270 checks, 0 failed** against the bare build's 2,269 - the F405's fitted arms taken for the first time. See traps 205. On the NAS `fc-build aerialkit-f405` → log with the source revision and artifacts under `/srv/nas/Portable/fc-firmware/aerialkit-f405/` |

One bug was caught by reading the linked image rather than the source: the
`RCC_PLL_P` macro encoded the PLLP field as `P / 2`, and RM0090 encodes it as
`2 * (field + 1)`. The part would have run at 84 MHz while every piece of
bookkeeping - system clock, UART divisor, tick - believed 168 MHz. The word in
the image was `0x07415408` before the fix and `0x07405408` after.

What is *not* evidence: the LED blinking, the banner appearing, anything about
the pads. Those need the board.

## M1 - the flight core on a host  (done, with the caveats below)

The core (sensors -> estimator -> control -> mixer -> outputs) builds and runs
natively, driven by synthetic inputs, with tests. `make test` is the entry
point; `docs/04-flight-core.md` has the module map and the conventions.

What exists: portable math (our own `sqrtf` and `atan2f`), a complementary
attitude estimator, a PID with derivative on measurement and a clamped
integral, RC decoding, a CRSF frame parser with its CRC, a table-driven mixer
with eight airframes - quad-X, a twin-motor elevon wing, three more quad
layouts, a Y4, a V-tail, a tricopter, and the same wing with one motor and no
rudder, the first frame whose mix states what the aircraft *cannot* do. The six
multirotor frames are each checked row for row against Betaflight's own tables,
the twin-motor wing against the saved INAV preset of the aircraft that flies it
(`projects/twin-wings/ghf435-inav/`), and the single-motor wing against its twin
- and the arming/failsafe state machine - plus one selftest that runs both on
the host and on the board at boot, a replay tool for recorded traces, and a
scripted 3-second trace in `docs/evidence/`.

Evidence, measured on the NAS (aarch64). The check counts are from the run of
2026-09-16; the numeric comparisons below them have not changed since they were taken:

| What | Result |
| --- | --- |
| Host tests | 1088 checks, 0 failed (`make test`), plus 27 client-against-firmware checks (`make proto-test`) |
| The port itself, executed | `tests/test_arch.c` maps the peripheral region at its real addresses and runs the real clock, pin, UART, tick, output, sensor-bus, ADC, USB, flash and configuration-record code against it: 189 checks, including the board file itself, see [21-port-on-the-host.md](21-port-on-the-host.md). It found the UART divisor sixteen times too large on every port and an unbounded wait in the console's sink; the DShot timers, pins and DMA burst, the SPI bus, the ADC's two sample-time registers and the saved configuration's refusals. The USB driver is the last of the register work - thirty-two checks that found a receive status word read from the wrong register, an endpoint never armed for setup packets, a banner queued and never sent, and, on 2026-09-17, the second packet of a configuration descriptor 67 bytes long: it was sent from past the end of the one-packet buffer with a length of zero, so every host would have received a descriptor it could not parse and the console - the only door this board has - would never have appeared. Two drivers needed the *device* to act on a write, and both are closed with a seam and a model: the flash controller's success path (an erase that empties a sector, a word that lands where it was addressed) against `tests/host_flash_model.c` - the first attempt at that model was withdrawn because its sectors were all one size, and the page has that story and the two ways the checks are known to fail - and, since 2026-09-16, the sensor bus, where `tests/host_i2c_model.c` runs the transaction sequences RM0090 describes against a slave with a register file: the one-byte read with its early NACK, the two-byte read through POS and BTF, a longer read with the pointer walking, a write ending on BTF, and an address nobody answers followed by one that works. On top of both, the blackbox's own region and ring are driven end to end: 2184 records filling a sector exactly, the log stopping when the next sector holds somebody else's bytes, the erase on the ground letting it carry on, and the index arithmetic crossing that sector boundary the way a tool pulling the log needs it to |
| The whole firmware, in the loop | `make test` also runs forty-two scripted sessions against a simulated board and checks what it saw - **491 checks in all, every flying one of them in five metres a second of wind** unless the session's own point is to take it away. The sessions, by the number of checks each one makes: the quadrotor on the sticks 14, the same with noisy sensors 14, the same with the receiver alive inside its own failsafe 15, the fixed wing 21 and the wing again with noisy sensors 21, the quadrotor's return 20, the return with no rangefinder 20, the return with a rangefinder that went quiet 22, the same return on a bare board 22, with a dead barometer 20, with a degraded fix 24, with a fix that lies 21, with a warmed-up gyro 21, with noisy sensors 20, a fence with a ceiling 9, the GPS gone quiet 13, the GPS gone quiet with a landing commanded 12 and without a rangefinder 9, a pack going flat under a pilot 11, a mission on the wing 7, on the quadrotor 7 and in still air 7, a mission taken away by a pack that went critical under it 4, rate mode 10, the gyro dying in flight 9, an aircraft held at 60 degrees of roll with the arm switch thrown 9, a wing thrown by hand 6, the same launch with a pack that went critical under it 2, the bench calibration 10 (the boot's own stages are one of them), the calibrations refused 5 and the same with no receiver 5, the output test meeting the arm switch 8, an aircraft armed while it was still being held 9, a mission the pilot takes back with a stick 2, a boot after a crash 6, the quadrotor holding station for five minutes 7, the wing circling for five minutes 7, a wing with no navigator that loses its link 17, the board that lies 8, a board with no outputs 5, a two-motor board 3 and a GPS module that never speaks 7 (the last three are the preflight's failure lines, executed - see [15-preflight.md](15-preflight.md)). Two of them are worth naming: the wing's return is flown in five metres a second of wind with its nose up to 86 degrees off the track it is making, and a full roll stick in rate mode asks for 400 deg/s and the airframe reaches 389. What each session asserts is in [18-software-in-the-loop.md](18-software-in-the-loop.md), and the transcript it leaves is `docs/evidence/sil-*.txt`, re-recordable with `make sil-trace` |
| The bring-up checklist, typed at a console | `make bench-check` drives the simulator's console - the prompt, the command echo, the answers - and checks them: 19 checks, of which 17 pass on a bare board and 2 are "needs a person" (the MOSI-MISO jumper and the calibration). It is the same script that runs against the F405 with `make bench-check PORT=/dev/ttyACM0`, and it writes the session out as a transcript for `docs/evidence/` - the human checklist in [05-bringup.md](05-bringup.md), made reproducible. The console itself gained the `ak> ` prompt it had been documented as having, which is what a board that is listening prints and what a script watches for |
| `ak_sqrtf` vs libm | worst relative error 1.2e-7 over 2000 values |
| `ak_atan2f` vs libm | worst absolute error 7.7e-6 rad over the whole circle |
| Closed loop against a toy plant | 15 deg commanded, 15.1 deg reached, settled |
| CRSF | 16 channels recovered from packed 11-bit fields, corrupted frames rejected, parser recovers after a truncated frame |
| DShot and servo encoding | hand-derived frames: 0x0000 disarmed, 0xFFFF at full throttle with telemetry, 0x7D0A at throttle 1000; servo centre 1500 us, limits 1000/2000 us |
| Target image | still builds for F405 and still passes `scripts/check-image.sh`: 94632 B of flash, 9.02% of the part, 78272 B of RAM (33 KB of that is the two RAM blackbox rings: 16.5 KB of fast log and 16.5 KB of retained; 8 KB of the flash is the BMI270's configuration file; the six sensor drivers and their tables are the rest of the growth; 2 KB of RAM and a quarter of a kilobyte more are the USB console's two rings, 52 B are the flash log's state, 3.7 KB are the parameter table and the configuration record growing to 96 slots and 2048 bytes - which is the fix for a table that was full at 64 and a record that was 111 bytes too small - 1.3 KB are the CRSF telemetry frames, and 3.2 KB are the two return profiles with their arithmetic, the ground-track alignment, the landing rule and the wing's turn coordination - plus the quadrotor's own arrival radius, the windowed climb rate the vertical loop flies on and the wind its position loop learns, the hold a navigator flies when its fix is gone and the quadrotor's mission answers. The last 5436 B of flash and 9376 B of RAM are the six things after that: the rangefinder (its driver, its filter, the landing rule that reads it and the descent a quadrotor with no position flies on it), the four things a quadrotor's guidance was missing (the vertical loop's integral, the heading measured through the air rather than over the ground, the nudge that earns one and the bounded turn rate that keeps it valid), and the standing pitch the wing's altitude loop learns for the same reason the quadrotor's does, and the gyro bias the aircraft measures for itself the moment it is switched on, the preflight line that says which airframe and which bias the flight is actually on, and the position in the blackbox - the record is 8 bytes longer and there are three 384-record rings, which is where the RAM goes. It is eight bytes smaller as a *release* image than one built from a dirty tree, because `git describe --dirty` puts `-dirty` into the revision string the image carries). The linker script now refuses an image that grows into either the blackbox region or the configuration sector, and the check names each vector the firmware depends on, so a handler that quietly lands on the default loop is a test failure rather than a mystery |
| Largest stack frame | 472 B, in the selftest, which holds a complete flight state |

Four bugs were found by the tests, none of them visible to a compiler: the
CORDIC `atan2` was wrong on the left half of the circle, the mixer indexed
servos by row number, which misplaced every servo of the elevon-wing table, the
CRSF parser had no way out of a frame that stopped half way - a receiver
rebooting mid-frame would have left the link dead until a power cycle - and the
DShot CRC folded the wrong twelve bits, so every frame an ESC would receive was
one it would ignore. All four are described in
[04-flight-core.md](04-flight-core.md).

Five more were found later, by other means, and they are the ones worth reading
before trusting a green build:

- **The UART divisor was sixteen times too large on every port**, so the
  console would have run at 7200 baud against a terminal at 115200, the
  receiver at 26250 instead of 420000 and the GPS at 600 instead of 9600 - a
  silent console, no frames and no fixes, on the first flash, which reads like
  three wiring problems and is one line of arithmetic. Found by running the
  port's own clock, pin, UART and tick code against a mapped register block,
  which is the only thing in this repository that had never executed it; the
  same exercise found an unbounded wait in the console's sink that would have
  stopped the firmware inside the first line of its own banner. Both are in
  [21-port-on-the-host.md](21-port-on-the-host.md).
- **Every UBX NAV-PVT offset was two bytes too high**, so the GPS would have
  decoded nonsense from a real module and never produced a usable fix. The unit
  test and the simulator had both been written to the same wrong numbers, so all
  three agreed with each other and none with a receiver - and the document that
  should have caught it claimed the cross-check had been done. Found by reading
  the offsets against the pinned upstream's own struct;
  [13-gps.md](13-gps.md) has the post-mortem.
- **The simulator's clock advanced twice as fast as the firmware's loop**, so
  the radio liked its frames and the aircraft never armed. Found by the
  simulator itself once it started asserting what it saw; the "two thirds of
  the frames are lost" story in the first draft of
  [18-software-in-the-loop.md](18-software-in-the-loop.md) was a plausible
  explanation of the wrong module.
- **The navigator could never engage.** It asked whether the pilot's link was up
  *before* the flight core had sensed it for that step, so on the one step that
  mattered it was reading the previous answer - by which time the core had
  latched the failsafe, which is not a state a navigator may take over from.
  Return-to-home was unreachable from the day it was written, and the unit tests
  could not see it because they call the navigator directly, with no flight core
  to be a step behind.
- **The altitude gain was a thousand times too high.** `rth_alt_kp` is "pitch
  per metre of altitude error" - the header, the help text and the parameter
  table all say so - and the navigator multiplied it by the error in
  millimetres. A tenth of a metre saturated the pitch command, so the loop was
  bang-bang and the wing porpoised all the way home. The test asked which way
  the pitch went and never how far.

Both of those last two came out of flying the fixed wing in the loop, which is
what starting from the airframe rather than the module buys;
[14-navigation.md](14-navigation.md) has them in full.

Still not done, and these are what M2 and the tuning milestone are for: no
sensor driver, no RC driver, nothing on an airframe, and no gain or mixer sign
that has been checked against real hardware.

## M2 - F405 bring-up  (written and host-tested; no sensor has been read)

Clock, GPIO, timers/PWM, DMA, SPI, I2C, UART, ADC, flash; IMU and baro drivers
talking to real silicon; calibration; a working attitude estimate.

The peripheral list is now complete: clock, GPIO, timers and PWM, DMA, SPI,
UART receive, flash, ADC1 for the flight pack
([19-battery.md](19-battery.md)), and I2C for the barometer
([20-i2c.md](20-i2c.md)). The sensor drivers are written and host-tested with
their register maps cross-checked against the reference implementations
([09-sensors.md](09-sensors.md)): six inertial parts behind three drivers and
**six** barometers behind three, which between them now cover every sensor the
plan names - the BMP388 was the last one missing, and adding it was worth it
for a reason that had nothing to do with the part: its arithmetic comes in two
variants whose results differ by a factor of a hundred, and the host test holds
the two against each other. That test also found an overflow in the last line
of the integer variant, which the Bosch API casts to `uint64_t` to avoid and
this driver now does too. The barometer driver did not change a line when its
bus did, which is what `ak_bus.h` was for.

What has not happened is the last clause of the milestone: nothing has talked to
real silicon. The bench board has no inertial sensor at all, so the IMU is
written for a sensor-less board to be ready for one, with the SPI loopback as
the check that the bus works in the meantime; the battery divider **is** fitted
now (the owner soldered it with the barometer, which is why the `-fitted` image
exists), though no pack has been measured against it yet; and
the I2C bus has run against modelled parts rather than a real one
([21-port-on-the-host.md](21-port-on-the-host.md)), which makes the timing
arithmetic and the read sequences the only *measured* part of that bus.

Board alignment, gyro bias and the accelerometer's own six-position calibration
are in ([10-calibration.md](10-calibration.md)): three rotation parameters and a
`calibrate` command that refuses to run while armed or while the aircraft is
moving, refuses samples that show it moving, refuses to produce a correction
from six faces that do not add up to gravity, and stores whatever it did measure
where `save` can keep it. All three are arithmetic-checked on the host, and the
accelerometer one is driven end to end by the simulator holding the aircraft on
each face - which is how the `%g` that the console formatter does not have got
found.

The first half of it has a checklist: [05-bringup.md](05-bringup.md) - flash the
skeleton, check the three board assumptions one symptom at a time, measure the
clock rather than believing the banner, and write the result down. Fault
capture is already in place for when it resets instead of running.

Ready and waiting: the console ([06-console.md](06-console.md)) with its
range-checked parameter table, saved configuration in the last flash sector
behind a checksum, and a flash driver that checks every error flag instead of
assuming. None of it has touched a board - it is host-tested and waits for the
first flash.

## M3 - RC in, actuators out  (written and host-tested; nothing has moved a pin)

CRSF/ELRS over UART (the wing uses CRSF on UART2) and SBUS in; DShot300/600 out
to ESCs and 50-400 Hz servo PWM for elevons; arming, disarm, failsafe.

The actuator half is written: DShot on TIM3 with a DMA burst ([07-outputs.md](
07-outputs.md)) and servo PWM on the timer the board names - TIM2 on the WeAct
board, TIM4 on the Feather - with the pin map, the timing arithmetic and the
register encodings host-tested. The output path is deliberately not
verified: no waveform has been seen, and the encoder tests cannot see one. The
receiver half is written and host-tested end to end ([08-receiver.md](
08-receiver.md)): CRSF in on USART1 through an interrupt and a ring buffer,
decoded into sticks, with counters that separate "no receiver" from "a receiver
saying nonsense" - and now SBUS as well, at its own line settings, including
the receiver's own failsafe flag, which keeps a receiver that is still sending
frames while it has lost its transmitter from looking like a live link. The
quadrotor in the loop flies on SBUS frames for the whole session, so the path
from a console command to a turning motor is covered in both protocols. What is
left in this milestone is the hardware: a scope on the output pins, a receiver
on PA10, and an inverter in front of it if that receiver speaks SBUS.

**And the other direction of the same wire is written**: CRSF telemetry out, so
the handset shows the pack the aircraft is flying, the attitude it has, the
position the module is reporting and the mode it is in - the frames a
Crossfire/ELRS receiver forwards, built to the layout Betaflight writes and
verified against it ([08-receiver.md](08-receiver.md)). The percentage in the
battery frame is an estimate from the configured thresholds and the voltage
rides next to it; the flight mode is a wire string (`ANGLE`, `RTH`, `!FS!`)
derived from the flight core's state so the two cannot disagree. The encoder is
host-tested field by field and against the CRC-8/DVB-S2 catalogue value, every
frame is fed back through the firmware's own receive parser, and the simulator
compares what a handset would have read with the aircraft that was flying -
including `ANGLE` on the way out and `RTH` after the link went. PA9 has been
labelled "wired for telemetry later" since the board file was written; no
transmitter has ever been wired to it, so a real handset is still a bench step.

**And the arming path says why it will not arm.** Arming was three conditions -
the switch, the throttle down, the estimate converged - and the fourth thing
both references check, that the aircraft is the right way up, was missing: a
quadrotor held at 40 degrees of roll armed and then flew the correction with the
motors on their stops. The gates are one function now, `ak_flight_arm_check()`,
with `arm_max_tilt_deg` (25 degrees by default, the reference default, 180 for
the whole sky), and the same function answers `status`, the preflight report and
the sentence printed the moment a pilot throws a switch and nothing happens -
which used to be silence, and a bench hour. The tilt gate is also the check that
catches a board-mount alignment parameter that is wrong: a board mounted at 40
degrees reads 40 degrees of roll while sitting still. Flown in
`aerialkit-fw-sim 20 tilt` ([18-software-in-the-loop.md](
18-software-in-the-loop.md)); the gates and the citations are in
[04-flight-core.md](04-flight-core.md).

## M4 - fly both airframes  (both fly in the simulator; no airframe has flown)

Rate and attitude control; mixers for quad-X and for the twin-motor elevon wing
with differential thrust; stabilized modes; blackbox logging.

Control, mixing and logging exist: rate and angle modes, both mixer tables, the
authority limit that keeps a saturated mix flying the aircraft the pilot asked
for rather than a flattened version of it ([04-flight-core.md](
04-flight-core.md)), and three blackboxes ([11-blackbox.md](11-blackbox.md)): a
fast ring at 250 Hz for 1.5 seconds of bench detail, a long one at 25 Hz for
fifteen seconds in memory that survives a reset, and - since this milestone's
last gap - a log written into five 128 KB sectors of the part's own flash at
5 Hz, which is the one that survives losing the battery and the only one that
would still be there after a crash in a field. Both of the reset-surviving ones
announce themselves at boot: how many records, and how far into the previous
run they reach.

The flash log needed the flash *write* path to be believable first, and that
landed the same day: the driver's success path runs against a modelled
controller ([21-port-on-the-host.md](21-port-on-the-host.md)), so an erase that
empties a sector and a word that lands where it was addressed stopped being the
one thing in that file nobody could check. All three logs also come out over
the config protocol, so a tool pulls the one in flash the same way it pulls the
fast ring - `akproto.py --source flash crash.csv` -
([16-protocol.md](16-protocol.md)). What is still missing from M4 is
everything that needs an aircraft: the gains are untuned, no motor has turned,
and nothing here has written to a real flash sector. The order those get settled
in - the ground tests, the first hover and the first launch, the tuning ladder
and what the log is read for - is [23-first-flight.md](23-first-flight.md), which
is the part of this milestone that cannot be rehearsal.

**And the wing can be thrown now.** A fixed wing's first flight starts with two
seconds in which nobody is holding the sticks, so `launch_channel` names a
switch and `launch_throttle`, `launch_climb_deg` and `launch_timeout_s` are what
it flies while the pilot's hands are busy - a climb attitude and a throttle,
held until a stick, the clock or the arm switch takes it back
([24-launch.md](24-launch.md)). The numbers are INAV's, because that is the
implementation flying this airframe today. What is deliberately missing is
INAV's other half - it watches the accelerometer and the GPS for the *throw* so
the motors stay at idle until the aircraft is moving - and the reason is the
instrument: this project's wing has no throw in it, so a detection threshold
would be a number nobody could measure. The switch is the detection until there
is a plant, an airframe, or a log of a real throw to measure one against. It
caught its own bug on its first run in the loop: the switch was read as a level,
so a launch that ended on its timeout was followed by another one every five
seconds for as long as the switch was held.

## M5 - fixed-wing navigation  (in progress)

GPS, return-to-home, then waypoint follow.

**Three inertial parts and one barometer are driven**, from the milestone's own
list: the ICM-42688-P and the ICM-42605 (the same part under another who-am-i,
so a table entry), the MPU-6000 (a different register map entirely, and the
sensor most boards of the last decade have on them), and the DPS310/SPL06-003
barometer - [09-sensors.md](09-sensors.md). Adding the 42605 found a bug
immediately: the driver's init insisted on the 42688-P's who-am-i, so its own
second entry failed to configure.

**The sensors are not just inertial.** A DPS310 (and the SPL06-003 that is the
same part) now sits behind the same bus seam the IMU uses, with the datasheet's
compensation arithmetic pinned against a separate computation and the
pressure-to-height curve checked against the standard atmosphere from -100 m to
2 km - [09-sensors.md](09-sensors.md). The simulator answers with the pressure
at whatever height its airframe is flying at, and the loop reads that height
back to a tenth of a metre. The bench board's barometer is fitted now (the
owner soldered it, which is what the `-fitted` image is for) though nothing has
read a real part yet - and the navigator flies on it in the loop: a complementary
filter fuses the barometer's fast changes with the GPS's absolute reference, and
the test measures what that buys (0.43 m of rms error against the GPS's 2.06 and
the barometer's 5.47 on the same synthetic flight).

**The GPS is parsed**: UBX NAV-PVT at 5 Hz over USART3, with the fix
kept in the protocol's own integer units and validity requiring a fresh, ok,
3D fix - see [13-gps.md](13-gps.md). The framing is checked against a captured
frame, and the decode is pinned by tests and still needs a real receiver.

**And it navigates**: when the pilot's link is gone and `rth_enable` is set, the
wing holds cruise, steers home on a course error that wraps the short way, holds
the altitude it had, and circles on arrival - [14-navigation.md](
14-navigation.md). It defaults to off, is untuned, and has never flown - but it
does now fly *in the loop*: the whole firmware against a simulated wing, out to
118 m, link lost, home again inside 12 m, altitude held to a metre for half a
minute, and the console's own `return: engaged` line as the proof
([18-software-in-the-loop.md](18-software-in-the-loop.md)). That flight found
two bugs in this code, and both are recorded in [14-navigation.md](
14-navigation.md): the navigator could never engage at all, and its altitude
gain was a thousand times too high.

**And it flies a mission.** A list of up to four waypoints lives in the
parameter table (`wp0_lat` .. `wp3_lon`, `wp_count`), a console command or a
protocol client fills it, and `mission start` hands the aircraft to the
navigator *with the link still up* - which needed a fifth flight state, because
"a navigator flying because nobody else can" and "a navigator flying because
somebody asked" are different things and only one of them is a failsafe. Moving
a stick gives the aircraft back. The same guidance flies the return with a
target that moves when it is reached, and past the end of the list the wing
circles where it is - [14-navigation.md](14-navigation.md) has the shape, and
[18-software-in-the-loop.md](18-software-in-the-loop.md) the flight.

**And a return has a box around it: a floor, a ring and a lid.**
`rth_min_alt_m` is the lowest altitude the navigator will hold while it is
flying, which is what stops a return descending into the hill beside it.
`fence_enable` with `fence_radius_m` and `fence_ceiling_m` is a geofence: fly
outside the ring or over the lid and the aircraft comes home, with its link up,
which makes it the one automatic behaviour here that takes an aircraft off a
pilot who is still flying it - so it is off until somebody turns it on, it says
so on the console, and it hands back the moment the aircraft is inside again.
All of it is in [14-navigation.md](14-navigation.md), and all of it was flown in
the loop - which is how the note about fences smaller than a turn radius got
written, and how the ceiling's own version of that trap was found: a navigator
holding the altitude its own trigger fires at swapped control with the pilot
four times in twenty seconds, and it holds ten metres under the lid now.

**And a mission has a switch.** `mission_channel` names a receiver channel; high
starts the mission and low ends it, and the switch wins over the console while
it is set, because a mode that can only be selected from a laptop is a mode
nobody selects in the air. The mission in the loop is started and stopped that
way, and the check after the switch goes low is that the flight core says the
pilot has the aircraft.

**And the quadrotor has its own return now** (2026-09-16): climb, translate,
settle, in its own profile rather than a branch in the wing's
([14-navigation.md](14-navigation.md)). Three things it needed that the wing's
return does not: a tilt commanded from a *velocity* error (a tilt is an
acceleration for a quad, and the GPS velocity is what damps the approach), a
climb-rate loop on the throttle, and a heading that means something - which came
from aligning the estimator's yaw to the GPS ground track
(`ak_estimator_aid_heading`), since nothing on this aircraft measures north. The
profile holds, rather than translating, until that alignment exists. It descends
to `quad_hover_m` above the ground home was captured on, waits two seconds
there, and then lands itself - motors stopped, disarmed - on a landing rule
that reads the *stall* of the descent rather than an altitude. (That rule has a
second form now, for an aircraft carrying a rangefinder: the part has to say
the ground is close **and** stop changing, and a reading that disagrees with
the height estimate refuses the landing rather than stopping the motors in the
air - [09-sensors.md](09-sensors.md).) The return has an arrival radius of its own
(`quad_arrive_m`): a wing's arrival is a circle it can hold, a quadrotor's is a
vertical descent, and starting that descent at a wing's sixty metres put the
aircraft on the ground thirty-eight metres downwind. Verified by flying a point
mass through the same code at 50 Hz, and in the loop: the simulator's quad has
tilt-to-motion, drag and a quantised barometer, and the return is one of the six
sessions - out on a stick, the link lost, home, down over the pad and disarmed,
in five metres a second of wind. Three bugs came out of those flights and none
was visible in a single step: a tilt driven from position alone flew at home
and past it forever; an altitude error straight into throttle oscillated forty
metres; and the *climb rate*, taken as the difference between two passes of a
1 kHz loop on a 32 Hz barometer, read as zero while the aircraft descended at
ten metres a second, so the loop commanded more descent. The rate is measured
over a window now - the arithmetic is in [14-navigation.md](14-navigation.md).

**And a return that loses its fix keeps flying.** The pilot's link is already
gone when the module stops answering, so the navigator is the only thing flying
the aircraft and the one measurement it cannot navigate on has just
disappeared. Until this was measured, that combination stopped the motors two
seconds later and the aircraft fell out of twenty metres - the flight core was
doing the correct thing for a lost link with nobody flying, which is a rule
that does not belong in the air. The navigator holds now, on the two
measurements that still work: level attitude and the altitude it was told to
hold, with the mode still RTH and the console counting the steps. Measured in
the loop: motors never below a collective of 0.53 against a hover of 0.55, the
altitude held at 20.2 m for the rest of the flight, and the aircraft drifting
with the wind because there is nothing left to hold a position against - which
is the honest shape of it. Bringing a quadrotor down without a position
reference needed something to measure the ground, and that is in now: with a
rangefinder fitted and `quad_hold_land_s` set, the hold becomes a descent onto
whatever is below the aircraft, flown on the part for the last two metres and
stopped by the same rule that stops any other landing. The parameter is off by
default, the part is required (a descent with nothing measuring the ground
would be flown on the instrument that failed), and it is measured in a session
of its own - `aerialkit-fw-sim 130 gpslostland` - with the same flight run
without the part as the control. Both are in
[14-navigation.md](14-navigation.md) and [09-sensors.md](09-sensors.md).

**And the same waypoint list flies on the quadrotor.** The mission is the same
code with the other profile, and flying it is what found the three things that
are the airframe's answers rather than the list's: how close counts as arriving
(a wing's radius is the circle it can hold, a quadrotor's is a point - with the
wing's sixty metres the quadrotor "arrived" 45 m out and cut every corner, and
with its own it arrives 12 m and 0 m from the two waypoints), whether arriving
means coming down (a waypoint is not ground to land on, and the quadrotor's
descent aims at the altitude home was captured at - the descent gate is the
return's now), and counting each waypoint once (a quadrotor holding station on
the arrival radius counted two waypoints as three). The eighth in-the-loop
session flies it: `aerialkit-fw-sim 130 mission quad`.

**Three more things the quadrotor could not do, found by asking a machine to
fly it rather than a person to believe it.** The vertical loop was proportional
only, so a descent settled at *0.27 m/s where the profile asked for 0.6* - the
loop settles where the plant needs the throttle the error happens to supply -
and it has an integral now, bounded by what the rate being asked for is worth
(a fixed bound is a bound the loop sits on: the first version carried the
aircraft through the hover height and into the ground). The heading estimate
aligned the yaw to the GPS *ground* course, which in wind is not the way the
aircraft is pointing and at a hover is the wind itself: a mission ran with the
estimate 50 to 136 degrees from the nose before it was corrected by the wind
the navigator had already learned. And a mission started from a hover in *still*
air could not begin at all - no motion means no track, no track means no
heading, and no heading meant no translation - so the navigator now shoves
itself straight forward for up to eight seconds to earn one, and caps its turn
rate so the heading it earns stays valid. Measured: `aerialkit-fw-sim 150
mission quad calm` reaches both waypoints (4 m and 0 m) and holds station,
against hovering 221 m from the first one for the whole session before.
[14-navigation.md](14-navigation.md) has the numbers and the wrong turns.

**And the wing's height had the same shape of bug.** A wing sinks, so holding an
altitude takes a standing nose-up, and a proportional law can only take it from
the error: the return held 14 m where it was told to hold 15, and a fence that
asked for a 30 m floor settled at 26. The pitch is learned now (`rth_alt_ki`,
bounded and conditioned like the quadrotor's), and the same two flights hold 14
and end at 14, and reach 28 of the 30. The A/B against a plant with a sink is
5.00 m of droop without it and 0.12 m with.

**And the gyro's own offset is measured by the aircraft, not only by a bench.**
Every MEMS gyro reports a small rate when it is still, the control loop cannot
tell it from a slow rotation, and it grows an attitude error for as long as the
flight lasts - in yaw there is no accelerometer to catch it and the GPS-track
aid only works while the aircraft is moving. `calibrate` measures it on the
bench and stores it in the parameters; the firmware now measures it again by
itself, once, at power-up while the aircraft is disarmed and still, which is the
moment somebody has just put it down. Measured: with five degrees a second of
bias in the part that the stored calibration does not know about, the heading
drifts **100 degrees in twenty seconds** of a stationary aircraft - and
**zero** with the measurement. In the loop, the same five degrees walks the
heading **zero degrees** while the return sits parked at the end of a flight,
against 109 degrees with the measurement disabled. What the "still" test
cannot be is the *size* of the rate, because the offset being measured is one;
it is steadiness - of the gyro and of the accelerometer, which a rolling
aircraft moves. [10-calibration.md](10-calibration.md) has the three wrong
turns that took, including an arm-time version that wrote a wing's take-off
roll into its own bias.

**And the receiver can be calibrated**: `calibrate rc` measures where the sticks
actually sit and writes the centre into the parameter table, because a receiver
a few counts off centre is a permanent stick input that looks exactly like an
accelerometer problem on the bench.

## M6 - ESP32 target  (port built and running under emulation)

The same core on an ESP32 part through ESP-IDF, with Wi-Fi config and telemetry
and logging, LAN/tailnet only. Survey what already exists (Espressif's ESP-Drone
is a starting point) before writing any of it.

**The port exists and runs.** `ports/esp32/` is an ESP-IDF project whose
`app_main()` calls AerialKit's `main()`; `src/arch/esp32/` and
`src/boards/ESP32DEV/` satisfy the board contract for the chip, and `src/core/`
arrives unchanged. Both Espressif toolchains are installed here (aarch64-hosted,
so this Pi builds them) and every portable source file compiles for xtensa and
riscv32 with no failures and no warnings - `make port`.

And it runs: `scripts/esp32-qemu.sh` builds the image and boots it under the
QEMU that ships with IDF, where the firmware prints its banner, passes its own
sixteen-check selftest and its preflight check, and runs the flight loop at
about 750 Hz under emulation. The captured run is
`docs/evidence/esp32-qemu-boot.txt` and the detail is
[17-esp32-port.md](17-esp32-port.md).

The port found two bugs that only exist at a port boundary - a CMake glob that
did not re-evaluate, and `UART_NUM_0` being zero where the code used zero as
"nothing attached" - both of which would have been silent on hardware.

And the config protocol works there, driven by the same client that talks to the
F405: `scripts/esp32-proto.sh` waits for the boot banner, then reads parameters,
sets one, and saves it - into NVS rather than a flash sector, which is the board
contract doing its job. Running that found a third bug in the flight core: a
disarmed aircraft with no sensors reported "failsafe" from its first loop
iteration, which reads as an aircraft in trouble rather than one that has never
flown.

**And the outputs run now, which is the one part of this port that had never
run anywhere.** QEMU's ESP32 has no RMT, so no frame completes there and the
driver's honest report - "no frame has completed" - is the same sentence
whether the frames are right, wrong or truncated. So the real
`src/arch/esp32/output.c` is compiled for the host against a stand-in for the
two IDF drivers it calls (`tests/idf-stub`, `tests/host_esp32_output_model.c`)
and driven by `tests/test_arch_esp32_output.c`: channels, tick, memory block,
queue depth, the servo duty arithmetic, the busy-channel skip, and - the one
that matters - the symbols handed to the RMT, decoded back into the frame an
ESC would see. **It found that every frame was a quarter of a frame**:
`rmt_transmit()` takes bytes, not symbols, so seventeen symbols went out as
seventeen bytes. That is a DShot frame cut off after four bits, on the target
whose whole reason for existing is that it can fly the same aircraft as the
F405. [07-outputs.md](07-outputs.md) has the arithmetic and the fix.

**And its two sensor buses run too.** `spi.c` and `i2c.c` were in the same
position - a devkit carries no sensor, QEMU models neither peripheral, and a
bus with nothing on it only proves the failure path is reached - so they are
compiled against a modelled SPI and I2C with a register file behind each
(`tests/idf-stub/`, `tests/host_esp32_bus_model.c`), and the core's own
ICM-42688-P and DPS310 drivers run on top of them. What is checked is what went
out on the wire: the address byte with its read flag and the padding byte after
it, the register pointer a write leaves, a 3000-byte burst as three chunks with
the select held down rather than three conversations, a register write landing
where it was addressed, and a part that stops answering costing a failed read
and a driver error rather than a hang - 35 checks, and the six IMU and six
barometer drivers are now on a bus that has been *run* on both targets
([21-port-on-the-host.md](21-port-on-the-host.md)).

**And the receiver's and GPS's UARTs, which is the last of that target a host
can run.** `uart.c` is compiled against a modelled IDF driver
(`tests/host_esp32_uart_model.{c,h}`): a ring the test fills, the driver's
event queue, a transmit buffer, and the failure switches a bench cannot
arrange. The three decisions the file makes are the three that are checked -
the read path never waits, the driver's events are drained on that same path
(a full event queue is how an overflow goes missing), and a framing error is
said once and then only counted - along with the protocol switch, which is this
chip's advantage over the F405: SBUS's inversion happens in the GPIO matrix, so
the checks watch the line settings change and the receive ring be flushed with
them. 28 checks. What is left on this target with no host coverage at all is
the radio itself.

**And the last part of the aircraft the second target had no representation of
is there: the flight pack.** The chip's converter is a genuinely different
animal from the F405's - the original ESP32 calibrates by line fitting from two
eFuse points, where the newer parts carry a curve - so the arch layer hands the
core *volts* where the F405 hands it counts, and a chip whose eFuse has no
calibration falls back to the datasheet's numbers and says so. The divider
ratio, the cell count and the thresholds are the same core file on both chips.
Nothing is soldered to it on a bare devkit and the console says "none fitted"
rather than a voltage from a floating pin, which is what the emulated check now
asserts ([19-battery.md](19-battery.md)).

**It serves config and telemetry over the network too.** The port brings up a
netif and a listening socket on 5555, and the core answers its own protocol over
that socket and streams telemetry to a client that subscribes. Thirty-two checks
under QEMU, nine of them on the network, with the client that talks to the F405
on the other end of a real TCP connection -
`docs/evidence/esp32-network.txt`. `scripts/esp32-net-watch.sh` is the human
view: live telemetry from the emulated board on this machine, recorded in
`docs/evidence/esp32-net-watch.txt`. That found a fourth bug: a subscription
that outlived the client that asked for it, so a reconnecting client was sent a
stream it had never requested, before its first request was answered.

**And the radio path is written.** Which medium carries that socket is a
compile-time choice (`main/Kconfig.projbuild`), because a chip does not have
"the network": QEMU has an emulated Ethernet MAC and no radio, and a devkit has
the radio and no PHY. The Wi-Fi half does what its saved parameters say: it
joins the network `wifi_ssid` names (WPA2 or better) or, with nothing to join,
carries its own access point - which is **never open**, since the config
protocol has no authentication of its own, and which is the one mode that gives
a board an address somebody can guess (`192.168.4.1`, IDF's default for that
netif) rather than one it has to be told. A station re-associates by itself
after the router comes back - IDF does not, and a station that is up and
associated with nothing would report "no address" for the rest of the flight.
It compiles and links (952,176 B against the emulated-Ethernet image's
532,752 B, measured 2026-09-18; the radio's share of a 1.25 MB app partition
is about three quarters) and **has never been run**: QEMU has
no radio, so the automated checks still exercise the Ethernet medium and the
image above is the whole of the evidence for the other one - which is why it is
now *published* rather than left in a build directory:
`dist/aerialkit-esp32/aerialkit-esp32-wifi.bin` is the one a devkit takes, and
`aerialkit-esp32.bin` beside it is the emulator's. The two names are the whole
of the difference and each image is useless in the other's place, so
`scripts/esp32-proto.sh` reads the medium out of both map files on every build
(see [17-esp32-port.md](17-esp32-port.md), "First, which image"). **And until that
date it did not compile at all** - `net.c` used the core's `ak_strlen()` in the
Wi-Fi half without including the header that declares it, and nothing built
that half; `scripts/esp32-proto.sh` builds both now, which is what would have
caught it days earlier.

**And the outputs are written, in the shape this chip forces.** An ESP32 has no
timer with a DMA burst, so the same frame leaves through the **RMT**: one
channel per motor, a level-and-duration list straight out of the core's
`ak_dshot_edges()`, at a 100 ns tick - the APB at 80 MHz divided by a whole
number. DShot600's 1667 ns bit becomes 17 ticks, 0.2% fast, which an ESC has
margin for; a *wrong* bit is what none of them has margin for, so the encoder
lives in the core as a function with two host tests - decode its durations back
into the frame an ESC would see, and compare it bit for bit with the timer
encoder the F405 uses. The servos are LEDC channels at 50 Hz and 16-bit duty,
from the same `ak_servo_pulse_us()` the F405 uses. What the emulator can say is
that it compiles, links and boots and that the protocol still works; what it
cannot say anything about is the waveform, because QEMU's ESP32 model has no
RMT - so no frame completes there, and the board reports `output ready` 0 and
"configured, but no frame has completed" rather than pretending. Two things
that cost real time are written up in [07-outputs.md](07-outputs.md):
`rmt_transmit()` waits forever unless it is told not to (and inside a flight
loop that is a dead aircraft, not a slow one), and IDF logs an error on every
refused frame, which a thousand frames a second turns into an unreadable
console.

**And the receiver, the GPS and the sensor buses are written too.** The receiver
is UART2 (CRSF 420000 8N1, SBUS 100000 8E2) and the GPS is UART1 at 9600, both
on IDF's driver behind the same board contract the F405 uses: the read path
never waits, the dropped count comes from the driver's own event queue, and a
framing error is said once rather than once a frame. SBUS needs no transistor
here - this chip inverts the receive line in its GPIO matrix, where the F405's
board file has to ask for one - and the check asserts that `rc` does *not* print
the F405's warning. The sensor buses are the same `ak_bus_t` the F405 hands its
drivers, over IDF's SPI and I2C masters, so the same six IMU and six barometer
drivers run on either chip: the IMU's bus is declared, the barometer's is
*probed* at boot, and the three possible answers are three different lines on
the console. `esp32-proto.sh` now types at the console as well as talking the
protocol, so this is verified there: `rc`, `gps` and `preflight` answer, the
protocol parameter moves the line settings, the barometer probe reports, and
the SPI loopback times out honestly instead of hanging. **No device has ever
been attached to any of those pins.**

What it does not have: a board, and anything wired to one. And the port has a
reachability problem the UART never did - anyone who can reach it can set
parameters, and the protocol has no authentication of its own, which is why the
Wi-Fi path refuses an open network and why the credentials live in the build
configuration rather than in the parameter table the port itself serves.
Sensors, a receiver and a GPS need a board and a decision about which one; the
outputs and the buses are waiting only for that board, and for a scope on one of
its pins.

The decision the port forced is recorded as #6 in
[00-decisions.md](00-decisions.md): ESP-IDF for the platform, AerialKit for the
aircraft. [12-esp32.md](12-esp32.md) has the reasoning and the list of what the
port still needs - starting with a board, since no ESP32 hardware has been
inventoried and the pin map follows from which one it is.

**The survey the milestone asks for is done**:
[22-esp32-survey.md](22-esp32-survey.md), written 2026-09-16, reads Betaflight's
ESP32 platform (in this workspace's pinned checkout), ArduPilot's ESP32 HAL,
Espressif's ESP-Drone and a project-level search, and records what each is good
for and what it leaves. Three things it settles: ESP-IDF stays (three
independent ports agree); an **access point** is the default link and joining a
network is the configured case, so Wi-Fi credentials have to stop being
compile-time Kconfig; and the ESP32 needs a partition table of its own, the way
this board now has a flash layout, so that parameters and a blackbox have
somewhere to live that a reflash does not overwrite. It also records the part
that is known to go wrong in ArduPilot's port - parameter loading over the
link, "slow and not reliable" - which is a warning about where not to put a
second configuration path.

Feasibility checked 2026-09-14, before anything was downloaded: Espressif
publishes **aarch64-hosted** xtensa and riscv32 toolchains, so this Pi can host
the port rather than the laptop being the only machine that builds it. Both
URLs answered a range request (HTTP 206) from GitHub:

```text
crosstool-NG/releases/download/esp-14.2.0_20241119/
  xtensa-esp-elf-14.2.0_20241119-aarch64-linux-gnu.tar.xz
  riscv32-esp-elf-14.2.0_20241119-aarch64-linux-gnu.tar.xz
```

**Both of the decisions this paragraph used to leave open are now taken, and
not by guessing.** *Which part first* was answered by the owner's bench
inventory - a WROOM, a C3 and an S2/S3, with their pins soldered - so the
answer is the **family**: one board file per chip, each declaring what its
silicon has (the C3's two RMT channels cannot drive a quadrotor, the S2 and C3
have one UART to spare), and the image a devkit takes is built and published
beside the emulator's. *How much ESP-IDF* is decision 6 in
[00-decisions.md](00-decisions.md): the platform is IDF's whole, the aircraft is
AerialKit's, and the port boundary is the same `ak_board.h` the STM32 port uses
- which `make port` re-checks by compiling the portable core for xtensa and
riscv32 on every build.

## M7 - configurator  (done, on the host)

Parameter store, config protocol, host tooling - a client a person can use -
plus a mixer-parity regression against INAV for the same airframe. **The "where
does it live" question was answered by building it**: a window that runs
wherever Python and Tk do rather than a page on the NAS
([27-configurator.md](27-configurator.md)), which is the decision the paragraph
further down records as taken.

The parameter store is done, and it was the part that mattered: one table that
the console, the saved configuration and the configurator all use, so there is
one definition of a parameter rather than three.

**The wire protocol exists and both ends are tested against each other**
([16-protocol.md](16-protocol.md)): hello, parameter get/set/save and a status
snapshot, framed with a CRC and sharing the console's UART by sync byte. The
host client is `tools/akproto.py`, and `make proto-test` drives it against the
firmware's own protocol code over a pipe - a protocol whose two halves are tested
separately is a protocol that fails in the middle.

**There is a live stream now, and a host tool that reads it**: a client can
subscribe to telemetry at a rate and get frames pushed at it, which is what a
configurator needs and what the blackbox could never be. `akproto.py --host
10.0.2.15:5555 telemetry 10` prints it, and it is the same client over a socket
as over a UART ([16-protocol.md](16-protocol.md)). The stream is verified
against the ESP32 under QEMU, which is the only target with a network so far.

**And the mixer-parity regression is in**: our quad-X table is compared,
coefficient by coefficient and then through the arithmetic, against
Betaflight's `mixerQuadX` and INAV's target defaults, both transcribed with
their revisions ([04-flight-core.md](04-flight-core.md)). It found the yaw
column inverted against both of them - either convention flies, but only one is
the one everybody's props and wiring assume.

**The configurator has a face, and it is a window now**:
[27-configurator.md](27-configurator.md) - `tools/akconfig.py`, which lists the
table by name, sets a value through the console's own range check and shows the
board's words when it refuses, reverts what a session changed, saves, and draws
the aircraft's state, attitude, fix, outputs and log counts beside it - and
**pulls any of the three logs and draws it**, which is the one thing this
milestone's own words asked for and the window did not do until 2026-09-17: the
records come back over the same protocol the command-line tool reads them with,
and roll, pitch and yaw go on one scale against time. It runs
wherever Python and Tk are (the laptop, or the Pi beside the aircraft) and it
needs nothing installed: the serial port is `termios`. Asking for a stream over
a *console* link gets 0 back rather than a rate, because the console is the wire
a person types at and only a network link pushes - an answer the protocol had
been promising and not keeping, which the configurator is what found.

**And the window itself is checked, which this page's own doc said could not be
done.** `tools/akconfig_window_check.py` builds the real window on a Tk root
with no screen (under `xvfb-run`; it skips with a line where Tk is absent) and
drives it - table, filter, selection, set, refuse, revert, save, the panes - and
`tools/akconfig_net_check.py` does the same over the ESP32's own socket under
QEMU, which is the only place a *stream* exists to fill the live pane from. The
first run of the first of those found that **opening the window had never
worked** (`Window.connect()` called a name that is out of scope on the link's
thread), and three more wiring bugs behind it - [27-configurator.md](
27-configurator.md) lists them, and the lesson is the one this project keeps
relearning: the layer no check reaches is the layer that is broken.

**And it talks to boards that are not ours now**, which is the owner's
"compatible with all firmwares" asked back on 2026-09-18. There is no single
protocol to be compatible with - the five firmwares speak three - and the
window now asks which one is on the other end: its own protocol first, then
**MSP**, which is what Betaflight and INAV answer. A Betaflight board is named,
shown (state, attitude, pack, fix, motors, from MSP's own frames) and *never
written to*, with the parameter pane saying why it is empty; a board that
answers neither is reported as unknown, because ArduPilot and PX4 are MAVLink
and that is a third protocol not yet written. `tools/msp.py` is the client,
`tools/msp_fake_board.py` is a Betaflight stand-in built from Betaflight's own
constants and layouts, and `tools/msp_check.py` (25 checks) plus
`tools/akconfig_msp_check.py` (12, the real window against the stand-in) hold
the two halves - [27-configurator.md](27-configurator.md) says what they can and
cannot prove.

**And it is still not on the file server.** The NAS
console at `:8080` briefly grew a Flight page (`#flight`) that read this
firmware's protocol over the network - [evidence/console-flight.txt](
evidence/console-flight.txt) is the captured run, the deployed console reading
55 parameters from the emulated ESP32 and writing one - and **it was taken out
again at the owner's direction**: flight control does not belong on the
storage box's dashboard. The endpoints are gone (`/api/flight` answers 404
where `/api/live` answers 200, which is how that was checked) and the page is
not coming back.

What is left is where it should have been: `tools/akproto.py` on a laptop, and
**the Pi 3 companion**, which sits next to the aircraft and now draws the same
numbers on its own panel. `companion/fc_link.py` imports the firmware's client
rather than growing a second implementation of the framing, the dashboard's
flight-controller panel shows state, attitude, GPS, motor outputs and the
record count of each of the three logs - `not on this board` for one the
aircraft does not have, which is a different answer from zero - and
`tools/fc_link_check.py` drives the whole path against the firmware's own
protocol code over a pipe: 12 checks, and no aircraft anywhere near it.

**And a parameter list now *says* how it survived an upgrade.** The table was
always name-keyed, so a parameter this build does not have was skipped and one
it has just gained kept its compiled-in value - and both happened silently,
which is the wrong half to leave out when the next thing a person does is flash
new firmware over a saved configuration. A load now reports what it did: how
much of this build's table the record carried, what the record carried that is
gone, and what this build has that the record never saw - with the *name* of
each, because a count only says something happened. It prints at boot (the boot
loads the record), after the console's `load`, and in `status` for asking
later; a value that is out of range for a name that still exists is still a
failed load rather than a skipped line, because a configuration that
half-applies is worse than one that refuses. Verified in
`tests/test_params_cli.c` by saving from a three-parameter table and loading
into a table with one of them removed and one added, and at the console by a
record carrying a name this build has never had.

## A third target: the wing's own board (built, never flashed)

The milestones above are written for the two MCUs the goal names - STM32F405 and
ESP32 - and the wing's actual flight controller is neither: a JHEMCU GHF435 AIO
V2, which is an **AT32F435** (Artery). The plan said since the bench inventory
that AerialKit support for it was a later target. What happened instead is that
all of it was written: the survey this repository asks for before a target's
first line, the port, the board file, the linker script and the workspace
target - because the wing is the one airframe whose hardware exists, and the
WeAct F405 on the bench has no inertial sensor and cannot fly anything. The
survey,
[25-at32-survey.md](25-at32-survey.md), has the chip (8 MHz HEXT to 288 MHz
through a PLL that is not the F4's), 2 KB flash sectors rather than 16 KB, the
board's own pin map and the parts on it, what is different about its
peripherals, the order a port would be written in, and what can and cannot be
checked before a board is flashed. `src/arch/at32f435/` now holds the whole of
that order - the register map and the clock, the pins, the tick,
the console's port, the flash the configuration record lives in, the SPI bus the
gyro sits on, the timing of the I2C bus the barometer sits on, the
interrupt-driven receive the receiver and the GPS need, the ADC the pack is
measured on, and the outputs that turn motors and servos - with two hundred and one
checks running the real drivers against a mapped register block and a
modelled flash controller, the same way the F405 port was made believable before its board existed
([21-port-on-the-host.md](21-port-on-the-host.md)).

**And the port has become a target.** `src/boards/AERIALKIT_GHF435/` holds the
board file, `linker/at32f435rg.ld` the memory map it links against, and
`targets/aerialkit-ghf435/` in the workspace builds it: the image, the host
tests, the portability check, the protocol check, and the structural check on
what came out. `scripts/fw build aerialkit-ghf435` publishes it to
`dist/aerialkit-ghf435/`, and it reports **105,224 B of flash and 78,780 B of
RAM** - the same flight core as the F405's image (which is 104,008 B and 78,432 B
at the same revision), on a part whose timers run at 288 MHz. The vector table
had to stop being shared to make that possible: it lived in the Cortex-M4
startup file, which is code both parts compile, and it was the F405's table.
Each arch provides a `vectors.h` now, the shared reset
path includes it, and the image check reads the table's length and each named
entry out of the part's own facts file - this part's table is 74 words against
the F405's 56, because its motor DMA is at interrupts 56 and 57 rather than at
15. Two of the AT32's checks exist because of what writing the board file
found: this part gives each *pin* its own function number, so the receiver's
two pins take two numbers (PB0 is function 6, PA8 is function 8), and the GPS's
port needs the peripheral's swap bit because its two pins share one.

**And it can be reflashed from its own console.** The board is put into DFU by
hand with a button *and* a solder joint, and neither has been tried on it; the
firmware it arrived with could reach the ROM by command. So AerialKit has the
same: `dfu` on the console, `ak_board_enter_bootloader()` in the board contract,
and the ROM's vector-table jump in each arch ([00-decisions.md](00-decisions.md)
§7). The checklist that uses it - flash, banner, then one command per sensor,
outputs and pack, with what a silence means - is
[26-ghf435-bringup.md](26-ghf435-bringup.md), which is also where the four facts
about this board that are *read* rather than measured are listed with the
measurement that settles each.

The plan's M-numbering for the AT32 port is deliberately absent. It went in
beside the milestones rather than inside them, because the goal's two MCUs are
still the F405 and the ESP32 and this is a *third* part that the wing happens to
need - and because the milestones are the record of what has been measured
against a goal, while this is a board that has never had an image on it.

**Why it goes no further for now**: it is a multi-session job (the F405's own
arch is about three thousand lines) and what remains is everything that only the
board can answer: no image has been on it, so the clock, the console, the
barometer, the gyro and the outputs are all read rather than measured. **Why it
was started at
all**: the wing is the one airframe whose hardware is in the drawer, the WeAct
F405 on the bench has no inertial sensor and cannot fly anything, so this port
is the shortest path from where the project is to "a fixed wing flies on
AerialKit".

## Evidence rules

Same rules as the rest of this project, because they are what keeps a claim
worth reading:

- A build proves the compiler and linker did what they were told. A flash
  proves the bootloader accepted the image. Neither says anything about pad
  routing, a PWM waveform, or a servo moving.
- Bench evidence — serial transcripts, scope and logic-analyzer captures,
  logs — goes next to the claim it supports, in this repo.
- Anything not measured is written down as unverified, in the same sentence as
  the rest, so nobody has to guess which half is which.
