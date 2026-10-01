> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - the ESP32 port

The second target, and the first one that has been *run* on a board it was not
built for.

```text
ESP-IDF  ──> bootloader, image format, drivers, FreeRTOS
AerialKit ──> the flight core (unchanged) + the board contract for this chip
```

Where the pieces came from, and what every other ESP32 flight controller
already does, is [22-esp32-survey.md](22-esp32-survey.md); why the port is
shaped this way is [12-esp32.md](12-esp32.md).

## What it is

`ports/esp32/` is an ESP-IDF project. `src/arch/esp32/` and
`src/boards/ESP32DEV/` satisfy the same board contract the STM32 board does -
console, clock, time, LED, saved configuration, and the sensor and output
interfaces - and `src/core/` arrives unchanged. `app_main()` calls AerialKit's
`main()` and that is the whole of the glue:

```c
void app_main(void) { (void)main(); }
```

The decision behind this shape is recorded as #6 in
[00-decisions.md](00-decisions.md): the STM32 is bare metal because it starts
the way this firmware expects; the ESP32 is not, and the reason the target
exists is the radio, which is a vendor blob rather than a register map. IDF
supplies the platform; it does not get to own a control law.

## Build and run

```bash
scripts/esp32-qemu.sh 30          # build, then run for 30 seconds under QEMU
```

Needs ESP-IDF (`IDF_PATH`, default `~/esp-idf`) and the `qemu-xtensa` tool from
`idf_tools.py`. Neither is installed by this repository; on this machine both
live under the user's home directory and cost about 6 GB together.

## The family, not one board

The owner's boards are a WROOM, a C3 and an S2/S3, so the target is the
*family*: one IDF project and one set of portable sources, with the board as a
build choice. What changes per chip is the board file - its pins, its LED, and
how many outputs the chip can actually drive - and the sdkconfig fragment that
names the target:

```bash
scripts/esp32-families.sh              # every board that has a file
scripts/esp32-families.sh ESP32S3DEV   # one of them
```

| Chip | Board file | Board | Built here | Run here |
| --- | --- | --- | --- | --- |
| esp32 (WROOM) | `ESP32DEV` | a plain devkit | yes, 532,528 B | yes - QEMU, with the protocol and the configurator over the network |
| esp32s3 | `ESP32S3DEV` | DevKitC-1 | yes, 532,528 B | yes - `docs/evidence/esp32-s3-qemu.txt` |
| esp32s2 | `ESP32S2DEV` | DevKitM-1 | yes, 532,256 B | no - this QEMU has no S2 machine |
| esp32c3 | `ESP32C3DEV` | DevKitM-1 | yes, 532,256 B | no - and no C3 machine either |

All four are one command, and the sizes are close because it is the same
firmware: what differs is the chip's own code generation and which peripherals
its board file declares.

**And both halves of the family's converter run on the host.** The port has
two ADC calibration paths - the ESP32's line fitting and everyone else's
curves - and only one of them could ever be *executed* here: the ESP32 build
compiles the line, the S2/S3/C3 builds compile the curve, and nothing on this
machine runs the second. So `adc.c` is compiled twice on the host, with its
entry points renamed the way the two fault records are, and
`tests/test_arch_esp32_adc.c` drives both: the curve's branch is checked for the
unit translation it repeats and for the *channel* a curve has and a line does
not, which is the field a copy-paste between the two schemes would get wrong -
and which only the newer chips would ever notice.

**The RISC-V toolchain was the last thing in the way, and it is worth writing
down how it went in**: `idf_tools.py install riscv32-esp-elf` installs it (~2 GB
in `~/.espressif/tools/`), and `export.sh` then *does not put it on the PATH* -
because it exports the toolchains for the default target, which is Xtensa. The
compiler is there, under
`~/.espressif/tools/riscv32-esp-elf/*/riscv32-esp-elf/bin/`; the family script
finds it there itself.

**The second chip found two things the first one could not**, and they are the
kind of thing "the family" means: the port had been written for one chip while
claiming to be the family's.

- **The ADC's calibration scheme is not the same.** The original ESP32
  calibrates with a *line* - a reference voltage and a span in eFuse, or the
  datasheet's nominal when the eFuse carries neither - and every other chip in
  the family calibrates with a *curve*: two points, fitted, per channel. The
  line-fitting types do not exist on the S3 at all, so `adc.c` would not
  compile for it. The port picks the scheme per chip now, and the host build
  says which chip it is standing in for (`-DCONFIG_IDF_TARGET_ESP32=1`) because
  that is what the branch keys on.
- **The LEDC's duty resolution differs**: the ESP32 runs to 20 bits, the S2 and
  S3 stop at 14, and IDF hides the wider entries behind
  `SOC_LEDC_TIMER_BIT_WIDTH` - so a port that asks for 16 bits by name does not
  compile for them. The servo resolution comes from that macro now; fourteen
  bits of a 20 ms period is 1.2 microseconds a step, finer than any servo's
  deadband.

**Two chip limits, and both change what the board can be.** DShot here is one
RMT transmit channel per motor, and the **ESP32-C3 has two of them** - against the ESP32's
eight and the S2/S3's four. A C3 cannot drive a quadrotor's four motors at all.
That is why the motor count is a *board* fact (`AK_BOARD_MOTORS`) rather than
the core's maximum: a C3 board will say two, the port will drive two, and the
preflight's mix-versus-board line is what tells its owner that a quadrotor's mix
needs four. The host check covers that shape on this machine
(`tests/test_arch_esp32_output.c`: a board with two motors' worth of channels
drives exactly two).

**And the UART count, which decides whether an aircraft can navigate at all.**
The family has three UARTs on the ESP32 and the S3 and **two** on the S2 and the
C3 - and one of them is the console. So a WROOM or an S3 can carry a receiver
*and* a GPS, and an S2 or a C3 cannot: those two board files declare no GPS port
at all (`ak_board_gps_poll()` answers nothing) rather than pretending to have
one, the preflight says "fix: none yet", and a return-to-home is refused. What
that leaves is an aircraft flown on the sticks - which is a real thing to build,
and it is the *chip's* answer rather than a missing line of code. The way out
for those two is the console's other door: these chips have a USB-Serial-JTAG
peripheral, and a port that put the console there would free UART0 for a
module. That is a change to the port rather than a pin choice, and it is written
down here rather than assumed.

**One QEMU lesson worth writing down**, because it cost an hour: the S3 image
booted its bootloader, loaded the app, and then printed nothing at all. The
firmware was fine - the QEMU command line was not. `idf.py qemu` disables the
watchdogs as part of its own invocation, and a hand-written
`qemu-system-xtensa -M esp32s3 ...` without those globals leaves the timer
watchdogs running, so the app is reset before it can say anything. The boot
transcript is from `idf.py qemu`, and the rule is the obvious one: use the
project's own runner.

## The evidence

`docs/evidence/esp32-qemu-boot.txt` is a real run, captured rather than
described:

```text
AK Firmware
  product:  aerialkit-esp32
  board:    ESP32DEV / ESP32 devkit
  built:    2026-09-14T05:05:25
  clocks:   CPU 160 MHz
faults: none recorded
selftest:  ... sixteen ok lines ...
selftest: passed
console: type 'help'
no saved configuration
preflight: the machine is what the firmware thinks it is
boot: ok
alive: 13200 ms, 9997 loops
alive: 23194 ms, 20000 loops
```

That is the firmware's own selftest - the same one the STM32 builds run - passing
on a second instruction set, its own preflight check agreeing that the console,
clock, tick and configuration store are where it thinks they are, and the flight
loop running at roughly 750 Hz under emulation.

**The loop rate is not a claim about the hardware.** QEMU emulates a CPU that
does not exist at a speed that does not exist; 9997 loops in 13.2 seconds means
the loop is paced by its own 1 ms tick and that each iteration costs about
1.3 ms in emulation. A real ESP32 at 160 MHz will be somewhere else, and
measuring it is a hardware job.

## The three bugs the port found

Both are the kind that only exist at a port boundary, and both would have been
silent on hardware:

- **CMake globs are evaluated once.** The source list was globbed, so the fault
  handler added afterwards was not in the build and the linker could not find
  it. The list is explicit now - make re-evaluates its wildcards every run,
  CMake does not, and the file says so.
- **UART0 is port zero.** The ESP32 console code used `0` as both the port
  number and the "nothing attached" sentinel, so every console write was
  dropped: the firmware booted, ran its whole selftest, and printed nothing.
  The port now keeps an attached flag separately, and the comment explains why
  the value cannot double as a sentinel here.
- **A disarmed aircraft with no sensors reported "failsafe".** The flight core
  asked the IMU before it asked whether it had ever been armed, so the first
  loop iteration on a board with no IMU - which is what this port is - latched a
  failsafe. Nothing had been armed, so there was nothing to fail safe from. The
  disarmed case now comes first *inside* the "something is missing" path, and
  both halves are tested. It only showed up because something ran the flight
  loop on a board with no inertial sensor, which is to say: because the port
  exists.

## What this port does not do yet

- ~~No motor or servo outputs.~~ **Written 2026-09-16**, and it is a different
  design rather than a translation, as predicted: DShot goes out of the **RMT**
  (one channel per motor, a level-and-duration list from the core's
  `ak_dshot_edges()`) and the servos are **LEDC** channels at 50 Hz. What is
  verified now is that it *runs*: the real `src/arch/esp32/output.c` is compiled
  for the host against a modelled RMT and LEDC (`tests/idf-stub`, `tests/
  host_esp32_output_model.c`) and the check reads the symbols back out and
  decodes them into the frame an ESC would see - `tests/test_arch_esp32_output.c`,
  28 checks. **The first run of that found that every frame this file sent was a
  quarter of a frame**: `rmt_transmit()` takes the payload's size in *bytes*,
  and the copy encoder walks it four bytes to a symbol, so passing the symbol
  count sent 17 bytes of a 68-byte frame - a DShot frame cut off after four
  bits. Nothing else here could have seen it: QEMU's ESP32 has no RMT, so no
  frame completes either way, and the report's honest "no frame has completed"
  is the same sentence for a truncated frame as for no frame at all. No ESC has
  been driven; what is left is the pin and the waveform on a board.
  [07-outputs.md](07-outputs.md) has the timing and the two things it cost.

  **And what "ready" means, which had been answering the wrong question**
  (2026-09-18). `ak_esp_output_ready()` used to be "the motors are up and a
  frame has gone out, *or* the servo timer is configured" - and a quadrotor
  declares no servos, so the servo timer alone made a board whose RMT channels
  never came up report its outputs present. It would arm and write DShot frames
  into channels that do not exist: an armed aircraft with motors that never
  turn, which the pilot has no reason to look for. It is now what the *board*
  declares - four motors is four RMT channels, two servos is two LEDC channels
  - and `main.c` states the board's shape only when that answer is yes, so a
  board that is not ready states zero and the arming gate refuses with both
  numbers. The three refusals are in the host test (`ak_host_rmt_refuse`,
  `ak_host_ledc_refuse`, `ak_host_ledc_timer_refuse`), and the whole chain -
  the preflight's line, the boot's "1 problem", the mix line and the arm line -
  is the simulator's `badboard noout` session.
- ~~No sensors, receiver or GPS.~~ **Written 2026-09-16**, and the two halves
  are deliberately different shapes of problem.

  The **receiver** (UART2: CRSF at 420000 8N1, SBUS at 100000 8E2) and the
  **GPS** (UART1 at 9600) are IDF's UART driver with this project's board
  contract on top, and three things are deliberate in `src/arch/esp32/uart.c`:
  the read path never waits (a UART with nothing on it must not cost the loop a
  millisecond a pass - the same rule `rmt_transmit()` taught this port), the
  overflow counter is the driver's own event queue drained in that path, and a
  framing error is *said once* on the console, because that is the symptom of
  the one mistake this port cannot see for the pilot: the wrong protocol, or a
  receiver wired the wrong way round. SBUS is where the second target is better
  off than the first - the F405 cannot invert its own receive line and its
  board file asks for a transistor, this chip inverts in its GPIO matrix - so
  `rc` does *not* print the F405's "no inverter" warning, and the check asserts
  exactly that.

  The **sensor buses** are the same `ak_bus_t` the F405 hands its drivers, over
  IDF's SPI master and I2C master, so the six IMU and six barometer drivers in
  `src/core/sensors` run on either chip. The IMU's bus is *declared* -
  `AK_BOARD_IMU_FITTED`, because a devkit has no inertial sensor and probing
  with a driver is a slow way to learn what one line already says. The
  barometer's bus is *asked*: I2C can answer "is anything at 0x77?" inside a
  bounded time, and the three answers - something answered, nothing answered,
  the bus itself did not come up - are three different lines on the console at
  boot, which is more than a flag can say.

  What is verified: the console answers `rc`, `gps` and `preflight`; the
  receiver's protocol parameter moves the port's line settings and the SBUS
  case inverts; the barometer probe runs and reports; the SPI loopback answers
  instead of hanging. What is *not*: anything at the other end of any of those
  wires. No receiver, no GPS, no sensor and no ESP32 board has been here.

  **And the buses themselves run on the host now** (2026-09-17): `spi.c` and
  `i2c.c` are compiled against a stand-in for IDF's two drivers and a modelled
  pair of devices (`tests/idf-stub/`, `tests/host_esp32_bus_model.c`), with the
  core's own ICM-42688-P and DPS310 drivers on top of them, and the checks are
  the ones a scope would make - the address byte with its read flag, the
  padding byte, the register pointer after a write, a 3000-byte burst as three
  chunks with the select held down, and a device that stops answering costing a
  failed read rather than a hang. 35 checks, in `make test`. It is the same
  method the F405's buses were made believable with
  ([21-port-on-the-host.md](21-port-on-the-host.md)).

  **And both files are at no line the host suite does not reach** (2026-09-18).
  What was left was the *peripheral* refusing rather than a part staying quiet,
  which on this chip is the likelier of the two: a bus another driver has
  already claimed, no device slots left, a transaction the queue will not take,
  and - on the barometer's side - the re-probe that removes the old address
  before adding a new one. All of them end in an error the board reports rather
  than a sensor that reads zeros, and the two loopback failure messages a person
  reads with a jumper in hand are now printed by a test rather than merely
  compiled. Writing them found three things: the barometer *driver* had never
  been opened through this bus (the tests probed it, which stops at the port's
  register reads, so the port's own delay had never been called), the model's
  "silent" mode failed the *queue* call where its own header documents a
  transfer queued and never completed - a different fault, and the one that left
  the driver's collection timeout unreached - and one branch that `-O2` folded
  into its neighbour, which is a coverage artifact rather than a gap.

  **And every one of them is at no line the host suite does not reach now**
  (2026-09-18): the UART and the network joined the two buses, which means the
  ESP32 port has no file left with a gap this machine can close. What that took
  was the *other* role's failures - a station with no netif, an access point
  whose radio will not start, a listener another service holds - and a host-only
  forget seam on the network (it keeps process-global state, so a failed start
  after a successful one needs the port to be able to begin again).

  **And so do the two UARTs** (2026-09-17), which is the last file on this
  target a host can run: `uart.c` against a modelled IDF UART driver
  (`tests/host_esp32_uart_model.{c,h}`) - a receive ring the test fills a byte
  at a time, the driver's event queue, a transmit buffer, and switches for the
  failures a bench cannot arrange on purpose. 28 checks: the port comes up with
  the pins and line settings the board named, bytes come back one a call and in
  order, SBUS reconfigures the line (100000 8E2) *and* inverts it in the GPIO
  matrix while flushing what was in flight, switching back restores CRSF, a
  buffer overflow is drained off the event queue on the poll path and counted
  rather than lost, a framing error is printed **once** and then only counted -
  the check for that one points the console at a mapped register and watches
  the data register for a second line that never comes - and a telemetry frame
  that the driver only took part of is an error rather than a success. What it
  cannot say is whether a receiver is wired correctly; that is
  [05-bringup.md](05-bringup.md)'s.
- **The SPI loopback has no jumper to match it.** `spi` on the console runs the
  same MOSI-to-MISO check the F405 offers, and under QEMU it answers "transfer
  timed out after 0 transfers" - the emulator does not model that peripheral,
  so the transfer never completes and the bounded wait reports it rather than
  waiting forever. One jumper on a real devkit is what turns it into a fact.
  The pins are the board's own guesses, listed in
  `src/boards/ESP32DEV/board.h`.
- **No Wi-Fi that has ever been run.** The radio path is written
  ([below](#the-network)) and compiles, and it is what a real board needs - a
  devkit has no Ethernet PHY at all. What it has not had is a board: QEMU's
  ESP32 has an emulated MAC and no radio, so the automated check exercises the
  Ethernet medium and the Wi-Fi medium is verified by building it and looking
  at what is in the image. Everything in the section below about Wi-Fi is
  therefore a description of code, not of a link that came up.
- **No configuration in flash that survives an IDF partition change.** The
  parameters go into NVS, which IDF owns and which a repartition can move.

## The protocol works there too

`scripts/esp32-proto.sh` builds the image, boots it under QEMU, waits for
`boot: ok`, and then talks to it with the *same client* a person or a companion
computer would use - `tools/akproto.py`:

```text
  ok       hello answers from the ESP32
  ok       with the same parameter table the F405 build has
  ok       a parameter reads back
  ok       and a value can be set
  ok       and reads back as what was set
  ok       save reaches the ESP32's own storage (NVS)
```

That is the whole path on a second chip: a framed request over the emulated
UART, the shared parser, the shared parameter table, and a save that lands in
NVS instead of a flash sector - the board contract doing its job.

**The check has to wait for the boot banner before it speaks.** Bytes written to
the emulated UART before the guest has configured it go nowhere, and the first
version of this check wrote its request into that void and then waited for a
reply that could not come. That is a property of QEMU rather than of the
firmware, and the check says so where it waits.

## The network

Which medium carries the config protocol and the telemetry is a compile-time
choice, and it is the only thing about this port that is not the same on every
ESP32 - because there is no such thing as "the network" on a chip. A devkit has
a radio and no PHY; QEMU has a PHY and no radio; a build that carried both
would have one of them failing on every board it ever ran on, with the same
symptom the choice would have explained. `main/Kconfig.projbuild` is where it is
chosen:

```bash
# the default: QEMU's emulated OpenCores MAC, which is what this machine tests
idf.py -C ports/esp32 build

# a real board: the radio. What it joins, or whether it carries its own
# network, is configuration rather than a build. `sdkconfig.wifi` is a file in
# this repository (`ports/esp32/sdkconfig.wifi`), and the separate build
# directory matters: IDF keeps the sdkconfig it generated, so a build that
# reuses the default one quietly stays on the emulated medium.
idf.py -C ports/esp32 -B ports/esp32/build-wifi \
    -DSDKCONFIG=build-wifi/sdkconfig \
    -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wifi" build
```

**The credentials are parameters, not build configuration** - which is the
change the ESP32 survey asked for ([22-esp32-survey.md](22-esp32-survey.md)).
One image now does all three of the things a board can be asked to do, and
which one it is follows from what was saved:

| Parameter | What it is |
| --- | --- |
| `wifi_ssid` | the network to join; empty means "do not join anything" |
| `wifi_pass` | the WPA2 password, for the station and for the access point. **A secret**: `params`, `get`, the protocol's param get and the boot report print `***`, while `save`/`load` carry the real value |
| `wifi_ap_ssid` | the name this aircraft broadcasts when it is the network |
| `wifi_mode` | 0 auto (join if an SSID is stored, otherwise carry the network), 1 access point, 2 join only |

The bootstrap is the console: a board with nothing in it comes up on no network
at all, and the first `set wifi_pass` + `save` is what puts it on one - over
USB, which is the one link that needs no configuration. That matters because
**the access point is never open**: it needs `wifi_pass` of at least eight
characters, and without one the firmware says so and stays off the air rather
than creating a network anybody nearby can fly an aircraft over. The station
side has the same rule about the networks it will join (`WIFI_AUTH_WPA2_PSK`),
because the config protocol on this port has no authentication of its own.

Two things the Wi-Fi path does that the Ethernet one does not need. It
reconnects: IDF does not re-associate by itself after a router reboots or an
aircraft is carried out of range and back, so a station that is up and
associated with nothing would report "no address" for the rest of the flight.
And it names its role in the console output, because "listening, no address
yet" has different causes for a station and for an access point and the address
alone cannot tell them apart.

Getting the credentials into the table needed a **text parameter type**, and
doing that found a bug worth naming: the table was full at 64 slots, so the
four new parameters were dropped in silence, and the configuration record was
1024 bytes while the table serialised to 1135 - a `save` that wrote most of a
configuration and reported success. Both are fixed (`AK_PARAMS_MAX` 96,
`AK_PARAMS_TEXT_MAX` 2048, an overflow counter the boot report prints, and a
`save` that refuses rather than truncating), and the ESP32 check now measures
the serialised table on the real firmware: 1254 bytes of 2048.

The medium is named in the firmware's own output, because "listening, no
address yet" has different causes on the two and the address alone cannot tell
them apart:

```text
net: ethernet, listening on 5555, no address yet
net: ethernet, 10.0.2.15:5555
```

**The Wi-Fi image is nearly twice the size of the Ethernet one: 930 KB against
520 KB**, measured 2026-09-18 on this build (the numbers in this page were
838 KB against 426 KB until then, and both had gone stale as the port grew). It
is the radio's own stack, its calibration data and NVS. It fits the application
partition in `partitions.csv` (1.25 MB) with **27% to spare**, which is the
first thing to check before this target grows another library.

**And the Wi-Fi half did not compile**, which is how those numbers came to be
measured: `net.c` used the core's `ak_strlen()` in `net_start_access_point()`
without including `ak_text.h`. The default build never compiles that function -
the Wi-Fi and Ethernet halves are a compile-time choice, and QEMU's ESP32 has
no radio - so the configuration a real devkit needs was the one nothing built,
and it had been broken since the Wi-Fi credentials moved into the parameter
table. The include is there now and `scripts/esp32-proto.sh` **builds both
halves** on every run, with a separate build directory and sdkconfig because
IDF keeps the sdkconfig it generated and quietly ignores a default that
changed behind it.

### The Wi-Fi half, run on the host

That paragraph is why this half kept being the part that broke: it is a
compile-time choice, no automated run of this repository reaches it, and the
only thing that would catch a mistake in it is a person with a board. So the
same file is now compiled *and run* a second way. The stand-in IDF headers in
`tests/idf-stub/` gained the four layers `net.c` calls (Wi-Fi, netif, the event
loop, lwip's sockets and FreeRTOS's stream buffers), and
`tests/host_esp32_net_model.c` answers them: a radio that records what it was
asked to do, a netif whose address the test sets, an event loop the test raises
events through, buffers that really are buffers, and a socket that plays a
client - one that goes quiet, then sends a frame, then hangs up. The real port
file is compiled as the Wi-Fi build against that and driven by
`tests/test_arch_esp32_net.c` in `make test`: **38 checks**, and 170 of the
file's 179 lines under `make coverage` where it was not compiled at all before.

What they pin is the *decisions* rather than the radio - which role the image
takes for a given pair of settings, that the driver is told `WIFI_STORAGE_RAM`
(the credentials are parameters, and flash is for the record `save` owns), that
the station asks for WPA2 and the access point refuses to come up without a
real password, that the address and the disconnect events are registered and
acted on, that the report says "not listening" / "no address" / the address,
and the socket task itself with every byte the client sent reaching the flight
loop and every byte the loop wrote leaving on the same pass.

Two of them found something. The access point's name and its length could
disagree: the name was copied into a 32-byte field with `strncpy(dst, src,
sizeof dst - 1)` - 31 characters and a terminator - while `ssid_len` was taken
from the name it was *given*, so a 32-character `wifi_ap_ssid` broadcast a
length the array did not hold. The copy is bounded and always terminated now,
and the length is what it copied. And the `net` command printed `0.0.0.0` where
the address event's own line says "no address yet"; the command is what a
person at a bench types when the configurator cannot find the board, so it says
the same thing now.

One more thing worth writing down, because it is the sort of number that reads
as the wrong unit: the socket task's `select()` timeout is `{ 0, 2000 }`, and a
`timeval`'s field is **microseconds** - so the task wakes every two
milliseconds while a client is attached, not every two seconds. That is what
this loop wants (an answer the flight loop wrote must not wait for the client
to say anything), so it is unchanged; the check asserts the two milliseconds
and the code now says which unit it is.

### The partition table, and the log in flash

This target has a partition table of its own (`ports/esp32/partitions.csv`)
rather than IDF's single-app default, and the reason is a log: the default
gives everything to the application and reserving nothing means a chip with no
free flash, only flash nobody has claimed. The table is the F405's three-way
split in this chip's terms:

| Range | What it is |
| --- | --- |
| `nvs` (24 KB) + `phy_init` (4 KB) | IDF's own storage - the radio's calibration, and the saved configuration this board writes (src/boards/ESP32DEV/board.c). It is the ESP32's answer to the F405's configuration sector, except IDF keeps it for us |
| `factory` (1.25 MB at `0x10000`) | the application |
| `aerialkit` (704 KB at `0x150000`) | the blackbox in flash: eleven 64 KB regions, about fifty minutes at five records a second |

The log itself is the F405's - `src/core/ak_flashlog.c`, the same ring over
regions, the same 48-byte records, the same refusal to erase without being
asked. What is different is underneath it, and that is
`src/arch/esp32/flashlog.c`: reads go through `esp_partition_read` a word at a
time (slower than mapping the partition, and the version that cannot go wrong),
writes through `esp_partition_write`, and an erase is
`esp_partition_erase_range` over one region - sixteen flash sectors, about half
a second of blocked CPU, which is why the core only asks for it while the
aircraft is disarmed.

A build whose table has no `aerialkit` partition gets a board with **no** flash
log and one line on the console saying so, rather than one that writes over
whatever was there. That is also how the core's own limit was found: the log
holds at most `AK_FLASHLOG_MAX_SECTORS` regions, this partition has eleven, and
the limit was eight - so the ESP32 reported "no storage" at boot with nothing
to say why. The limit is sixteen now, and `flashlog.c` has a static assertion
that its region count fits it, because a compile-time answer is the one that
arrives before a bench session.

The port brings up a netif and a listening socket, and the core serves its
config protocol and its telemetry over whatever connects. That split is the
point: the firmware's half is `ak_board_net_poll_rx` / `ak_board_net_write` -
bytes, exactly like the console - and the socket, the task and the stack are the
port's. The F405's answer to the same four functions is "no network on this
board", which is a different thing from "nobody is connected", and the console
reports both.

The two are different cables and the same code above them:

| | QEMU (this machine) | A devkit on a bench |
| --- | --- | --- |
| link | `-nic user,model=open_eth` - an emulated OpenCores MAC and DP83848 PHY | the radio |
| netif | `ESP_NETIF_DEFAULT_ETH()` | `esp_netif_create_default_wifi_sta()` |
| address | DHCP from QEMU's user-mode stack: 10.0.2.15 | whatever the router or the tailnet gives it |
| verified | end to end, by the check above, `esp32-net-watch.sh` and the configurator's window over the forwarded socket | compiled and linked; never run |
| the firmware above it | identical | identical |

**And the emulated end of that column is a *slow* guest when the machine is
busy**, which two of these checks learned the hard way on 2026-09-18: building
this target while the host suite ran beside it turned the console reader (an 8 s
timeout, against a line the guest produces in well under one) and the
configurator's window (one connect, judged on its first round trip) red, and
both were green on an idle machine. The reader waits 25 s now and the window
check presses connect again until the board answers, because what is under test
is the firmware's answering rather than the host's load.

`scripts/esp32-net-watch.sh` is the human view - it boots the same image with
the firmware's port forwarded to this machine and streams telemetry through
`tools/akproto.py`:

```text
== telemetry, at 10 Hz for 5s
# telemetry at 10 Hz, Ctrl-C to stop
    3955 ms  state 0 link 0 fix 0/0  roll    0.0 pitch    0.0 yaw    0.0 ...
    4057 ms  state 0 link 0 fix 0/0  roll    0.0 pitch    0.0 yaw    0.0 ...
```

The whole run is [evidence/esp32-net-watch.txt](evidence/esp32-net-watch.txt),
and the checks - thirty-two now, nine of them on the network - are
[evidence/esp32-network.txt](evidence/esp32-network.txt). What they prove:

| Check | What it means |
| --- | --- |
| a client can reach the firmware's port | QEMU's emulated link, the DHCP client, lwip and the socket all line up |
| the ESP32 answers over the network | the config protocol over TCP, parsed by the client that talks to the F405 |
| with the same parameter table the console has | one table, two transports |
| a value set over the network is the value the console reads | two links, one firmware, no per-transport state that matters |
| the firmware agrees to stream telemetry at 20 Hz | the subscribe path |
| telemetry arrives without being asked for | a push, on a socket, with no request in flight |
| at about the rate that was asked for | nine frames inside the window a 20 Hz stream needs |
| a second client can connect / its first request is answered first / it is not already streaming | the reconnect bug, and its fix |

**A client that connects is not a client that is being served.** QEMU accepts
the host's connection before the guest has a listener, so a port check passes
while the firmware is still booting; the first version of the watch script
believed it and blocked on a request nobody had received. The script now waits
for the firmware to answer a `hello`, which is the only honest signal - the same
lesson as the boot banner on the serial console, one layer up.

## Where this leaves the second target

The port boundary has now been tested twice over: the portable core compiles for
xtensa and riscv32 without a warning (`make port`), and it *runs* on an emulated
ESP32 with its selftest and preflight passing. What is left is the same thing
that is left on the first target: **a board, and the decision about which one**.
The console, the outputs, the sensor buses, the receiver's UART and the pack all
have code behind them on this chip - the paragraphs below are the list, and the
guessed part of it (the pins a devkit brings out) is the part a board settles.
**And every one of those files has now been *run* somewhere**: the outputs, the
two sensor buses, the two UARTs and the pack's converter against models on this
machine ([21-port-on-the-host.md](21-port-on-the-host.md)), and the whole image
under QEMU. The last one to be reached was the converter, and it is the one that
was wrong: a board header that says `ADC1` is a board whose converter IDF calls
`ADC_UNIT_1` - which is **0**, not 1 - so the port had been asking for the
second unit, whose pins GPIO34 is not one of. Nothing on this machine could see
it, because the conversion itself is IDF's; the model is what made the port's
half of the contract visible.

**And the parts of the aircraft the core needs are all represented now.** The
last one was the flight pack: this chip has a converter and the board has a
divider drawn for it (GPIO34, ADC1 channel 6), read through IDF's line-fitting
calibration and handed to the core as volts at the pin - the only conversion
this chip can honestly make, and the reason the arch layer returns millivolts
where the F405's returns counts. Nothing is soldered to it on a bare devkit and
the console says that rather than a number; [19-battery.md](19-battery.md) has
the rest, and the emulated check asserts the honest answer.

### When there is a board: what to look at, in order

**First, which image.** This target builds **two**, and they are for different
hardware: `aerialkit-esp32.bin` is the *emulated Ethernet* build, which is what
every automated check on this machine boots under QEMU (QEMU's ESP32 has no
radio), and `aerialkit-esp32-wifi.bin` is the *radio* build - the one a devkit
takes, with the IMU and the pack divider compiled as fitted
(`-DAK_BOARD_IMU_FITTED=1 -DAK_BOARD_VBAT_FITTED=1`). Both are in
`dist/aerialkit-esp32/` as of 2026-09-18, and the names are the whole of the
difference: **an emulator's image on a devkit boots, talks on the console and
has no network at all**, which is the failure that reads as "the configurator
cannot see the board" and sends somebody to the credentials and the cables.
`scripts/esp32-proto.sh` asserts the pair on every run by reading the medium out
of each map file - the radio's symbols in one, the PHY's in the other, never
both - because that is a mistake a build log cannot show.

So flashing a devkit is the radio's build directory, and the IDF command writes
the bootloader and the partition table at their own offsets as well:

```bash
idf.py -C ports/esp32 -B ports/esp32/build-wifi \
    -DSDKCONFIG=build-wifi/sdkconfig \
    -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wifi" \
    -DAK_VARIANT_DEFINES="-DAK_BOARD_IMU_FITTED=1;-DAK_BOARD_VBAT_FITTED=1" \
    -p /dev/ttyUSB0 flash monitor
```

Nothing below needs a module soldered on. Each step is one line at the console
or one jumper, and each one turns a claim in this document into a fact:

1. **The console.** `idf.py -p /dev/ttyUSB0 flash monitor` and read the banner:
   the product, the revision, the CPU clock and the preflight verdict. If that
   works the boot chain, the image and the console are all real.
2. **The barometer probe**, which needs nothing attached: the boot report says
   `baro: i2c0 came up, nothing answered at 0x77` on a bare devkit. Wiring the
   part on GPIO21/22 then changes that line - and "the bus did not come up" is a
   third answer, so a wire off reads differently from a part that is not there.
3. **The SPI loopback**: one jumper from GPIO23 (MOSI) to GPIO19 (MISO), then
   `spi`. The pattern is in the transcript in step 3d of
   [05-bringup.md](05-bringup.md); a match says the clock, the pins and the
   transfer path all work.
4. **The receiver and the GPS.** `rc` and `gps` print what arrived. Set
   `rc_protocol` to match the receiver before believing either: CRSF and SBUS
   do not share a baud rate, and SBUS's inverted line is handled in the GPIO
   matrix here, so it needs no external inverter. Framing errors appear once on
   the console, not once per frame.
5. **The outputs**, which need a scope before they need an ESC: `output test`
   walks each one, and DShot300's bit is 1.67 µs with a 35% or 70% duty - see
   [07-outputs.md](07-outputs.md). The pins are guesses until a board says
   otherwise, and the preflight prints what they are.
6. **The pack divider**, which needs a multimeter rather than a scope: solder
   the 10k/1k pair to GPIO34, set `AK_BOARD_VBAT_FITTED` to 1, and `battery`
   prints the volts at the pin next to the pack it makes of them. Measure the
   pack with the meter and hand the number to the firmware rather than doing the
   division by hand: `calibrate vbat <what the meter says>` reads the pin and
   writes `vbat_ratio` - the same procedure as the F405's, in
   [19-battery.md](19-battery.md).

   **And the code behind that step is compiled before the part is soldered.**
   `scripts/esp32-proto.sh` builds this board a second time as *the board a real
   devkit becomes* - the radio, `AK_BOARD_IMU_FITTED=1` and
   `AK_BOARD_VBAT_FITTED=1` - because those are compile-time facts, which makes
   the configuration that actually flies one that nothing used to build:

   ```sh
   idf.py -C ports/esp32 -B ports/esp32/build-wifi \
       -DSDKCONFIG=build-wifi/sdkconfig \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wifi" \
       -DAK_VARIANT_DEFINES="-DAK_BOARD_IMU_FITTED=1;-DAK_BOARD_VBAT_FITTED=1" build
   ```

The console on this target is the same firmware console as the F405's - the same
commands, the same `ak> ` prompt - so the automated checklist works here too:
`make bench-check PORT=/dev/ttyUSB0` in `firmware/`, which types the same list
and writes down the answers. What it will say about a bare devkit is "not
fitted" for the sensors and the receiver, which is the honest answer.

**And a bare board's aircraft has now been flown**, which is the claim every
step above rests on: `aerialkit-fw-sim 100 quadrth bare` flies the quadrotor's
return on a machine with no barometer, no rangefinder and no pack divider - the
shape of every devkit here - and lands it on the GPS's altitude where the
barometer would have been and on the barometer where the rangefinder would have
been. The console names each missing part first. Its transcript is
[evidence/sil-bare-board.txt](evidence/sil-bare-board.txt) and the session is
described in [18-software-in-the-loop.md](18-software-in-the-loop.md); it is
the simulator's answer to "can this devkit fly", with the parts a real one
needs named in step 2, 4 and 6 above.
