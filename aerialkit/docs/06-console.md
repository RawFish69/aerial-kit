# AerialKit - the console and the parameter table

The console is how a person talks to this firmware. It is line oriented: type,
press enter, read. It has two doors on the F405 - the board's UART at 115200
8N1, and the firmware's own USB device, which arrives on the host as a serial
port with no adapter at all and no baud rate to get wrong
([05-bringup.md](05-bringup.md) step 3a). Both carry the same bytes; what is
typed at one is a command at the other.

**The USB door has to describe itself before a host will open it**, and it
describes itself in a 67-byte configuration descriptor - one packet more than
endpoint 0 carries. Until 2026-09-17 the driver sent the first 64 bytes of that
and then nothing: the continuation built the second packet from past the end of
its one-packet buffer with a length of zero, so the host received a
configuration it could not parse, which means no CDC interface, no
`/dev/ttyACM*` and no console - on the board whose other door needs a USB-TTL
adapter it does not have. Both ports that share the driver had it. The host
checks now read the descriptor the way a host does, 64 bytes and then the last
three, and the whole story is in
[21-port-on-the-host.md](21-port-on-the-host.md#the-usb-console-which-is-where-this-technique-paid-for-itself).

```text
ak> help
commands:
  help                 this
  banner               the boot banner, again - it is printed
                       before a host can attach, so this is the
                       only way to read it back
  version              firmware, board, revision, build stamp
  status               state, uptime, attitude, outputs
  params               every parameter and its current value
  get <name>           one parameter
  set <name> <value>   change one (range-checked, not clamped)
  defaults             back to the built-in values
  save                 write the parameters to the board
  load                 read them back
  clear                wipe the screen
  reboot               reset the board
  dfu                  hand the part to its ROM bootloader (reflash
                       without a jumper), where the board has one
  output               motors, servos, timers, frame counts
  rc                   receiver counters, raw channels, sticks
  imu                  the inertial sensor, if there is one
  baro                 pressure, temperature and height above the bench
  range                how far the ground is, if a rangefinder is fitted
  spi                  sensor bus loopback (jumper MOSI to MISO)
  calibrate [rc]       gyro bias with the aircraft still, or receiver centres
                       with the sticks centred
  calibrate vbat <V>   the pack's divider, against a multimeter reading
  log [reset]          dump the blackbox, or empty it
  log long [clear]     the long log (survives a reset)
  log flash [clear]    the log in flash (survives the battery)
  gps                  position, speed, satellites
  home [clear]         remember this spot, or forget it
  preflight            check the board against what the firmware believes
  proto                config-protocol counters (the binary port)
```

Three of those are about the board rather than the aircraft. `reboot` starts the
firmware again. **`banner` prints the boot banner a second time**, because the
first one is emitted before a host can be attached and is therefore not
recoverable by waiting: measured on the bench board on 2026-09-29, six resets
with the host attaching as soon as the port appeared received 0, 0, 0, 16, 0 and
14 bytes, and the two non-empty rounds were fragments of the `built:` line - the
banner's middle, never its head. **Read on the bench board on 2026-09-29** -
flashed over ROM DFU and then asked:

```text
ak> banner

AerialKit Firmware
  product:  aerialkit-f405
  board:    AERIALKIT_F405 / WeAct STM32F405RGT6
  rev:      a20d7c6-dirty
  built:    Sep 29 2026 / 12:34:22
  clocks:   168 MHz sysclk (HSE 8 MHz x PLL), apb1 42 MHz, apb2 84 MHz
  state:    no sensor drivers - the flight loop runs in failsafe
```

**Its `board:` line is not a hardware reading, and the earlier version of this
paragraph said it was.** `ak_board_name()` and `AK_BOARD_STR` are both
compile-time strings taken from the selected board file
(`src/boards/AERIALKIT_F405/board.c`), so that line names what the image was
*built for* and prints identically on any board the image is flashed onto. The
one thing on it that is measured is the crystal, and that is the line worth
reading on a board that looks dead.
**`dfu` hands the part to its ROM bootloader** - the one that
writes flash - and it is the answer to "how do I flash something else now": on a
board whose bootloader has to be entered with a button (and, on the wing's
flight controller, a solder joint too) the alternative is opening the case. It
is the ROM's own vector table, jumped to from the firmware, so it works while
the console does; a board with no such path - the ESP32 - answers that it has
none rather than pretending. Expect to unplug and replug the cable afterwards:
the host now sees the bootloader's USB device, not the console.

## Why it exists before the sensors do

The first bench session has no IMU, no receiver and no servos: what it has is a
board, a serial adapter and questions. A console answers questions - what clock
did it come up at, did the selftest pass, is the flight loop running, what does
the estimator believe, what gains are loaded - without a rebuild for each one.
It is also the interface sensor bring-up will use to show a register read going
wrong.

## The parameter table

One table, one definition of "a parameter", used by three things: the console,
the saved configuration, and later the configurator.

```text
ak> params
params (90):
  rate_kp_roll          0.250  rate loop P, roll
  rate_kp_pitch         0.250  rate loop P, pitch
  ...
  max_tilt_deg             35  max commanded tilt in angle mode
  airframe                  0  0 quadx, 1 elevonwing, 2 quadx1234, 3 quadp, 4 y4, 5 vtail4, 6 tri, 7 elevonwingsingle (no rudder) - see docs/07-outputs.md
```

Three decisions in it are deliberate:

- **A parameter points straight at the field it controls.** `set rate_kp_roll
  0.4` writes `flight.cfg.rate_kp[0]`, the same memory the control loop reads
  every iteration. There is no copy that can go stale and no "apply" step to
  forget.
- **A value out of range is rejected, not clamped.** A mistyped gain should
  look like an error; clamping turns a typo into a quietly different aircraft.
- **The unit is the one a person thinks in.** Tilt limits are degrees, rates
  are degrees per second. The PID gains stay per rad/s, which is what the
  control loop works in, and the help text says so.

**A parameter can be text, and one of them can be a secret.** A network name or
a password is not a number, and the ESP32's Wi-Fi credentials are the reason
the type exists ([17-esp32-port.md](17-esp32-port.md)); everything else about
them is the ordinary path. The value of `set` is the rest of the line rather
than one word, so an SSID with a space in it works the way it looks like it
should. A secret is one the firmware never prints - `params`, `get` and the
config protocol all show `***` - while `save` and `load` carry the real value,
because an aircraft has to be able to remember its own password.

`defaults` restores the values that were in place when the table was built,
which is right after `ak_flight_init()`. `airframe` is a table entry rather
than a compile-time choice, and changing it takes effect immediately: the mixer
is re-resolved from the airframe number.

**One of those values is the board's rather than the core's**, and it lands in
that same window: between the init and the registration, `main` asks the board
for `ak_board_default_airframe()`. So the
`0` in the dump above is the *WeAct* board's default and the *core's* - a
Feather F405 prints `7` there, because its header brings out two motor pads and
two servo pads and the core's quad-X mix needs four motors (see
[07-outputs.md](07-outputs.md)). `defaults` on that board restores 7, which is
the point: a board whose own pads cannot fly the core's default has to be able
to say so, and a default it cannot arm on is a board that looks alive and never
spins a motor.

`dshot_khz` (150, 300 or 600) is the other one that reaches hardware
immediately: it recomputes the timer period and the two compare values, which
is all that changes between DShot rates. `output` prints what those are now,
along with how many frames have been sent and how many were dropped because the
previous one had not finished - the two numbers that say whether the output
path is alive.

## Saving and loading

`save` writes the table as `name=value` lines into the last 128 KB flash sector
(`0x080E0000`), behind a magic, a length and a checksum. `load` reads it back
and skips names it does not know, so a file written by a newer build does not
stop an older one from starting. The linker script asserts the image never
grows into that sector - an image that did would be erased by its own first
`save`.

**A table that does not fit the record is refused, not saved in part.** The
record holds `AK_PARAMS_TEXT_MAX` (2048) bytes and the table has
`AK_PARAMS_MAX` (96) slots. It used to be 1024 and 64, and both were too small:
the table was full, so the last parameter registered did not exist, and the
64-parameter table serialised to 1135 bytes, so `save` wrote three quarters of a
configuration and reported success - the tail then came back on `load` at
whatever the aircraft happened to be holding. Both numbers were raised, a
registration that is refused is counted and printed at boot, and the ESP32's
own check measures the serialised table on the real firmware (1254 bytes of
2048 at the time of writing).

**And a load says what it did across an upgrade.** Skipping a name this build
does not have is right, and so is leaving a parameter this build has *gained*
at its compiled-in value - but both used to happen without a word, and after a
flash those are exactly the two things a person wants to know. So the record is
read against the table and reported:

```text
ak> load
loaded 1298 bytes: 70 of 71 parameters
ak> load
loaded 1298 bytes: 69 of 71 parameters, 1 gone (gain_removed), 1 new (battery_rth)
```

Three counts and up to two names: how much of this build's table the record
carried, what the record carried that is gone, and what this build has that the
record never saw. The names are the important half - a count says something
happened. It is printed at boot as well, because the boot loads the record, and
`status` repeats it so the answer is available later:

```text
ak> status
...
config:   69 of 71 parameters from the saved record, 1 gone (gain_removed), 1 new (battery_rth)
```

The mark that makes this possible is one flag bit on each parameter, set by a
load and cleared before the next one; nothing about how a parameter is *stored*
depends on it. A value that is out of range for a name that still exists is
still a failed load rather than a skipped line, because a configuration that
half-applies is worse than one that refuses - see the record's own rule above.

**A save stalls the CPU for about a second - but now only once in thirty-two
saves.** The record lives in a *ring*: a save writes the next erased slot of the
sector and the sector is erased only when every slot has been used. What that
buys is the failure the single record could not survive: a save whose program is
refused - a worn cell, a supply that sagged, the cable pulled - used to have
already erased the only copy of the aircraft's settings, and now the record
before it is still there and still what loads. The erase that remains is in the
same flash bank the code is executing from, so every instruction fetch waits
while it runs. That is fine on a bench with the props off, and it is why the
console is a bench tool: an aircraft in the air must not be saving parameters.
The AT32F435's port erases one 2 KB page at a time and erases only the slot it
is about to write, so it has no such stall at all - and a board file with an
even coarser erase unit than the F405's would have a worse one.

**And the firmware enforces that sentence rather than asking politely**: `save`
on an aircraft that is not disarmed is refused, with
`save: refusing - this writes flash, and an armed aircraft is not a bench`. It
is the same rule the four calibrations follow, and it was missing here for the
same reason it was missing on `calibrate accel`: everybody who typed the command
was standing at a bench. What it prevents is concrete - a second of stalled loop
stops the DShot frames, an ESC's own failsafe times out in a fraction of that,
and a quadrotor arrives. `10-calibration.md` carries the check that types it in
that state on purpose.

An interrupted write is rejected rather than half-applied, because the
checksum has to match. A sector that has never been written reads as `0xFF` and
reports "no saved configuration".

## Deliberate omissions

**Nothing here arms the aircraft.** Arming is the arm switch on a receiver, so
no console command can spin a motor on a bench whose receiver is not there - and
that is the rule this section has always been about. The commands that *do* make
a board move are gated rather than absent: `output test` walks every motor and
every servo, one at a time, and **refuses while the aircraft is armed**, and the
three `calibrate` commands take a measurement from a still, disarmed aircraft
and write it into the parameter table.

This section used to say something stronger and by now out of date: that there
was **no motor test, no servo test and no calibration command, and no
machine-readable protocol**. All four exist - `output test` and the
calibrations are the ones the bring-up checklist uses, and the config protocol
is [16-protocol.md](16-protocol.md) with `tools/akproto.py` as a client - so what
is left out on purpose is the narrow thing: there is no way to arm from the
console, and no command that sets an output to a value of your choosing. The
only things that move an output are the flight core and `output test` walking
them in turn, and the second refuses while the first is flying.

**And the armed guard is exactly where it is needed, not everywhere.** `output
test` and the calibrations refuse while armed, because they move an output or
write what a control law flies on. `reboot`, `dfu`, `save` and `defaults` do
not refuse: the first two end the flight rather than change it, the third stalls
the CPU for the length of a flash erase, and the fourth sets parameters back to
how they were built. None of them is something an aircraft in the air is being
typed at - the console is a bench tool and the machine interface is the protocol
([16-protocol.md](16-protocol.md)) - and the alternative, a reboot that a pilot
cannot get from a console when the thing in front of them is misbehaving, is
worse than the hazard it removes. It is written down here because a reader could
reasonably assume the guard is wider than it is.

## The wire it shares

The console's UART carries the config protocol too ([16-protocol.md](16-protocol.md)),
told apart only by a frame's first byte, and neither reader can recognise the
other's bytes: a payload byte that happens to be printable is a character as far
as the console is concerned, and a typed `U` is the byte after a sync that never
came as far as the protocol is concerned. So the arbitration between them is the
one place in this firmware where a byte can be handed to the wrong reader, and it
was - in both directions, for as long as the rule was written out by hand in the
firmware's receive loop:

- a frame abandoned over a bad length leaves the rest of itself still arriving,
  and those bytes reached the console - printable ones into the line buffer, a
  CR or LF among them into a `ak> ` prompt nobody asked for, which is what a
  script reading the port takes to mean "the command has finished";
- a command typed while a half-received frame was still outstanding lost its
  first character to the parser and ran as `unknown command: ersion`.

Both are one rule now, in `src/core/ak_console_link.c`, and
`tests/test_proto.c`'s `test_console_link` drives it through the same entry point
the loop calls. `ak_cli_forget` is the console's half of it: it drops a
half-typed line without running it, and without reprinting the prompt, because
the caller is about to put binary on the wire and `ak> ` in the middle of a frame
is one more thing for the client at the far end to parse around.

## What it costs

Two kilobytes of RAM for the serialisation buffers (one in the console, one in
the board's record) and about 10 KB of flash for the text, the table, the
console and the flash driver. Measured, not estimated: the image is 20 KB of
1 MB and RAM use is 4 KB of 128 KB, which is what `make` reports.
