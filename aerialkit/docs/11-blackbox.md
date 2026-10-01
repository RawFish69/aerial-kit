# AerialKit - the blackbox

There are three logs, because there are three questions. Every fourth loop
iteration one record goes into a **fast ring in RAM**; every fortieth, the same
record goes into a **long ring** as well; and every two hundredth, it goes into
a **ring over flash sectors** - the one that is still there when the aircraft is
picked up out of a field and the battery is not.

| | fast | long | flash |
| --- | --- | --- | --- |
| Rate | 250 Hz (every 4th pass) | 25 Hz (every 40th) | 5 Hz (every 200th) |
| Holds | 1.5 seconds | 15 seconds | about 50 minutes |
| Lives in | ordinary RAM | memory startup does not clear | five 128 KB flash sectors |
| Survives a reset | no | yes | yes |
| Survives losing power | no | no | yes |
| Answers | "what did the loop do just now" | "what was it doing when it stopped" | "what happened on that flight" |

`log` dumps the fast one, `log long` the long one, `log flash` the one in flash,
all as the same CSV over the console:

```text
ak> log
# aerialkit blackbox, 384 records (90 overwritten, so this starts part way through)
# sampled every 4 loop iterations
# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, alt mm above the take-off reference, sticks per-mille, torque percent, motor 0..254
time_ms,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw,alt_mm,...
12345,-12,0,0,0,0,1000,-455,0,-9000,23300,0,500,0,0,0,0,-7,254,0,0,0,1,3,521234567,49876543
```

The last two columns are where the aircraft was - 1e-7 degrees, the module's own
units - and they are the newest thing the record carries, for the question that
comes after a flight that ends somewhere unexpected: *where is it*. The log in
flash is the one that survives the battery, so it is the only one that can
answer that after a crash in a field, and until this it could not: the console
and the telemetry had the position all along and the log that is still there had
none. The `flags` byte says whether a *fix* is behind the two numbers or whether
they are the last ones the module sent - a position from a module that has
stopped answering is a place the aircraft was, not a place it is - and the
simulator checks the last record against where its own airframe ended up, which
a log full of zeroes or of the home position cannot pass.

And it is not free, which is worth writing down rather than discovering in the
linker's output: the record went from 48 bytes to 56, and there are **three**
rings of 384 records - the fast one, the fallback the long log uses on a board
with no retained RAM, and the retained one itself - so the image costs about
9 KB more RAM (69056 B to 78272 B, 52.7% of the part to 59.7%). The flash side
is cheaper than it looks as well: the slot went from 52 bytes to 60, and a
sector still divides exactly, at 2184 records rather than 2520 - so a sector
holds about seven minutes instead of eight and a half, and the five sectors
about thirty-six minutes instead of fifty.

And after the reset the long log was written to explain, the board says so
before it says anything else:

```text
long log: 372 records from the run before, to 14980 ms into it - 'log long'
boot: ok
```

That line is the point of the second ring. A board that resets into a quiet loop
and says nothing is a board that has taken its own evidence with it.

## Why it exists, and why now

The interesting failures last 200 milliseconds and leave nothing behind: an
armed check on the bench that twitched, a first hover that felt wrong, a
failsafe that triggered at the wrong moment. A memory of that is not evidence; a
file is. It is the difference between "it seemed to shake" and "here is the gyro
and the motor outputs for the eleven loops before it shook".

## What is recorded, and in what units

| Column | Unit | Why that unit |
| --- | --- | --- |
| `time_ms` | milliseconds | the board's own clock, so records can be spaced unevenly |
| `gyro_*` | 0.1 deg/s | a tenth of a degree per second is finer than the sensor's noise |
| `accel_*` | 0.001 g | a thousandth of a g is finer than any mount is aligned |
| `roll`, `pitch` | 0.1 deg | the estimate, not the measurement |
| `yaw` | 0.1 deg | a heading, wrapped to +/-180: the estimate itself runs on unwrapped, and nine turns of it would overflow the field |
| `stick_*` | per-mille | -1000 to 1000, which is what the mixer sees |
| `torque_*` | percent | what the control loop asked for, before the authority limit |
| `motor*` | 0..254 | 255 would mean "no output"; 254 is full throttle |
| `state`, `flags` | enum, bits | armed/disarmed/failsafe, and whether RC and the IMU were valid |

Everything is an integer in a fixed unit, which is not only about size: the
console formatter has no floating point on purpose, so a log of integers dumps
over the same console that everything else does. The units are in the header
line, so a reader does not have to find this document.

## Three decisions

- **The ring keeps the newest data.** When it fills, it overwrites the oldest
  record — the newest is what you want after something went wrong — and counts
  what it overwrote, so the dump can say it starts part way through instead of
  quietly appearing to start at the beginning.
- **One record every four iterations.** 384 records at 250 Hz is 1.5 seconds of
  memory. The same records at 1 kHz would be 0.38 seconds, and an aircraft does
  not change meaningfully in a millisecond. The header says how often it
  sampled, so a reader is never guessing.
- **The dump is oldest first**, with a header naming the units and a comment
  line saying how many records were overwritten.

## The long log, and what survives what

The long ring lives in memory that startup does not clear - the same `.noinit`
section the fault record uses on the STM32, and a slice of RTC memory on the
ESP32. Three things are worth being precise about, because a log that is
believed when it should not be is worse than no log:

- **A ring is checked before it is read.** Uninitialised RAM looks exactly like
  a log with an unlucky head and count, so the ring carries a magic, a layout
  version and its capacity, and a ring that fails any of those is thrown away
  rather than read. That is also what stops the F405's ring and the ESP32's
  (which is smaller, because RTC memory is eight kilobytes) from being mistaken
  for each other after a build changes the capacity.
- **It survives a reset, not a power cycle.** Cutting the power loses it, as it
  loses everything else in RAM. A log that survives power wants flash, and that
  is what the third log is - see below.
- **It costs a ring's worth of RAM even where the board has retained memory.**
  The core keeps an ordinary-RAM fallback for a port that has none to offer, and
  on the F405 that fallback is dead weight: about seventeen kilobytes of a hundred
  and twenty-eight, taking the image to just over half of RAM. The linker prints
  it and `AK_LOG_CAPACITY` is the knob; what it buys is one code path with no
  null checks in the flight loop.

## The log in flash, and the one decision it does not make

The third log is the one that matters after a crash: five 128 KB sectors of the
part's own flash, written at 5 Hz, holding a bit under an hour, surviving
everything the other two do not. `log flash` dumps it and `log flash clear`
erases it.

Its shape is a **ring over sectors**, which is the ordinary way to log to NOR
flash: records go into a sector until it is full, then into the next one, and a
sector that is about to be reused is erased first. What is not ordinary, and is
the whole design, is *who decides when*:

> **A sector erase stops the CPU for about a second** - the instructions doing
> the erasing come from the same flash bank - and a second of no control loop
> is a crash. So the log never erases anything itself. It writes into sectors
> that are already erased; when it runs out of them it **stops** and says so;
> and the flight loop gives it room only while the aircraft is disarmed, which
> is the one state that cannot be flying. An aircraft in failsafe may still be
> in the air, and does not count.

What that is worth: the logger never costs the aircraft a millisecond. What it
costs: a flight that fills the region stops logging until the aircraft is on
the ground again, and a part that has never had this firmware on it starts with
six erased sectors and needs no erase at all. `status` says which state it is
in - `full - clears a sector on the ground` is the line to look for.

The rest of the design follows from flash's own rules, which are stricter than
RAM's:

| Rule | What it forces |
| --- | --- |
| a word can only be programmed once between erases - programming can only clear bits | **no counters in flash.** The write pointer is not stored, it is *found*, by looking for the first slot that is still all ones. A header carries a sequence number and nothing in the log is ever rewritten |
| a 128 KB sector holds exactly 2184 records of 60 bytes after its 32-byte header | the slot size is a property of the part rather than a round number, and the tail is accounted for rather than left over |
| a write can be cut in half by a power loss | every header and every record carries an FNV-1a checksum, and one that does not match is **skipped and counted** rather than decoded into a plausible-looking lie. The dump's header line says how many were skipped |
| a sector may hold another firmware's bytes | a sector that is not already erased is not written into: it becomes the pending erase and the log waits |

The region is sectors 5 to 9, and its two neighbours are enforced rather than
remembered: the linker script fails the build if the image grows past
`0x08020000`, and a static assertion in the board file fails it if the region
would reach the configuration's first bank at `0x080C0000`. Firmware that
erased its own settings while logging would be a memorable bug.

It was six sectors until the saved configuration took one for a second bank,
which is what stops a power cut during a `save`'s erase from wiping the
aircraft's settings outright; see the section comment in
`src/boards/AERIALKIT_F405/board.c` and `make config-recycle`. The log's own
subject is unchanged and 128 KB shorter.

Its tests are the ones worth reading in `tests/test_log.c` and
`tests/test_arch.c`: the ring wrapping and the count dropping what the erase
took with it, a slot whose checksum fails being skipped, a header from another
version being ignored, and - on the board's real region, through the modelled
flash controller - 2184 records filling a sector exactly, the log stopping
when the next sector is somebody else's, and the erase on the ground letting it
carry on into a second sector.

## What it is not

- **It is not always armed.** There is no trigger, no pre-arm buffer and no
  rate change on an event. `log reset` before the thing you want to capture.
- **It can be pulled by a tool as well as printed.** The same records come out
  over the config protocol, as the same CSV, with
  `python3 tools/akproto.py --port ... log flight.csv` - the fast ring, which is
  what a tool wants while a board is on a bench. See
  [16-protocol.md](16-protocol.md).
- **All three come out over the protocol**, and that is the difference between
  a log and a log somebody can use: `tools/akproto.py --source flash crash.csv`
  asks for the log in flash and writes the same CSV the console prints
  ([16-protocol.md](16-protocol.md) 0x09). Both targets have that source now -
  the F405's is five 128 KB sectors of its own flash, the ESP32's is a 704 KB
  partition ([17-esp32-port.md](17-esp32-port.md)) - and a build without the
  partition says "not here" rather than pretending.
### The same log, read two ways

The core reads the log a **word** at a time, and on the F405 that is free:
flash is memory there, so a word is a load. The ESP32 is the case where that
assumption stops holding - its flash is behind a vendor call that disables the
cache and talks to the chip - and a resume touches about two thousand words, so
the word path was about two thousand hardware transactions before the aircraft
had done anything at all.

The store therefore has an optional `read_block` ([ak_flashlog.h](../src/core/
ak_flashlog.h)): one call for a whole header or a whole record. Measured in QEMU
on 2026-09-16, same image, same 153-record log:

| What | Boot to `boot: ok` |
| --- | --- |
| empty log | 8.2 s |
| 153 records, read a word at a time | 141.9 s, and 142.8 s on the next boot |
| 153 records, read a block at a time | 27.6 s |

Those are QEMU's prices for emulated flash transactions, not a real part's, and
they say nothing about how long a board takes. What they do measure is how many
calls the firmware makes, which is the part that carries over: the host test
`test_block_reads` runs the same log through both read paths and counts - 12
calls against 128 on the test's small ring, and the two paths are checked to
find the same log, sector for sector. The F405 is unaffected: its store leaves
the hook null and keeps the word path, because for memory-mapped flash the word
path *is* the fast one.

- **None of it has been written to real flash.** The region, the ring, the
  wrap, the checksums and the erase policy are driven end to end on the host -
  against a store the tests own, and against the board's own region through the
  modelled flash controller ([21-port-on-the-host.md](21-port-on-the-host.md))
  - but a store made of ordinary memory is not a part that takes a second to
  erase and wears out after ten thousand cycles. The first flight that ends in
  a field is still the only proof that matters.
