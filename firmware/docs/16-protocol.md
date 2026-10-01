> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - the config protocol

The console is for a person. This is the same information for a program: the NAS
web console, a companion computer on the aircraft, a test script.

```text
AA 55  version  command  length  payload...  crc16(lo)  crc16(hi)
```

The CRC covers the version, command, length and payload - everything except the
sync pair and the CRC itself, which is the only arrangement where a corrupted
*length* cannot be hidden by the bytes that follow it. A reply carries the same
command with the top bit set, so a reply to command 1 is `0x81`: one field to
check instead of a correlation table.

## Commands

| | Command | Payload | Reply |
| --- | --- | --- | --- |
| 0x01 | hello | - | protocol version, product name, parameter count, how many have changed, **capability word, config hash** |
| 0x02 | param get | index | status, name, value as text |
| 0x03 | param set | index, value as text | status, then the board's own words about it (may be empty) |
| 0x04 | param save | - | status |
| 0x05 | status | - | flight state, link, fix type, satellites, attitude, position, motor outputs |
| 0x06 | log info | - | how many records the *selected* log holds |
| 0x07 | log get | index (u16) | status, then one 51-byte record |
| 0x09 | log source | source (u8, or nothing) | status, the selected source, and its record count |
| 0x0A | param info | first (u8) | status, first, carried, then that many entries |
| 0x0B | param help | index (u8), offset (u16le) | status, index, offset, total, length, then the text |
| 0x0C | param default | mode (u8): 1 = one, 2 = all; then index (u8) when mode is 1 | status, then the board's own words (may be empty) |
| 0x0D | rc channels | - | status, flags, protocol, count, then that many raw counts (u16le), four sticks (i16le), switches, seven counters (u32le) |
| 0x0E | sensor info | topic (u8) | status, topic, present, then that topic's body when `present` is 1 |

Everything is text where a person would type it and a fixed-width integer where
a program wants to do arithmetic, and there are no floating-point fields: the
console formatter has no floating point, and the one place that decides what a
parameter means is the parameter table. A protocol that parsed values itself
would be a second answer to a question that already has one.

`param info` and `param help` answer a question the wire never used to carry:
not what a parameter *holds* but what it *is*. The console has printed a
parameter's type, group, bounds and default beside its value since it had a
`params` command; the protocol carried only the value, so every client either
showed a value with no range or invented one. The configurator in this
repository did the second thing, from a thirty-two row snapshot of a
ninety-two parameter board, joined by name — which is a claim about the board
dressed as a reading of it, and it was already wrong.

### `param info` — a page of entries

An entry is an index, not a name to be joined on. A client that joins by name
gives a row whose name it does not recognise *no* metadata, or worse, its
neighbour's; a client that reads by index cannot.

```text
entry : name (NUL-terminated)  type (u8)  group (u8)  decimals (u8)  flags (u8)
        numeric (type 0 or 1): min (text, NUL-terminated)
                               max (text, NUL-terminated)
        text    (type 2)     : max_len (u8, the characters it may hold)
        then, either way   : default (text, NUL-terminated)
```

The two shapes are told apart by `type`, which the entry states before either
one, so nothing here depends on a version byte. `group` is the
`ak_param_group_t` number, and the thirteen names for those numbers are
`ak_param_group_name()` in the firmware — a fixed vocabulary a client carries a
copy of, in the same way it carries a copy of the command numbers in this table.
A number a client does not know renders as **unknown**, which is honest, and
never as a neighbour's name. What the field replaced is worth stating: a screen
that derived headings from name prefixes was claiming a structure the board
never stated, and it would silently misfile every parameter it had never seen.

**The bounds and the default are spelled by the same function that spells the
value** (`ak_params_format_value`), so "0.1" beside a value and "0.1000" in a
range message cannot both be right. A client that formatted a bound for itself
would be a second authority for what a parameter's text means, and the two
authorities would disagree first about a float and then about a secret. **A
secret's default is served as `***`**, exactly as its value is: a default is a
value, and a Wi-Fi password restored by `defaults` is the same string as a Wi-Fi
password.

The reply is paged because an entry is a name and three numbers spelled out and
a frame carries 96 payload bytes. `carried` is what was actually written:

| status | Meaning |
| --- | --- |
| 0 | a page, carrying `carried` entries from index `first` |
| 1 | refused: the request named no index |
| 2 | refused: the entry at `first` does not fit the frame this board would send it in |

**The page stops between entries, never inside one**, and the entry's size is
measured before a byte of it is written. Half an entry reads as the next
entry's bytes — a right value against a wrong name — which is the one mistake
this table has already made once. A client walks from `first = 0`, adds
`carried` to its index, and stops at the first index it is told is past the end
(`carried == 0`, `status == 0`).

That measuring rests on one property of the builder, and it is worth naming
because it was wrong until the real table found it: **a string is written whole
with its terminator, or not at all.** The builder this page uses wrote a string
whose last character landed on the final byte available and then had no room for
the terminator, so the entry that followed read its default out of the missing
byte — a page with a correct length and a correct CRC, whose last entry ran into
the next one's bytes. Every size the page measured had said the string fitted.
It is now impossible rather than checked: the builder measures first, and a
string that does not fit is not started.

**Status 2 is a real answer and not a leftover.** With `AK_PROTO_FRAME_MAX` it is
not reachable today — the longest possible entry is 92 bytes against 93 available
— and it is one admitted text default away from being reachable, which is why it
exists rather than being discovered later. It *is* reachable at any smaller
frame size, and that is not hypothetical: `ak_proto_feed`'s capacity is the
caller's array, which a device with less RAM than an F405 can size below a full
frame, and `tests/test_proto.c` drives it at 60 bytes against a row that cannot
fit. The refusal is about the entry, not about the capacity — the same 60-byte
page carries the shorter row after it. Without this status a client that read
`carried == 0` as "the table ends here" would stop early and never learn the row
exists, and one that re-asked from the same index would ask forever. There is no
third answer available: the entry cannot be split.

A bare `param info` with no argument is **refused**, not answered from zero.
"Empty frame means start at the beginning" is the shape of a booby trap, and the
answer from zero would look exactly like a complete table.

### `param help` — walked by offset

The prose beside a row, which is the one field nobody needs for all ninety
parameters at once and the only one with no useful bound on its length.

```text
request : index (u8), offset (u16le)
reply   : status (u8), index (u8), offset (u16le), total (u16le),
          length (u8), text[length]
```

It is walked rather than paged, which is what makes "the text was longer than a
frame" not a case at all: a client knows it is done when `offset + length ==
total`, and there is no path through this command that loses the tail of a help
string quietly. An offset past the end is an empty tail rather than an error, so
a client that asks once too often gets the same answer as one that stops and
neither has to know which it is.

## The capability word, and why the version byte did not move

`hello` gained two fields on 2026-09-30, appended after the four it had and
never interleaved with them:

```text
protocol version (1)   product name (NUL-terminated)   parameter count (u16le)
changed since saved (NUL-terminated)   features (u32le)   config_hash (u32le)
```

**The version byte stays at 1, and this is the rule that decides such things:
it moves only when an existing byte changes meaning.** Appending is not
changing meaning. A client written against the old `hello` reads its four
fields and stops. A board written before this change answers with a payload
that ends after `changed since saved`, and the client sees a capability word it
does not have — which it must report as *"this firmware predates the field"*,
never as *"this board has no capabilities"*. Those are different claims about
the aircraft and confusing them is the failure this field exists to prevent.

The word is a property of the compiled firmware, not a reading of the hardware,
so it is a plain field on `ak_proto_io_t` and not a callback: there is nothing
to ask at run time and nothing that could answer differently between two calls.

| Bit | Name | Set when |
| --- | --- | --- |
| 0 | `PARAM_INFO` | the board answers `param info` **and** `param help` |
| 1 | `PARAM_DEFAULT` | the board answers `param default` |
| 2 | `APPLIES_ON_WRITE` | a successful `param set` makes the board re-apply its configuration |
| 3 | `GATES_ON_ARMED` | **every** write path — set, save and default — refuses while the aircraft is armed |
| 4 | `RC_CHANNELS` | the board answers `rc channels` with a receiver's live state |
| 5 | `SENSOR_INFO` | the board answers `sensor info` for every topic, distinguishing a build that has no such topic from a board that has no such part |
| 6-11 | `OUTPUT_INFO`, `OUTPUT_TEST`, `LOG_STREAM`, `PREFLIGHT`, `CALIBRATE`, `MISSION` | reserved for the opcodes named, each set when that opcode exists *and* behaves as documented |

A reserved bit is not a promise about what the opcode will look like. `OUTPUT_INFO`
through `MISSION` have no shape written down yet beyond the one line in the
table's own plan, and a bit is set for one of them on the day its reply is
documented above — never earlier, and never because the constant exists.

A bit means the command exists in this build **and does what this document says
it does**. Defining the constant in `ak_proto.h` is how the vocabulary is
written down; setting the bit in `ak_proto_io_t::features` is how a board says
it speaks it, and those are deliberately two different acts.

`PARAM_INFO` covers both `param info` and `param help` under one bit, and that is
deliberate rather than sloppy: they are one answer in two shapes — what a
parameter is, and the prose beside it — and a client that could page the table
but not fetch a row's help would be drawing a form with no explanation of it. A
board with one and not the other is not a thing worth being able to say.

`config_hash` is `ak_params_hash` — FNV-1a over `name=value\n` for every row in
table order, computed through the same text path `param get` answers with, so a
secret contributes its name and `***` and never its contents. It is what a
saved configuration is filed under: two boards with the same hash hold the same
configuration, and a hash that moved between a backup and a restore says which
one of the two changed.

### The bit this build sets, and what it took to set it

`main.c` sets `AK_PROTO_FEATURE_GATES_ON_ARMED`, and it can do so because **every
write route on this wire now consults one predicate**: `ak_proto_io_t::writable`,
which `main.c` wires to `ak_flight_config_writable(&flight)` — the same function
`save_parameters()` has always passed into `ak_params_save`, and the same one the
airborne reload paths ask.

That was not always true, and the history is the reason the bit is worth
describing. Until this milestone `param save` was guarded and `param set` was
not: `AK_PROTO_CMD_PARAM_SET` checked the parameter index and never the flight
state. A board that had set bit 3 then would have been telling a client every
write was guarded while half of them were not — which is the one thing a
capability word must never do, because the client's own refusal is the only
refusal on the near side of the wire, and a client that believes the far side is
guarded stops applying its own.

The gate is asked **per request**, never latched at connect. A session on the
network link outlives an arming — that is the ordinary case for a ground station
— so an answer taken once at connect would permit a `set` on an aircraft that
armed a minute later, while the client's own display, which follows the
heartbeats, said armed. A gate that can disagree with the screen is not a gate.

`writable` returning **null** reads as *yes*, and that is deliberate: a device
with no aircraft to arm is not a device that is permanently disarmed. What a
client is entitled to know is whether the guard is *there*, which is what bit 3
says, and a board sets the bit and the callback together or neither.

### The status byte every write answers with

`param set`, `param save` and `param default` share one vocabulary, because they
are the same kind of answer — *did this write happen, and if not, what stopped
it* — and a client that had to learn three would get one wrong.

| Status | Meaning |
| --- | --- |
| 0 | done |
| 1 | no such parameter |
| 2 | the table refused the value (out of range, wrong type); the words beside it say which |
| 3 | nowhere to save, or nothing to save to |
| 4 | the board's storage refused the write |
| 5 | **refused: the aircraft is armed** |

Status 5 is new here, and it is the one a screen must not conflate with the
others: 2 tells a person to change the number, 3 and 4 tell them the board
cannot persist, and 5 tells them to land. It arrives in the same trailing text
field status 2 has always used, so a client that reads only the status byte is
unaffected and one that reads the text can say why.

`param set` checks the gate **before** the parameter index. Whether the board
will take a write at all does not depend on which parameter was named, and a
client told "no such parameter" while the aircraft is armed would reasonably
conclude that naming a real one would have worked. It would not have.

### `param default` — and the frame that must not mean everything

```text
request : mode (u8)          1 = the one at `index`, 2 = every parameter
          index (u8)         present only when mode is 1
reply   : status, then the board's own words about it (may be empty)
```

**A request that names nothing is refused** (status 2), not read as "all". The
mode byte is what makes "reset everything" something a client has to say on
purpose: a frame truncated in transit and a deliberate bare request are the same
bytes, and the one command on this wire that could turn a lost byte into a
factory reset is not a command to give a default to. Mode 0 is not a synonym for
2; it is a request that named nothing.

A defaulted table is a changed configuration, so the board is told the same way
a `set` tells it — `io->on_change`, the same callback — or the table would say
one thing and the mixer would be flying another. That is the exact failure the
`on_change` callback was added to prevent, one route further along.

**The console will keep disagreeing, deliberately.** `docs/06-console.md` argues
that the console should *not* guard `save` or `defaults`, because it is a bench
tool on a cable with a person watching it. The protocol is not: it is on a
socket, on an ESP32, often with nobody in the room. Two routes with two
policies is the intended state, not an inconsistency to be tidied away.

A parameter whose value is a secret answers `param get` with `***` rather than
with itself, and this is deliberate: the protocol has no authentication of its
own, so anything that can reach the port can ask for every parameter - which is
the whole reason the Wi-Fi password is stored as one and printed as nothing. A
client that needs to *set* one can do that; a client that needs to read one
back is asking the wrong question.

### `rc channels` — what the receiver is hearing

```text
request : (nothing)
reply   : status (u8)
          flags (u8)       bit0 LINK, bit1 FAILSAFE, bit2 DECODED,
                           bit3 NO_INVERTER, bit4 TELEMETRY
          protocol (u8)    0 = CRSF, 1 = SBUS
          count (u8)       how many counts follow
          count × raw (u16le)
          roll, pitch, yaw, throttle (i16le, per-mille; 0..1000 for throttle)
          switches (u8)    bit0 ARM_ON, bit1 ANGLE
          bytes, frames, crc_errors, rejected, lost, failsafe_frames,
          dropped (u32le)
```

There are no arguments, because there is nothing to name: this is what the
receiver is doing, not a question about a channel.

**It is polled and never streamed, deliberately.** The console link cannot
stream at all (`can_stream`), and the tab a person most wants while holding a
transmitter is exactly the one that should work on the cable. A client that
wants it continuously asks continuously; at the rate a handset moves, that costs
nothing.

**The firmware decodes the sticks.** `rc_min`, `rc_mid`, `rc_max` and
`rc_deadband` are parameters, and a client that recomputed roll and pitch from
them would be a second implementation of `centred()` in another language —
disagreeing at the deadband edge, with the screen and the airframe each
believing its own. The counts are sent as well, because a count out of range is
the first sign of a receiver on the wrong baud rate, but they are evidence and
not an input.

`DECODED` is the flag that keeps two replies apart whose sticks are both all
zero: a frame the decode would not use, and a handset sitting centred. The same
bytes, two meanings, and a screen that drew them the same way would be telling a
person their sticks are centred when the board has no idea. It is also why
`AK_PROTO_RC_NONE` exists: a board with no receiver port could answer a frame of
zeros, and that frame is indistinguishable from a receiver that is plugged in
and has never framed — a different fault at a different end of the cable.

The seven counters are named as the receiver names them, so no translation can
go wrong between them and a screen. Which are meaningful is a property of the
protocol, not a placeholder: an SBUS frame has no CRC to fail, and a CRSF frame
cannot carry a receiver's own failsafe flag, so those read zero and zero is the
true answer. `dropped` is the one the receiver cannot know — it is the UART
receive buffer's — and `protocol` is what tells a client which counters to show.

The CRSF return path's own counters (`N frames out, M device replies`, which the
console prints) are deliberately **not** here. They are the telemetry's business
and not the receiver's, and a tab that showed them would have to explain what
they have to do with the sticks.

### `sensor info` — what is fitted, and what it is reading

```text
request : topic (u8)
reply   : status (u8)    0 = this build knows the topic, 1 = no such topic
          topic (u8)     echoed on every path, refusals included
          present (u8)   1 when a part is fitted and a body follows
          <body>         the topic's own, and only when present is 1
```

Five topics, numbered in the order the console's bring-up checklist prints them,
which is the order of how much is lost without them:

| Topic | | Body | Bytes |
| --- | --- | --- | --- |
| 0 | imu | driver, absent reason, whoami, accel[3], gyro[3], align[3], gyro bias[3], samples, errors | 46 |
| 1 | baro | driver, pressure, temperature, have reference, reference, height, have GPS reference, fused height, samples, errors, fails, baro samples, GPS samples | 52 |
| 2 | range | driver, address, max, distance, age, samples, out of range, rejected, faults, fails, land, agree | 49 |
| 3 | battery | ready, have reading, state, cells, volts, volts per cell, pin mV, ratio, rth, warn, critical, samples, rejected, returns | 29 |
| 4 | gps | have fix, fix type, fix ok, satellites, valid now, lat, lon, altitude, speed, course, have home, home lat, home lon, home distance, home bearing, returning, rth enabled, fixes, dropped, config sends | 56 |

Every body is fixed per topic, and every multi-byte field is little-endian. **The
length is therefore a check and not a description**: a client that asked for
topic 4 and got 46 bytes is holding a frame it does not understand, and the sizes
above are what let it say so rather than reading the first 56 bytes of whatever
came. The largest reply is 59 bytes, so none of them can be a short frame.

`driver` is a fixed twelve bytes, NUL-padded. **A name that fills the field
exactly is a legal name** — the field is a bound on the wire, not a promise of a
terminator, and a reader that insists on one refuses the one driver whose name
happens to be twelve characters long.

**A request that names no topic is refused, not defaulted.** An empty payload
asks for topic 5, which is not a topic, and gets the same answer as any other
number this build does not have. "Which sensor did you mean" has no answer that
is safe to guess, and the alternative — the first topic — is a client that
thinks it is reading the IMU while it is reading a barometer.

**Three answers, and keeping them apart is what this opcode is shaped around:**

- `AK_PROTO_SENSOR_NO_SUCH` — **the build does not answer for that topic.**
  Either the number is not one of the five, or this firmware was compiled with no
  sensor reporting at all (`ak_proto_io_t::sensor_state` is null, which is every
  board that has not been through bring-up). The fix is a firmware update.
- `OK` with `present` clear — **this build knows the question and this board has
  nothing there.** The reply stops after the topic and the present byte. The fix
  is a socket, not a driver.
- `OK` with `present` set — a reading, and a zero in it is a zero.

The middle two are the reason the body is *left off* rather than sent as zeros.
A body of zeros is a barometer reading zero pressure, which is a barometer that
has failed, and a client that could not tell that from a board with no barometer
would send somebody looking for a driver when the part is simply not fitted. The
same rule as `AK_PROTO_RC_NONE` one section up, and for the same reason.

There is a fourth answer, and it is not a sensor fact at all: a build that
predates this opcode replies `0x7F`, the unknown-command byte, correlated to the
request. A client must say *that* rather than reading it as `NO_SUCH` — `0x7F` is
a statement about the build, and `NO_SUCH` is too, but only one of them is the
statement "this firmware has no sensor reporting", and a client that collapsed
them would report a hardware absence for a board whose parts may be soldered on
right now and whose firmware is simply older.

Three of the fields are signed for the same reason the present byte exists:

- `range.distance_mm` is negative for **nothing in range.** A wall against the
  lens is 0 mm and there is nothing there at all; a client shown both as `0`
  cannot tell a landed aircraft from one with its rangefinder covered.
- `battery.pin_mv` is negative when **the board has no reading at the pin**, as
  opposed to a divider that is reading 0 mV.
- `gps.home_distance_m` is negative when there is **no home or no fix** to
  measure from, rather than 0 m away.

`gps.fix_type` is **u-blox's own numbering, untranslated** — `ak_gps.h` says so in
as many words, and it is why the numbers are not MAVLink's `GPS_FIX_TYPE`: the
two number the same ideas differently, so a table borrowed from the other adapter
would name every fix type wrong in a way that looks plausible. And
`baro.height_cm` is height **above the reference**, not above the sea: read
`have_reference` first, because without one the number is meaningless rather than
zero.

Every counter in these bodies is the console's own — the same numbers `imu`,
`baro`, `range`, `battery` and `gps` print — and they are here for the reason
they are there: a sensor that reports a value and no history is a sensor nobody
can decide about. `samples` against `errors`/`fails`/`rejected`/`faults` is how a
person tells a working part on a noisy bus from a broken one, and the protocol
carries them rather than recomputing them, because the board is the only thing
that counted.

The body is written field by field in `ak_proto.c`'s `append_sensor_body` and
nowhere is a struct dumped wholesale — the wire has no padding, and `sizeof` is
the compiler's business.

**0x09 is which log the next 0x06 and 0x07 are about**, because an aircraft has
three answers to "what happened" and a tool should not have to know which
command pair belongs to which one:

| Source | What it is | Which boards have it |
| --- | --- | --- |
| 0 | the fast ring in RAM: 250 Hz, 1.5 seconds | every board |
| 1 | the long ring: 25 Hz, fifteen seconds, and it survives a reset where the board has retained RAM | every board (an ordinary-RAM fallback where it does not) |
| 2 | the log in flash: 5 Hz, about an hour, and it survives losing the battery | both targets: the F405's sectors 5-10, and the ESP32's `aerialkit` partition ([11-blackbox.md](11-blackbox.md), [17-esp32-port.md](17-esp32-port.md)) |

A source this device does not have is **refused** - status non-zero, selection
unchanged - rather than reported as a log with no records in it. Those are
different answers, and confusing them is how a tool concludes that a crash left
nothing when the truth is that it is asking the wrong device. With no payload
at all the command reports what is selected now, which is a cheap way for a
client to resynchronise after a reconnect.

`log get` takes an index **within the selected log**, oldest first, so the three
of them are one interface from a client's side. A record that comes back
refused is a slot the power cut in half: it keeps its place in the order, so a
client counts it as a hole and carries on rather than treating it as the end of
the log.

## Two ends, checked together

The firmware's half is `ak_proto.c`, byte-fed like every other parser here, and
the C tests check it against frames built from the wire description above.

The other half is `tools/akproto.py`, and it is checked the only way a protocol
can be: against the firmware's own code, over a pipe. `make proto-test` builds a
simulator that is the real protocol code with the real parameter table behind
it, and drives it with the real client:

There is a second caller of that client, and it is the reason the client is a
library and a command rather than only a command: the Pi 3 companion
(`projects/flight-computer/companion/fc_link.py`) imports it - through
`AKPROTO_PATH`, or from a checkout beside it - and drives the same framing over
a serial port or a socket. One definition of the wire, two callers, and the
companion's own check (`tools/fc_link_check.py`) drives it against this same
simulator - and `make proto-test` runs *that* check too, whenever the companion
is sitting beside the firmware in the workspace, so the second caller is
exercised by the first one's build. A change that breaks the companion's view of
a log record fails there rather than on the bench. When the firmware moves to a
repository of its own the check prints `companion:  not in this checkout` and
carries on, because a firmware repo should not need the Pi's tools to build.

```text
$ make proto-test
client against the firmware's own protocol code
  ok       hello names the product
  ok       and reports the parameter table
  ...
proto end to end: ok
```

And it has been driven against the **ESP32 target as well**, under QEMU, with
the same client - see [17-esp32-port.md](17-esp32-port.md). Two chips, one
client, one parameter table.

Two halves that pass their own tests and disagree in the middle is the classic
way a protocol fails, and neither half can catch it alone. It is also how this
one caught its own first bug: the firmware's parser set its expected length to
zero on entry, so every frame bailed into the checksum state after one byte -
visible immediately when a client sent something real.

## One port, two users

The protocol shares the console's UART, distinguished by the sync byte: while
the parser is idle, a byte that is not `0xAA` is the console's. A tool and a
person therefore coexist without either knowing about the other, and a tool that
sends something malformed hands the port back as soon as its frame is abandoned
(50 ms of silence). `proto` on the console prints the counters, which is how you
tell "the tool is not talking to us" from "the tool is talking nonsense".

**"Idle" there is load-bearing, and it is not the same question as "the parser is
in its sync state".** A frame the parser gives up on - a length past
`AK_PROTO_MAX_PAYLOAD` is the one that happens - is abandoned *before* the sender
has finished, so the rest of that frame is still arriving after the parser has
stopped reading it. Those bytes are not somebody typing, and showing them to the
console does two things: printable ones land in the line buffer and glue
themselves to the next command typed, and a `0x0D` or `0x0A` among them prints a
prompt nobody asked for - which is what a script reading the port takes to mean
"the last command has finished", so it stops reading in the middle of an answer.
The parser therefore stays on the wire until the frame's own gap expires, and the
console's half-typed line is dropped when the wire changes hands rather than
being left to arrive glued to the next command.

The rule is one function, `src/core/ak_console_link.c`, because it used to be
written out by hand inside the firmware's receive loop and was wrong in both
directions at once - that, and `ak_proto_idle_at` asking the same question
`ak_proto_feed` asks, is what `tests/test_proto.c`'s `test_console_link` holds
down.

## Using it

```bash
python3 tools/akproto.py --port /dev/ttyACM0 hello
python3 tools/akproto.py --port /dev/ttyACM0 list
python3 tools/akproto.py --port /dev/ttyACM0 set 12 0.35
python3 tools/akproto.py --port /dev/ttyACM0 status
python3 tools/akproto.py --port /dev/ttyACM0 log flight.csv
python3 tools/akproto.py --port /dev/ttyACM0 --source flash log crash.csv
python3 tools/akproto.py --host 10.0.2.15:5555 status
python3 tools/akproto.py --host 10.0.2.15:5555 telemetry 10
```

**Every one of those lines is executed by a check now**, because one of them was
wrong: the `--source flash crash.csv` line had lost its `log` (`--source` chooses
*which* log; `log` is what pulls it), so the page printed a command that exits
with argparse's usage message - and nothing had ever run it, because every check
in the repository imports this file's `Client` class rather than running its
command line. `tools/akproto_cli_check.py` now drives each line as a subprocess,
with the board on the far end of a pty (the serial path, the same harness the
bench tool's check uses) or of a socket (the `--host` path), and reads back what
a person would see: the hello line, the status panes, the parameter table, one
parameter by index, a CSV of the fast log, and a CSV of the log that survives
the battery. `make test` runs it.

The `telemetry` line is the one it does **not** stream with, and the reason is
the transport rather than the client: the firmware only pushes frames on a
*network* link (`can_stream` is set for the socket and not for the console), so
on this machine that line belongs to the ESP32 under QEMU - `make
net-window-test`, and `scripts/esp32-proto.sh` - where the same `subscribe()`
and `next_telemetry()` pair is driven through the window. What the check pins
here is the other half: pointed at a link that cannot push, the command says
`telemetry: the firmware will not stream` and exits, rather than sitting there
looking like a board that went quiet.

`log` pulls every record and writes the same CSV the console's own `log` prints,
so a flight becomes a file rather than a scrollback. That is the whole point of
the record format: integers in fixed units, one layout, two ways to get it out.
`--source` picks which log - `fast` by default, `long` for the one that
survives a reset, `flash` for the one that survives the battery - and the
header line names the one that was pulled, because a file that does not say
where it came from is a file somebody will misread later.

The record is encoded rather than copied as a struct - a C struct carries
padding that differs between compilers and architectures, and a wire format
whose field offsets depend on which machine built the firmware is a wire format
that works until it does not:

```text
u32 time_ms, i16 gyro[3], i16 accel[3], i16 attitude[2], i16 yaw,
i32 alt_mm, i16 stick[4], i8 torque[3], u8 motor[4], u8 state, u8 flags,
i32 lat_e7, i32 lon_e7                                    = 51 bytes
```

The three attitude fields are headings in their fixed units - a tenth of a
degree here, 1e-4 radians in a CRSF attitude frame - and they are wrapped to
`+/-180` degrees on the way into the field. The estimator carries them
*continuously*, because the quadrotor's hover subtracts two of them to recover
the course it is making, so the angle the flight core is flying on can be any
number of turns out; the fields it is written into are periodic, and are written
through one function (`ak_wrap_pi()`) so the record, the status body and the
handset's telemetry cannot disagree about which way round the aircraft is.

The layout has grown three times - the yaw and the height for a bad landing, the
position for the question after one - and every growth is a change to this line
*and* to `tools/akproto.py`, which decodes the record itself. That is deliberate:
a client whose offsets are wrong writes a CSV that looks right, so the
protocol test puts distinctive values in the fields the record grew and asserts
the columns they land in, and the check "the row has a column per field the
record has" is what catches a layout that grew on one side only. (It has: the
firmware's own log-record buffer was a hardcoded 48 bytes, which answered "no
record" to a client asking for a 51-byte one.)

Without `--port` it speaks the protocol on stdin and stdout, which is how the
tests drive it. With `--port` it needs `pyserial`; with `--host` it opens a
socket, which is how it talks to the ESP32 target.

## Telemetry

`STATUS` answers a question. Telemetry is the same body pushed without one: a
client asks for a rate, and frames arrive until it asks to stop or goes away.

```text
request : TELEMETRY, one byte: rate in Hz, 0 to stop
reply    : TELEMETRY, one byte: the rate that will actually be sent
streamed : TELEMETRY with **no response bit**, u32 uptime_ms, then the
           status body

aa 55 01 08 1a  cc 10 00 00  16 zeros  4 zeros  e9 d6
 |  |  |  |  |  \_______/      |          |
 |  |  |  |  |   uptime      status body  crc16
 |  |  |  |  \_ 26 bytes
 |  |  |  \____ command 0x08, no response bit: this was not asked for
 |  |  \_______ version
 \__\__________ sync
```

That frame is a real one, from the ESP32 under QEMU, and it is the vector
`make proto-test` feeds the client's parser - because the host-side simulator
has no network and cannot push one, and a client checked only against frames it
built itself is checked against nothing. It is the `frame` line of
[evidence/esp32-network.txt](evidence/esp32-network.txt).

Three decisions are in that design:

- **A push is not a reply.** The response bit is what tells a stream from an
  answer, so a client can read both with one piece of code and can still tell a
  disagreement from a burst of telemetry.
- **The firmware says what it will send.** Asking for 200 Hz gets a reply of 50,
  not silence and not 200. The cap is 50 Hz, because the aircraft does not
  change meaningfully in 20 ms and a stream faster than the link fills a socket
  buffer and then delays the config sharing it.
- **And it says when it will send nothing at all.** The answer is 0 on a link
  that cannot push, which is the console: it is the wire a person types at, and
  frames arriving among their keystrokes would make it useless for that. So a
  tool that wants a stream connects to the network port - on the ESP32 that is
  the Wi-Fi link - and a tool that asks over the console is told no instead of
  being given a rate no frame will ever arrive at. (That answer was a lie until
  the configurator needed it: the parser agreed to any rate on any link, and
  only the network loop ever pushed.)
- **A new connection starts from nothing.** The subscription lives in the
  link's parser, so a client that reconnects is not sent the last one's stream.
  It was, until a client in this repository refused the frame that arrived
  before its own reply: the check that caught it is in
  `tools/esp32_proto_check.py`, and the fix is four lines at the top of the
  network link's drain.

## What is not here

- **No framing over the wire for a lossy link.** A CRC catches corruption and a
  gap timeout recovers, but there is no sequence number, no retry and no
  acknowledgement: this is a wire you can see both ends of, not a radio link.
- **No authentication or access control.** Anyone who can reach the port can
  set parameters and read the blackbox. On the F405 that port is a physical
  header pin and the aircraft is in the same room, which is fine. On the ESP32
  it is a socket, and *that* is not fine yet: the network brings a real
  reachability problem with it, and the answer - a key in the parameter table,
  a token in the handshake, or a listener that only ever binds the tailnet
  address - is not written. Until it is, the ESP32's network belongs on a
  network nobody else can route to.
