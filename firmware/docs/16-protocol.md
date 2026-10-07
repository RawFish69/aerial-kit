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
| 0x07 | log get | index (u16) | status, then one record - 87 bytes, or 66 from a board that predates the controller fields, or 51 from one that predates the filtering fields |
| 0x09 | log source | source (u8, or nothing) | status, the selected source, and its record count |
| 0x0A | param info | first (u8) | status, first, carried, then that many entries |
| 0x0B | param help | index (u8), offset (u16le) | status, index, offset, total, length, then the text |
| 0x0C | param default | mode (u8): 1 = one, 2 = all; then index (u8) when mode is 1 | status, then the board's own words (may be empty) |
| 0x0D | rc channels | - | status, flags, protocol, count, then that many raw counts (u16le), four sticks (i16le), switches, seven counters (u32le) |
| 0x0E | sensor info | topic (u8) | status, topic, present, then that topic's body when `present` is 1 |
| 0x0F | output info | - | status, count, motors, servos, the test cap as a percentage, then that many descriptors of 7 bytes |
| 0x10 | output test | op (u8), kind (u8), index (u8), level as a percentage (u8) | status, op, kind, index, the percentage actually driven, how long it will keep going (u16le ms) |
| 0x11 | log stream | source (u8), start (u16le), count (u16le), rate (u8) | status, source, first (u16le), count (u16le), rate — then the range, pushed as frames with no response bit |
| 0x12 | preflight | index (u8), offset (u16le) | status, index, total, verdict, the length of the name, the name, the length of the sentence, then the sentence from `offset` |
| 0x13 | calibrate | verb (u8): 0 status, 1 gyro, 2 rc, 3 accel, 4 vbat, 5 abort; then the face (u8) for accel, or the measured millivolts (u32le) for vbat | status, verb, then the session — active, the face being sampled (`0xFF` for none), the faces already measured (bitmask), samples (u32le), rejected (u32le), then six fixed-point results (i32le each) |
| 0x14 | mission | op (u8): 0 status, 1 start, 2 stop, 3 home set, 4 home clear | status, op, then the mission's whole state — active, requested, waypoints, the one being flown (`0xFF` for none), the switch's channel, reached (u16le), started (u16le), cancelled (u16le), the altitude held (i32le mm) |
| 0x15 | perf | — | status, loops (u32le), samples (u32le), nominal us (u16le), period last/min/max (u32le each), late (u32le), jitter p50/p99/max (u16le each), jitter over (u32le), five per-section averages in tenths of a microsecond (u16le each), five per-section maxima in whole microseconds (u16le each), load in per-mille (u16le) |
| 0x16 | motor telemetry | - | status, count, poles, then that many entries of 19 bytes: flags, eRPM (u32le), rpm (u32le), temperature, its session maximum, millivolts (u16le), milliamps (u16le), the window's packets (u16le) and invalid (u16le) |

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
| 6 | `OUTPUT_INFO` | the board answers `output info` with an output list |
| 7 | `OUTPUT_TEST` | the board answers `output test`, refuses it while armed, bounds the run time and stops on link loss |
| 8 | `LOG_STREAM` | the board pushes a range rather than answering it a record at a time |
| 9 | `PREFLIGHT` | the board answers `preflight` with its own checklist, paged |
| 10 | `CALIBRATE` | the board answers `calibrate` with a session it advances itself, refuses every verb that writes while armed, and reports the samples it took and the ones it refused |
| 11 | `MISSION` | the board answers `mission`, with a navigator behind it, and separates a request from a running mission |
| 12 | `PERF` | the board answers `perf` with a window it has actually measured, and distinguishes a window of zeros from no window at all |
| 13 | `MOTOR_TELEMETRY` | the board answers `motor telemetry` with a speed per motor, telling a motor it cannot hear from one it heard turning at zero. **Set by no board in this tree yet** — see the note under the table |

Bit 10 was reserved for two days, and the way it was written down is worth
keeping, because it is the rule for the next one: *"reserved — set when the
calibration opcode exists **and** behaves as documented"*. A reserved bit is not
a promise about what the opcode will look like; it is a note that the vocabulary
has a word for this and nothing speaks it yet. It was set on the day its reply
was documented below — never earlier, and never because the constant existed. A
constant in `ak_proto.h` and a bit in `features` are two different acts, and
that sentence is what kept them apart.

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

### `motor telemetry` — how fast each motor is turning

```text
request : (nothing)
reply   : status (u8)     0 = this board can hear its ESCs, 1 = it cannot
          count (u8)      how many entries follow, at most 4
          poles (u8)      the pole count the rpm below was divided by, 0 = unknown
          count ×
            flags (u8)    bit0 MEASURED, bit1 RPM, bit2 TEMPERATURE,
                          bit3 VOLTAGE, bit4 CURRENT
            eRPM (u32le)
            rpm (u32le)   meaningful only when bit1 is set
            temperature (u8), session maximum (u8)
            millivolts (u16le), milliamps (u16le)
            packets (u16le), invalid (u16le)
```

There are no arguments, as with `rc channels`: this is what the motors are
doing, not a question about one of them.

**It is a new command and not a field added to `status`.** `status` carries
`motor[4]`, and every byte of it is the output the firmware *asked* for - a
command, never a reading - so a client watching a motor stop and a client
watching a motor the board has no idea about drew the same four bars. Appending
a speed to that frame would have moved the version byte, which moves only when
an existing byte changes meaning.

**It is polled and never streamed**, for `rc channels`' reason exactly: the
console link cannot stream at all, and a board on a cable is precisely where
somebody is holding a motor.

**Three replies look alike and the status and the flags keep them apart**, the
same discipline the receiver's `DECODED` bit follows:

- `status = 1`, one byte and nothing else — this build has no way to hear an
  ESC. Every field comes back empty rather than zero, because a header and four
  entries of zeros would read as four motors that are stopped.
- status ok with every `MEASURED` flag clear — the board has the path and has
  heard nothing yet. Four motors, none reporting.
- status ok with `MEASURED` set and an eRPM of zero — an ESC that answered and
  said it is not turning. This is the only one of the three that is a reading of
  a motor at rest, and it is the reason the flag is a field of its own.

**eRPM and RPM are both here, and so is the pole count.** eRPM is the ESC's own
number: it needs no arithmetic, it is what the notch bank in `ak_rpm_filter.h`
consumes, and it is the only figure a board that was never told a pole count can
honestly send. RPM is `ak_dshot_rpm(erpm, poles)`, and the console's `dshot`
command refuses to assume the fourteen poles an aircraft motor usually has, so
the wire does not either: a board with no `motor_poles` sends `poles = 0`, leaves
`MEASURED` set and `RPM` clear, and a client prints what it has. A client that
divided by a guessed fourteen would be inventing the number it shows, which is
the one thing this section exists to prevent - the same rule that keeps a client
from recomputing sticks.

`temperature`, `millivolts` and `milliamps` are the extended-telemetry fields
`ak_dshot_edt.c` already decodes; each is meaningful only under its own flag,
and `max_temperature` is the session maximum that module holds. There is
deliberately **no motor-health field**: `ak_motor_health.h` decides a verdict
from a window of samples, and a verdict is a different question from a reading,
with its own argument to make.

`packets` and `invalid` are the quality window's two counts. Zero packets is a
true answer and not an absence — the window is a window, and `MEASURED` is how a
client tells "nothing has ever been heard" from "nothing has been heard lately".
Their ratio is the ERPM CRC rate the roadmap's acceptance clause asks to be
under 1 %.

**No board in this tree sets bit 13, and every one of them answers `status = 1`.**
The decode is written and host-tested (`ak_dshot_gcr.c`, `ak_dshot_edt.c`) and
this codec is written and host-tested, but the *capture* half — a port's input
capture, its DMA buffer, where a frame begins — is not written for any board, and
`ak_dshot_edt_t` is still owned by nothing. A board that set the bit would be
advertising a reading it cannot take. The vocabulary is written down and the bit
waits for the port, which is how bit 10 waited for the calibration opcode.

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

### `output info` and `output test` — the two halves of one refusal, overturned

```text
output info
request : -
reply   : status (u8)    0 = here is the list, 1 = this board has no outputs,
                         2 = more outputs than one frame carries
          count (u8), motors (u8), servos (u8), cap (u8, per cent)
          <descriptors>  count of them, 7 bytes each

output test
request : op (u8)        0 = hold, 1 = stop
          kind (u8), index (u8), level (u8, per cent)
reply   : status (u8), op, kind, index, level actually driven, remaining (u16le ms)
```

**These are the opcodes written against a recorded refusal**, and the record had
to be corrected rather than quietly contradicted. `docs/27-configurator.md` says
the configurator "cannot arm or move an output — the protocol has no such
command", and `docs/06-console.md` narrows the console's omissions to "no way to
arm from the console, and no command that sets an output to a value of your
choosing". The first sentence is now false, so it was rewritten; the second is
still true, because it is about the *console*, and `output test` there still
walks the outputs of a disarmed aircraft and still takes no value from the
person typing. The owner took the opposite decision for the wire — a typed,
disarmed-only, hold-to-run output test — and `ak_proto.h`'s comment on `0x10`
carries the properties. What follows is why each of them is there.

**`output info` has no test verb and never will.** Describing the outputs and
driving them are different questions, and an opcode that answered both would let
a client that only meant to ask what is fitted move one.

**A board with no outputs answers `1`, which is not `count = 0`.** An aircraft
with no motor list and an aircraft with an empty motor list are different
claims, and the console's rule about absent sensors applies here unchanged: the
first is a fact about the board ("there is no list to draw") and the second would
be a lie this firmware never has reason to tell.

**`op = 1` is answered before every other check.** A stop that was refused
because nothing was running would be a stop a client could not send, which is
the wrong way round for the one verb whose whole job is to be available — and
the aircraft may be armed by the time the stop arrives, which is exactly when a
client most needs it to land. So stop is checked first, unconditionally, and only
then the gate: kind, then index against the board's shape, then `io->writable`,
which is the same predicate `param set`, `param save` and `param default` use.
Nothing here adds a second opinion about what "safe to write" means.

**The gate is re-read on every frame, not once at connect.** `wire_test_outputs`
calls `ak_flight_config_writable` each time it builds a frame, so an aircraft
that arms mid-hold has its output brought to zero at the next control tick — with
a line on the console and *no reply at all*. A client therefore cannot learn from
this opcode that its hold was cut short, and must not assume a hold is running
because it never saw a refusal.

**`level` is capped by a firmware constant, and the reply says what was actually
driven.** `AK_PROTO_OUTPUT_TEST_MAX_PCT` is 15 and appears in `output info`'s
`cap` byte as well as in the reply, so a screen can state the ceiling before
anything is pressed rather than discovering it in a refusal. A client asking for
more is given the cap and told so — the same shape as the log stream's clamped
rate, and for the same reason: the reply is what will happen, not an echo of what
was asked.

**The hold is renewed by the client, and the request *is* the renewal.** There is
no session and no keep-alive: a hold survives `AK_PROTO_OUTPUT_TEST_MAX_MS` (500)
after the last request that named it, and a client that wants one to continue
sends the same request again at a fraction of that window. This is what makes a
hold safe across a link that dies — the failure mode is the output returning to
zero, not an aircraft running on a number that never expires. It follows that
`remaining_ms` is the whole window every time and **not a countdown**: a client
under a held button is told 500 ms over and over. Drawing it as a countdown is
the mistake this document exists to prevent, and it was made once — a
configurator computed `deadline - now` against its own once-a-second page clock
and printed "another 1313 ms" about a 500 ms window, describing a stop that a
held button was never going to reach.

**The reply names the kind and index it acted on**, so a client that has lost
track of which row it owns can match the answer to a row rather than to whatever
it last sent.

**`AK_PROTO_OUTPUT_MAX` is 13, and it is a division rather than a number.**
`(AK_PROTO_MAX_PAYLOAD - 5) / 7` — five bytes of header, seven per descriptor. It
was a literal 16, and `5 + 16 * 7 = 117` overflowed the 96-byte payload: the
builder stopped writing at capacity while still counting its offset, so the frame
went out with a length byte describing bytes that were never written. The
division is the fix and the invariant is the test — a descriptor that grows must
break the arithmetic rather than the frame.

### `preflight` — one checklist, read twice

```text
preflight
request : index (u8), offset (u16le)
reply   : status (u8)   0 = here is the line, 1 = no such line, 2 = no checklist
          index (u8), total (u8), verdict (u8)
          name_len (u8), name
          sentence_len (u16le), part_len (u8), part
```

`status 2` is one byte and nothing else — a board with no checklist has no count,
no name and no sentence to describe, and sending zeros would invite a client to
print "0 of 0" about a build that simply compiled the checks out. `status 1`
answers three bytes and no more: the index that was **asked for** (the firmware
echoes it, and `0xFF` is its answer when the request named none) and the total,
so a client that walked one line too far learns where the end actually was
rather than getting a bare refusal.

**Index zero is the request that rebuilds the record.** The firmware builds the
checklist when it is asked for index zero and reads the built record for every
other index, so a walk that started at line 3 would be answered from the previous
walk's list. That is the same behaviour a real board has, it is deliberate, and a
client that respects it gets one consistent list out of twenty-four frames.

**The sentence is walked by offset, not truncated.** `sentence_len` is the whole
sentence's length in bytes and `part` is a slice of it starting at `offset`. A
sentence that fits one frame — most of them — arrives whole at offset zero; the
long ones (`outputs`, with its timer numbers and frame counts) take two or three
round trips. There is no truncation path, which is what makes the reply
self-describing: a client can always tell whether it holds the whole line or the
first part of one, and a page longer than the sentence it belongs to is refused
rather than silently overshooting.

**`detail` is the console's sentence verbatim, name and all.** It is not a
summary of the fields beside it and it is not reassembled by the client: the
board's console already prints these lines, this opcode carries the same strings
out of the same record, and `tools/akproto_firmware_check.py` compares the two
against a running simulator. Rebuilding the sentence from a name and a reading
would make every client a second author of the firmware's prose, and the two
would agree right up until one of them was edited.

The verdict byte is the console's own three-valued marker, and the three are
never summed: `FAIL` is the firmware being wrong about itself, `PASS` is a check
that ran and was happy, and `FACT` — `--` on the console — is a reading rather
than a verdict, which is what a board with no receiver or no GPS reports. "24
checks passed" is a sentence no client is entitled to write about a board that
reported nine passes and fifteen readings ([15-preflight.md](15-preflight.md)).

`AK_PROTO_PREFLIGHT_MAX` is 32, and a walk is bounded by it rather than by the
`total` byte. Twenty-four is `preflight_build`'s length today; a board that
answered a larger number has stated something this protocol does not allow, and
the client stops at the bound and reports the count it was given rather than
following it.

### `mission` — the list, and the four verbs

```text
mission
request : op (u8)   0 status, 1 start, 2 stop, 3 home set, 4 home clear
reply   : status (u8), op (u8)
          active (u8), requested (u8), waypoints (u8), flying (u8), channel (u8)
          reached (u16le), started (u16le), cancelled (u16le)
          hold_alt_mm (i32le)
```

**The waypoints are not in this opcode, and that is the design.** They are
parameters — `wp0_lat`, `wp0_lon`, and `wp_count` — so they are already on this
wire through `0x03`, they are range-checked by the table that owns them, they
appear in a backup, and `save` keeps them. The console's `mission add` is a
convenience that writes those same two parameters, not a second kind of storage.
An `add` verb here would be a second way to write them, and the two would
disagree the first time one of them grew a bound the other did not.

**Every verb answers with the whole state, not with an acknowledgement.** A
client that has just sent `start` gets the mission as the board now holds it,
so a screen cannot show a stale panel and does not need a second round trip that
could fail on its own. It is why there is one reply shape for five verbs rather
than five.

**`requested` and `active` are two facts, and conflating them is the bug this
avoids.** The console's `mission start` does not start anything: it sets a
request, and the flight loop starts the mission on a later pass, once the
aircraft is armed and flying. A reply carrying only `active` would show "not
flying" to a client whose start had just succeeded; a reply carrying only
`requested` would let a client print the console's line — "flying 3 waypoints" —
about an aircraft sitting on a bench. The two fields are what happened.

`flying` is the waypoint being flown, or `0xFF` when the navigator is flying
none. It is deliberately out of range for a four-waypoint list: "not flying a
waypoint" and "flying waypoint zero" are the two ends of a mission, and a client
that drew them the same way would mark the first waypoint as the one in
progress. `hold_alt_mm` is reported only while a mission is running, because the
field behind it is left wherever the last mission put it and a number nobody is
holding is not a fact about now.

**Nothing here is gated on the aircraft being disarmed, deliberately.** `stop`
exists to be reachable at any moment including in the air — it is the verb whose
whole job is to be available, so stopping a mission that is not running is a
success rather than an error. And `start` is a request the flight loop honours
only when it is already flying, so an armed board that received one has changed
nothing yet. The gate the other opcodes carry is a gate on *writing
configuration*; this opcode writes none.

**The refusals are the ones the firmware owns.** `no waypoints` is an empty list
with nothing to fly, and `no fix` is a home that cannot be set from a position
the navigator would not use — both are answers about this aircraft that a client
cannot compute, and both are checked against the same two facts the console
checks, so the two routes cannot disagree. `no navigator` is a build with no
navigation module at all, which is a different answer from an empty list, in the
same way `preflight`'s `status 2` is different from a checklist of zero lines.

Whether the pilot *meant* it is not on this list. That is the confirmation, and
it belongs in front of the button, where the person is.

### `calibrate` — the console's four, without the waiting

`calibrate gyro`, `calibrate rc`, `calibrate vbat` and `calibrate accel` have
been on the console since it had one. Every one of them *measures* the aircraft
rather than setting it, and every one of them ended in a parameter write:
`gyro_bias_*`, `rc_mid`, `vbat_ratio`, `accel_bias_*` and `accel_scale_*` are
ordinary table rows, range-checked and kept by `save` like any other. None of it
was reachable over the wire, which is why no configurator could offer a
calibration and why this is the last of the console's bench commands to be
carried across.

**The board owns the session, and the flight loop advances it.** The console's
version of each of these sits in a loop — a thousand milliseconds for the
receiver, up to three seconds for the six accelerometer faces — because the
console is a human wire and the person watching it is the one who asked. This
opcode is dispatched from the flight loop. A calibration that held that loop for
three seconds would stop the stabiliser, the telemetry and the failsafe timer
for exactly as long as someone was willing to hold an aircraft on its side, so
`calibrate` starts a session and returns, the loop feeds it a sample at a time,
and `status` reads it. The precedent predates the opcode: the gyro bias measured
while disarmed has been fed from the loop in `main.c` since long before anything
could ask for it over a wire.

| Verb | Number | Argument | What it measures |
| --- | --- | --- | --- |
| `status` | 0 | — | nothing; reads the session |
| `gyro` | 1 | — | gyro bias, while the aircraft is still |
| `rc` | 2 | — | receiver centres, sticks centred |
| `accel` | 3 | face (u8, 0–5) | one accelerometer face per command |
| `vbat` | 4 | millivolts (u32le) | the pack divider, against a multimeter |
| `abort` | 5 | — | nothing; ends a running session |

**Every reply is a reading, not an acknowledgement**, which is `mission`'s shape
and its reason: a client that has just sent a verb should be able to draw the
board's own answer without a second round trip that can fail on its own. It is
also the only shape that lets a wizard show progress, since the alternative is a
client guessing at a duration the board never promised.

`samples` and `rejected` are the pair worth looking at. `rejected` counts the
samples the calibration refused because the aircraft was moving, and it is
reported rather than hidden for the reason the six-face flow exists at all: a
calibration that quietly averaged in samples taken while the aircraft was moving
is worse than one that failed, because it looks deliberate. `faces` is a bitmask
of the six accelerometer faces already measured — the six-face flow is six
commands that accumulate, so that byte is the wizard's whole state — and it is
meaningful only for `accel`, reading zero for the other three. `step` is the face
being sampled right now, or `0xFF` for none, which is out of range for all six so
that "not sampling" cannot be read as "sampling face zero".

**The verb byte is the session's and not an echo of the request**, which is the
one place the two differ and the reason the field exists at all. `status` is how
a wizard follows a calibration it started, so its reply is a reading of the
*session* — a poll of a running `gyro` reports `gyro`, and the three biases in
the result slots are labelled by the byte beside them rather than by the byte the
client happened to send. `abort` names the session it ended for the same reason.
A board that has never calibrated anything answers `0xFF`, out of range for all
six, so a client cannot read "no session" as "these are the `status` verb's six
numbers".

The six result slots are fixed-point, like everything else here, and what they
mean is fixed by the verb beside them — there is nothing to infer:

| Verb | Slots |
| --- | --- |
| `gyro` | `[0..2]` bias, millidegrees per second |
| `rc` | `[0]` the centre applied, microseconds; `[1..3]` how far roll, pitch and yaw were off it |
| `vbat` | `[0]` the ratio measured, times 1e6 |
| `accel` | `[0..2]` bias, micro-g; `[3..5]` scale, times 1e6 |

Six slots because the accelerometer is the one calibration here that measures a
scale as well as an offset. The other three use a prefix and zero the rest, which
is a real zero — "this measurement has no fourth number" — and not a placeholder.

| Status | Meaning |
| --- | --- |
| 0 | done |
| 1 | refused: no such verb |
| 2 | **refused: the aircraft is armed** |
| 3 | refused: a session is already running |
| 4 | refused: nothing on this board to calibrate |
| 5 | ran, and did not get enough still samples |
| 6 | ran, and the measurement is not one this aircraft will accept |
| 7 | refused: nothing to abort |
| 8 | refused: no such accelerometer face |
| 9 | refused: the voltage given is not a pack this aircraft flies |

**These sentences are the whole of what the wire carries**, which is a status
byte and nothing more: the text is the client's, and three clients render it.
`tools/akproto.py`'s `CALIBRATE_STATUS_NAMES` and the web configurator's
`CALIBRATE_STATUS_TEXT` carry the same ten sentences character for character,
and `apps/configurator/tests/calibration.test.ts` reads this table and the
Python one and fails if either disagrees with the app. It is there because they
had already drifted — status 0 read `ok` in two of the three and `done` in the
console and here, and status 1 had lost the `refused:` prefix every other
refusal carries.

**`nothing` and `no samples` are the pair that matters**, and the console already
keeps them apart. The first is a fact about the *build* — `vbat` on a board with
no pack divider, `gyro` or `accel` on a board with no inertial sensor — and no
amount of trying will change it. The second is a fact about the *aircraft*: the
hardware is there and it would not hold still, or nothing arrived on the wire. A
screen that drew them the same way would send someone to look at a connector
that was never fitted.

Which of the two `rc` answers is worth stating precisely, because it is the one
that looks like it should have a build fact and does not. No board in this tree
lacks a receiver port — the ESP32 boards take one over a UART like everyone else
— so a receiver port with nothing plugged into it is the *aircraft's* answer: a
session that takes no frames and times out as `no samples`. A build that
genuinely had no receiver input would be a different thing and would be
unreachable rather than refused, exactly as `RC_CHANNELS` reserves its `NONE`
status for a build that leaves the callback null.

**Every verb that writes is refused while armed, and that gate is asked three
times.** Once in the dispatch, so the refusal is a policy with a status a client
can render — exactly as `param save` does, and for the same reason. Once in the
board's own callback, so the guard does not depend on the request having come
through `ak_proto.c`. And once by the *session itself*, which is the only one
that matters in the air: a calibration that is running when the aircraft arms
must stop, and that is a fact about the aircraft changing after the command,
which no gate at dispatch time can see. The gyro calibration in `main.c` has
checked exactly that from the loop since before this opcode existed.

`status` and `abort` are not gated, because neither writes anything: a client
must be able to see a session and stop one while armed, and refusing *those*
would leave a calibration running with no way to end it but a power cycle.

**`vbat` trusts a number a person typed**, and that is the one place here where
the firmware depends on the pilot rather than the other way round. The board
cannot know what a multimeter reads; it knows what its own pin sees. So the
client sends the measured volts and the board does the division — and the screen
that offers it has to say that the number in it came from a hand, not from the
aircraft. That sentence belongs in the app and is in `docs/27-configurator.md`.

Whether the pilot *meant* it is not a refusal here. The six faces are six
commands and each one is one click from a wrong bias, so the app puts a step in
front of each — but that is the confirmation above, and it belongs in front of
the button, where the person is. This is also why the calibration tab is the
last one in the plan rather than the first: it is bench work with a hand on the
aircraft, and it is deliberately the only screen in this app that a mistake
costs more than a redraw.

### `perf` — the loop's period, and where the time inside it went

Doc 29 could say what the loop's *period* was and nothing about its *duration*:
`long_loops` and `max_loop_ms` come from the millisecond tick, and a tick cannot
resolve the work inside one iteration. This opcode carries the other half — a
cycle counter, a stamp at each boundary, and the arithmetic that turns the two
into numbers a person can act on. It is the wire's copy of the console's `perf`
command, and the two print the same window out of the same snapshot.

**Two counts, not one.** `loops` is how many periods closed; `samples` is how
many of those the port could actually *time*. They are equal on a board whose
clock works, and they come apart on one whose cycle counter reads zero — which
can still say a period closed, because that needs no clock — and the reason both
are on the wire is what happens when a client divides by the wrong one. Every
timed figure below is averaged over `samples`; a client that used `loops` would
be dividing a histogram that was never fed by a count that kept climbing, and a
percentile walk off the end of an empty histogram returns its last bin. That is
a fabricated number that looks exactly like a measurement.

**The section averages are in tenths of a microsecond**, which is the one place
this reply changes units. A nanosecond figure does not fit a `u16` for any
section that matters, and a whole microsecond would hide the difference between
a loop that is comfortable and one that is nearly out of slot. The five sections
are the profiler's named ones, in the order it prints them — IMU read,
estimator, PID, mixer, output — and the state *between* them, where the loop is
doing the console, the links and the navigation, has no name and no field: it is
the period minus the sum of these, which is why `load` is documented as a lower
bound.

| Field | Unit | Read it as |
| --- | --- | --- |
| `loops`, `samples` | counts | how many periods closed, and how many of those were timed |
| `nominal_us` | microseconds | the period the loop is trying to hold |
| `period_last/min/max_us` | microseconds | u32, so a pathologically long loop reports its real duration rather than saturating |
| `late` | count | periods more than a tenth of `nominal_us` past it — 100 µs at a 1 ms period, and it was a flat 100 µs before phase 1.4, when 1 ms was the only period there was |
| `jitter_p50/p99/max_us` | microseconds | \|period − nominal\| over every loop since the reset |
| `jitter_over` | count | loops past the histogram's last bin — a count, not a clipped value |
| `section_avg_x10[5]` | tenths of a µs | work per section, averaged over `samples` |
| `section_max_us[5]` | microseconds | the longest single run of that section |
| `load_permille` | per-mille of `nominal_us` | the instrumented sections only — a **lower bound** |

`status` is `0` for a window and `1` for none, and the second is not an error
and not an empty window: it is a *build* with no profiler in it, which is a fact
about the firmware rather than about the aircraft. A reply carrying `1` still
has all 59 bytes — a client's offsets do not move with the status byte — and
every number in it is zero, which is exactly why the status byte is there. A
client that rendered that reply as "the loop costs nothing" would be reporting
the opposite of what the board said, and it is the same distinction `hello`'s
absent capability word and `preflight`'s `NONE` make.

The reply is 59 bytes and its size does not depend on what it reports: a window
with every field saturated is the same 59 bytes as an idle one. Whether a
particular board can measure any of this is the port's business and is stated in
`docs/29-timing.md` — the profiler reports `samples` rather than assuming a
clock, so a port without one says so in the numbers instead of in a footnote.

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
i32 lat_e7, i32 lon_e7,
i16 gyro_filtered[3], u16 notch_hz[3], u8 notch_engaged[3]        = 66 bytes
u32 time_us, i16 rate_setpoint[3], i8 pid_p[3], i8 pid_i[3],
i8 pid_d[3], u16 vbat_mv                                          = 87 bytes
```

The three attitude fields are headings in their fixed units - a tenth of a
degree here, 1e-4 radians in a CRSF attitude frame - and they are wrapped to
`+/-180` degrees on the way into the field. The estimator carries them
*continuously*, because the quadrotor's hover subtracts two of them to recover
the course it is making, so the angle the flight core is flying on can be any
number of turns out; the fields it is written into are periodic, and are written
through one function (`ak_wrap_pi()`) so the record, the status body and the
handset's telemetry cannot disagree about which way round the aircraft is.

The layout has grown five times - the yaw and the height for a bad landing, the
position for the question after one, the filter's own columns so that a
notch's effect is a reading rather than an opinion, and the controller's
(setpoint, P, I, D, the microsecond clock and the pack, roadmap 4.1) so that a
tuning pass reads which term did what - and every growth is a change
to this line *and* to `tools/akproto.py`, which decodes the record itself. That
is deliberate:
a client whose offsets are wrong writes a CSV that looks right, so the
protocol test puts distinctive values in the fields the record grew and asserts
the columns they land in, and the check "the row has a column per field the
record has" is what catches a layout that grew on one side only. (It has: the
firmware's own log-record buffer was a hardcoded 48 bytes, which answered "no
record" to a client asking for a 51-byte one - the record size at the time, and
66 since the filtering fields arrived, 87 since the controller's.)

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

## Reading a log without a round trip per record

`log get` answers a question about one index, and for the fast and long rings
that is the right shape: 384 records each, and a client that wants all of them
spends 384 round trips at whatever the link's turnaround is. The flash ring is
not that size. Four 128 KB sectors hold 5 460 records, and 5 460 round trips
is not a slow read - it is a read nobody finishes.

Batching the existing opcode is not available, and it is worth writing down why,
because it is the obvious first idea and it comes back every time somebody reads
the cost:

- 87 bytes of record in a 96-byte payload is one record per frame with nine
  bytes to spare (thirty when it was 66). Two do not fit.
- `log get` carries no sequence number. A client that asked for 99 records in
  one frame and got 98 back would not learn which one was missing - and would
  not learn that it had lost one at all.

So the shape is the other one: the client names a range, and the board pushes it.

```text
request : LOG_STREAM, source (u8), start (u16le), count (u16le), rate (u8)
reply   : LOG_STREAM, status, source, first (u16le), count (u16le), rate
pushed  : LOG_STREAM with **no response bit**, status, source, index (u16le),
          then the 87-byte record when status is 0

aa 55 01 11 5b  00 00 2a 00  <87 bytes>  crc16
 |  |  |  |  |  |  |  \____/
 |  |  |  |  |  |  |   index 42, little endian
 |  |  |  |  |  |  \_ source
 |  |  |  |  |  \____ status 0: a record follows
 |  |  |  |  \_ 91 bytes
 |  |  |  \____ command 0x11, no response bit: this was not asked for
 |  |  \_______ version
 \__\__________ sync
```

The status byte is one of three, and they are the whole of what a reader needs
to reassemble a log:

| | Meaning | Body |
|---|---|---|
| 0 | a record | source, index, then 87 bytes |
| 1 | a hole | source, index, nothing else |
| 2 | the end of the range | source, index, nothing else |

Five decisions are in that design:

- **Every frame names its own index and its own source.** The `log get` reply
  does not have to: it arrives because something asked, so the client already
  knows which record it is holding. A push has nothing above it tying it to a
  request, so four bytes go in every frame to say what it is. Without them a
  client would be inferring which record it held from how many had arrived,
  which is a count, and a count is the one thing a lossy read cannot trust.
- **A record the board will not produce is named, not skipped.** A ring that
  recycled under a reader, or a slot whose checksum did not survive the power
  cut in half, gets `1` and its index. Skipping it would silently shift every
  record after the gap onto the wrong timestamps, and the records after the gap
  are the ones somebody is reading the log for. This is the same rule `log get`
  already follows by counting skips instead of stopping at one.
- **The range ends out loud.** The last frame carries `2`, and the firmware
  clears the stream as it builds it. A stream that went quiet at the end of its
  range would be indistinguishable from a link that died - and "did the read
  finish" is the exact question a blackbox viewer is asking.
- **The reply is what will be sent, not an echo of what was asked.** A count
  running past the end of the ring comes back smaller; a rate past 50 comes back
  as 50; and a rate of `0` in the reply means nothing is coming. That last one
  is one answer covering three reasons - an empty range, a rate of zero, and a
  link that cannot push frames - which is `TELEMETRY`'s rule, kept because the
  two commands are the same shape and a client should not need two.
- **The stream is gated by `can_stream`, exactly as telemetry is.** The console
  is the wire a person types at; frames arriving among their keystrokes would
  make it useless for that. So the capability bit `LOG_STREAM` is set on every
  build that answers the opcode, and whether a *stream* can be pushed is a
  separate question the `case` answers per link. A board that sets the bit and
  then answers the console's request with a rate of zero is telling the truth
  twice.

Three things it deliberately does not do:

- **No sequence numbers, no retry, no resume.** The per-frame index and the
  `DONE` frame are the whole of it. A client that loses the link mid-range asks
  for a range again; the board keeps nothing between requests but the one stream
  the link's parser is holding.
- **One stream per link.** It lives in the parser, as the telemetry rate does,
  so a second request replaces the first rather than adding to it. And a new
  connection starts from nothing.
- **It does not move `log source`.** That selects the ring `log get` reads from,
  and a person reading one record at a time should not have their selection
  moved by a stream they started and stopped. The source a stream reads is in
  the request and in every frame, and nowhere else.

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
