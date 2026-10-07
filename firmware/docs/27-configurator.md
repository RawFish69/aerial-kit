> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - the configurator

The parameter table had no face until now: it could be read and written through
the console, and `tools/akproto.py` could do the same over the wire, but
changing a value meant knowing which index it was and typing a command. The
configurator is the window that does the same things with the names visible.

```bash
cd aerial-kit/aerialkit
python3 tools/akconfig.py --sim                 # the built simulator, no board
python3 tools/akconfig.py --port /dev/ttyACM0   # a board's USB console
python3 tools/akconfig.py --host 10.0.2.15:5555 # an ESP32 on a network
```

It needs Python 3 and Tk and nothing else - the serial port is opened with the
standard library's `termios`, so a laptop with a USB cable has no package to
install.

## What it does

| | |
| --- | --- |
| The table | every parameter by index, name and value, with a filter box; picking a row puts its value in the edit field |
| Set | through the same range check the console uses, and **it shows the board's own words when a value is refused** - `out of range 0.000..3.000` - which is why the `param set` reply carries a message now ([16-protocol.md](16-protocol.md)) |
| Revert | writes back the values this session first read, as real writes: a `set` changes the running aircraft immediately and the protocol has no undo |
| Save | the board's own write, after which the count of unsaved changes goes back to zero |
| The aircraft | state, link, fix and satellites, attitude, position, motor outputs, and the record count of each of the three logs - with `not on this board` for one the aircraft does not have, which is a different answer from zero |
| Live | a telemetry stream at a rate a person picks, in the same pane the status is in |
| The logs | pick one of the three and **pull it**: every record comes back over the same protocol the command-line tool uses, and the window draws roll, pitch and yaw against time on one scale. A log the aircraft does not have says so instead of drawing an empty box; a log too short to draw says that too |

**The stream is the one thing that depends on which door you came in.** Both the
simulator and a serial port are the *console*, and the console does not stream:
it is the wire a person types at, so frames arriving among their keystrokes
would make it useless for that. Ask for a rate there and the firmware answers 0,
and the window says so. A network link - the ESP32's Wi-Fi port - is where a
stream lives.

## What it deliberately does not do

**It cannot flash.** That is `dfu` on the console and `dfu-util`
([26-ghf435-bringup.md](26-ghf435-bringup.md) has the one command that matters).
Writing flash from a window is the one operation that can leave an aircraft
unable to say what happened to it, and it is not a thing to make easy.

**It cannot arm or move an output.** The protocol has no such command and the
console's rule is that nothing arms an aircraft but a receiver
([06-console.md](06-console.md)); `output test` on the console walks the outputs
with the aircraft disarmed, and that is where that stays.

**It is not a service.** Nothing starts it, nothing listens for it, and it is
not on the NAS's web console - the storage box serves files, and flight control
out of that stack is a decision this project made, reversed, and wrote down
(M7 in [01-plan.md](01-plan.md)).

## This window and the web one, where they differ

There are two faces on the same parameter table: this Tk window, which is what a
laptop with a USB cable runs, and the web configurator in `apps/configurator`,
which is the one that speaks to the other firmwares as well. They are not the
same program and they are not held to the same rules, so a refusal written down
here is a refusal *of this window*:

- **This window does not guard its writes on the flight state.** A `set` from
  here reaches the board while the aircraft is armed. The web configurator
  refuses locally, and — since the protocol grew `ak_proto_io_t::writable`
  ([16-protocol.md](16-protocol.md)) — the board refuses again on its own. That
  is the intended difference rather than a gap to close: this is a bench tool on
  a cable with a person watching the screen, which is the same argument
  [06-console.md](06-console.md) makes for the console's `save`.
- **This window cannot reset the table.** The console's `defaults` does, and the
  wire's `param default` does; the Tk window has no button for either, so
  putting a row for it in the table above would describe a control that is not
  there. Adding one is a small piece of work and it is not claimed here until it
  exists.

## Boards that are not ours

The owner asked for this window to work with the other firmwares - ArduPilot,
PX4, Betaflight, INAV and AerialKit - and the first answer is that there is no
single protocol to be compatible with. The five speak three:

| Firmware | What it speaks | Where the bytes are defined |
| --- | --- | --- |
| AerialKit | its own protocol, and the console | [16-protocol.md](16-protocol.md) |
| Betaflight | **MSP** | `upstream/betaflight-2026.6.1/src/main/msp/` |
| INAV | **MSP**, and MAVLink for telemetry | `upstream/inav-9.1.0/src/main/` |
| ArduPilot | **MAVLink** | `upstream/pymavlink-2.4.49/` (the reference's own dialect) |
| PX4 | **MAVLink** | the same |

So "all five" is three protocols behind one window, and **all three are in**:
this window asks which firmware is on the other end - its own protocol first,
then MSP, then MAVLink - and shows what it can:

```bash
python3 tools/akconfig.py --port /dev/ttyACM0    # a Betaflight board, too
python3 tools/msp.py --port /dev/ttyACM0         # or with no window at all
```

* **It finds the board.** Ours first, because a hello is the only frame a board
  can be asked for before anything is known; then MSP. A board that answers
  neither is reported as unknown rather than guessed at.
* **It shows a Betaflight or INAV board's state**, from MSP's own frames: the
  firmware name, version and board identifier, the arming flags, the attitude,
  the pack, the fix and position, and the motor outputs - in the panes this
  window already had, because `MspConfigurator` answers the same questions
  `Configurator` does.
* **It reads an ArduPilot or PX4 vehicle's state *and its whole parameter
  table***. MAVLink's parameter protocol is standardized
  (`PARAM_REQUEST_LIST` makes a vehicle answer with every parameter by name,
  value and type), so a MAVLink vehicle is the one foreign firmware whose
  parameters land in this window's table - forty of them in the check, by name,
  in the same widget that shows AerialKit's own. A vehicle announces itself
  with a heartbeat, so detection also has something to listen *for* rather than
  something to ask.
* **It reads and never writes.** Parameter writes to somebody else's
  firmware are how a tool crashes an aircraft: the buttons that would write
  refuse in the board's own name, and the parameter pane says why it is empty.
  MSP's parameter model is each firmware's own registry (Betaflight's
  `MSP_SETTING_INFO`/`MSP_SETTING`, INAV's parameter groups), which is the next
  piece of work and not this one.
* **A field the firmware does not carry is a question mark, never a zero.**
  Two firmwares answer this window now and they do not carry the same fields; a
  zero latitude is a place in the Atlantic, and "this board does not say" is a
  different answer from "this board says zero".

**The order the two probes go out in is not cosmetic.** Our hello is asked
first and the MSP question is only asked if nothing answered it, because that
question is bytes our *console* would otherwise see: our firmware shares one
UART between the console, its own protocol and now this, and a window that
greeted every board with somebody else's protocol would be typing at its own
console. Detection is also why the client has a deadline at all - a board that
does not answer is the normal case for the *first* question, not an error.

**MAVLink is the third question, and it is the only one that does not have to
be asked**: a vehicle heartbeats on its own, and it was almost certainly
announcing itself while the two questions above were going out. The ground
station still says hello first, because a vehicle only streams to a system it
has heard from - which is also how this window gets the state instead of
waiting for a vehicle that is waiting for it.

**All three protocols are done, and one question turned out to have no
protocol answer at all.** Reading a *list* of parameters works for AerialKit
(its own table) and for ArduPilot and PX4 (MAVLink's parameter protocol is
self-describing: `PARAM_REQUEST_LIST` answers with every parameter by name,
value and type). It does not exist for Betaflight and INAV, and the oracle says
so in the code rather than in a forum thread:

```text
src/main/msp/msp.c                          case MSP2_CLI_SETTING: 0x3010
    request: the text "name", or "name = value" to set
    reply:   the text "name = value"
src/main/msp/msp_protocol_v2_betaflight.h   MSP2_CLI_SETTING_INFO: 0x3011
src/main/cli/cli.c                          cliGetSettingInfoByName()
    request: "name\0" plus a u16 offset
    reply:   a u16 total length and a window of "pgn=/type=/min=/max=/default="
```

Every command takes a **name**. There is no "list the settings" request in the
protocol, which is why the Betaflight configurator ships its own list of names
for each firmware version - the per-firmware parameter model the goal predicts,
in the protocol's own words. So this window does what the protocol allows: for
an MSP board, **type a setting's name in the filter box and press return**, and
the board's own sentence comes back - `failsafe_throttle = 1050` and then what
the board says the setting is (`pgn=21`, `type=uint16`, `min`, `max`,
`default`) - fetched over MSP v2, with a description longer than one reply read
in windows. `tools/msp.py --setting NAME` does the same from a command line.

**And the list of names, which is now read rather than remembered.** A name
somebody has to know by heart is not a parameter model, so the filter box is a
list when the window knows the release's own names: for a Betaflight or INAV
board it offers them, and picking one asks the board about it exactly as typing
it did. The names come from the **pinned reference checkout in `upstream/`**,
not from a copy pinned in this repository - a copy would be a second version of
somebody else's data, and it would rot; the board says which release it is, and
the list is that release's own table (`tools/msp_settings.py`):

| The board says | What is read | How many, on this machine |
| --- | --- | --- |
| `BTFL` 2026.6.1 | `upstream/betaflight-2026.6.1/src/main/cli/settings.c`, and the `PARAM_NAME_*` macros it is written with (`src/main/fc/parameter_names.h`) | 817 names |
| `INAV` 9.1.0 | `upstream/inav-9.1.0/src/main/fc/settings.yaml`, its `groups:` section | 732 names |

**Both entry shapes are read, and that is the point of the Betaflight half.**
Its table names a setting either with a literal (`{ "failsafe_throttle", ... }`)
or with a macro (`{ PARAM_NAME_GYRO_HARDWARE_LPF, ... }`), and
`gyro_hardware_lpf` appears in **no literal** in `settings.c` - so a reader that
scanned for quoted names would produce a list that looks complete, is wrong, and
is wrong for the entries somebody took the trouble to name once.

**What the list cannot know, and says:** which of those names the *answering*
board was compiled with. Betaflight's table wraps some entries in
`#if defined(USE_...)` and a target's build decides - the vendor's own
configurator has the same caveat. A name the board does not have comes back as
the board's own refusal, quoted with the name that was asked for
(`no setting called 'gyro_lpf1_dyn_expo' on this board (the board refused
0x3010)`) rather than as a command id or, worse, as nothing. And a release this
workspace has no checkout of gets no list at all: the heading says which
version it wanted, because "this firmware has no settings" and "this machine
cannot read them" are different sentences.

**What is left, then, and it is not protocol work:** **parameter *writes*** to
any of those firmwares,
which this window deliberately does not do; the per-firmware parameter *models*
(mode maps and calibration wizards); and the four log formats, which are the
largest piece and the one with the least to do with the flight controller being
ours. A board
that answers none of the three protocols is still reported as *unknown* rather
than guessed at. All of those are recorded in `the historical development plan` with what
they cost, and none of them is needed to fly this aircraft.

### How that half is checked

`tools/msp.py` is the client, `tools/msp_fake_board.py` is a Betaflight board
that is not there, and `tools/msp_check.py` drives one with the other: the
exact six bytes a request is, the checksum's arithmetic, every decoder, the
state dict, a command the board does not answer (it must time out rather than
hang), a reply whose checksum is wrong, a frame that stops early, and the
detection question asked of both a Betaflight stand-in and our own simulator -
**34 checks** (the last four are the name list: that both variant strings map
to a reader, that each release's table is read complete - 817 names for
Betaflight 2026.6.1 *including* the macro-named `gyro_hardware_lpf`, 732 for
INAV 9.1.0 - and that a release with no checkout here is a reason rather than
an empty list). `tools/akconfig_msp_check.py` then builds the real window
against the same stand-in and checks what a person would see: the board named,
an empty table with the reason in words, the state, attitude, pack and motors
from MSP's own frames, a missing field drawn as `?`, the three logs reported as
"not on this board", the write buttons refusing, the live pane filling from
polls, the filter box asking about a named setting, the box *offering* the
release's own names, the refusal of a name the release has and this board does
not - and then the same window against an **INAV** stand-in
(`tools/msp_fake_board.py --inav`), which is what proves the dispatch from what
the board says it is to which release's table is read: INAV 9.1.0's 732 names,
out of `settings.yaml` - **20 checks**. Both run under `make proto-test`.

**And the MAVLink half is checked against the reference implementation rather
than against itself.** That is the difference MAVLink needed: its messages have
an id, a `crc_extra` byte and a field layout per message, none of which can be
guessed, and a client whose encoder and decoder agree with each other looks
perfect while being wrong. So `upstream/pymavlink-2.4.49/` holds pymavlink's
own generated dialect (1.6 MB of a 6.4 MB wheel, kept for this and recorded in
[03-attribution.md](03-attribution.md)), and:

* `tools/mavlink_check.py` - **12 checks** - compares every id, `crc_extra`,
  field order and wire format in AerialKit's table against the reference's own
  numbers, compares the frames this client builds with the frames pymavlink
  builds byte for byte, checks both MAVLink versions, and reads a whole
  conversation - heartbeat, streamed state, and a forty-parameter list - out of
  `tools/mavlink_fake_vehicle.py`, which is a vehicle *built on pymavlink*. Its
  last check is the naming: **PX4 is named PX4, not ArduPilot**, when the
  heartbeat says so (`MAV_AUTOPILOT_PX4` is 12 in the reference dialect);
* `tools/akconfig_mavlink_check.py` - **14 checks** - builds the real window
  against that vehicle and reads the widgets back: the autopilot named, the
  forty-row parameter table, the state panes, the logs reported as absent, both
  write buttons refusing, and the live pane filling - and then the same window
  against a **PX4** vehicle (`mavlink_fake_vehicle.py --px4`), because the
  refusals are made of the vehicle's own name (`this is PX4, system 1 component
  1, state 4: this window reads it and does not write to it`), and "which of
  the two answered" is what decides which sentence a person reads.

**And a parameter frame the link loses is asked for again.** That is what
`param_index` and `param_count` are for, and it is not a nicety: the first
version of the client skipped a `PARAM_VALUE` that did not arrive inside its
timeout and came back with a table one row short - which is how a *slow
machine* turned into a failing check under the sanitizers once. The client
answers a gap the way the protocol says to (a `PARAM_REQUEST_READ` for each
missing index), the fake vehicle can be told to lose one on purpose
(`--drop-param 7`), and the check drives that path deterministically rather
than hoping a timeout happens: "a parameter frame the link lost is asked for
again, not left as a hole in the table".

**Three of the client's message formats were wrong when that check first ran**
- `PARAM_VALUE`'s sixteen-byte id, `STATUSTEXT`'s fifty-byte text and
`AUTOPILOT_VERSION`'s byte arrays - because each message in the generated file
has *two* formats, a `native_format` for an older API and the `unpacker` the
wire actually uses, and the wrong one encodes plausibly. That is the whole
argument for keeping a reference beside the code instead of trusting the table.

**Where the numbers come from is the point of the stand-in.** Every constant
and payload layout in it cites the Betaflight file it came from
(`build/version.h` for `BTFL` and the calendar version, `msp_protocol.h` for
the command numbers and `API_VERSION_MAJOR/MINOR`, `msp.c` for each payload,
`common/streambuf.c` for the length-prefixed strings) - the same method the
CRSF frames are held against in [08-receiver.md](08-receiver.md).

**What those checks cannot prove**: that a real Betaflight board answers the
same way. That is a bench session, and the goal already names it - the F405 in
the drawer takes any of those images, so flash Betaflight, point this window at
it, and write down what its own configurator says. Until then this half is
"checked against the reference implementation's bytes", which is what the
stand-in can honestly claim.

## Where it runs

Anywhere with Python 3 and Tk: the laptop that builds the firmware, or the Pi
beside the aircraft. Which one is not a decision the firmware makes - it is a
client of the same protocol on the same wire, so "where should the configurator
live" stopped being an architectural question and became "which machine is
plugged into the aircraft".

`python3-tk` is the package, and on this machine it was not installed: the Pi
that builds the firmware had no Tk at all until the window check below needed
one, which is worth knowing before assuming a machine can open a window because
it has Python. A machine with Tk and no screen (this one) can still run the
window under `xvfb-run`, which is what the check does.

## How it is checked

A window's *appearance* cannot be tested; almost everything else about it can.
`tools/akconfig.py` is split
so that everything which talks to the aircraft - the connection, the table, the
set with its refusal, the revert, the save, the status, the log counts, the
stream, the log pull - is a `Configurator` with no Tk in it, and
`tools/akconfig_check.py` drives that against the real simulator over a pipe:
**20 checks**, run by `make proto-test` beside the protocol's own check and the
companion's.

What that covers is the part a person would notice: that the table is whole,
that a value in range is taken and reads back, that one out of range is refused
*with the board's reason*, that a refusal leaves the old value alone, that
revert puts back what was read, that a save is the board's write, and that a
console link is told it cannot stream.

**And the window is checked too, which this page used to say was impossible.**
`tools/akconfig_window_check.py` builds the real `Window` - the same class
`main()` builds - on a Tk root, with no screen and no person: it fills the
table, filters it, selects rows, presses `set`, `revert` and `save`, reads the
labels back, asks for a telemetry stream over a console link, and pulls a log
and looks for the traces on the plot canvas - a canvas is a widget, so "did it
draw" is a question about its items rather than about anybody's eyes. Sixteen
checks, and it is *not* a restatement of the controller's: it drives the
widgets, so it fails when the wiring between the two is wrong rather than when
the protocol is.

**The first time it ran, the window did not work at all**, and the four things
it found are the reason the file exists:

1. `Window.connect()` called a bare `configurator` instead of
   `self.configurator`. That name is not in scope in a closure that runs on the
   link's thread, so *opening the window* put `name 'configurator' is not
   defined` where the aircraft's name goes. Every other method in the class is
   correct, which is how it survived: the controller underneath was fully
   checked and the one line that joined it to the widgets was not.
2. The stream button read the rate box (`self.rate.get()`) *inside* the closure
   that runs on the link's thread. Tk refuses that - "main thread is not in main
   loop" - so asking for a stream reported a Tk internal instead of the
   firmware's answer. The rate is read on the widgets' thread now and carried
   into the closure as a number.
3. A refused or accepted `set` reads the table back and refills it, and
   refilling a `Treeview` drops its selection: the row a person was editing was
   gone, so the next press of `set` said "pick a parameter first". The refill
   keeps the selection.
4. `revert` wrote the old values to the aircraft but did not re-read the table,
   and the window shows the table it holds - so after a revert the window went
   on displaying the values it had just undone. `revert` reads back now, the way
   a set does.

None of those four is visible to the controller's check, and all four are
visible to anybody who opens the window. What is left to a person is what a
machine should not judge: whether the columns are wide enough and whether the
buttons are where a hand expects them.

## The other half of the window: the stream

Everything above runs over a *console* link, and a console link cannot stream -
the firmware answers a subscribe with 0 and says why. So the half of the window
that only exists for a network - `Link` reading frames nobody asked for twice,
and the live pane drawing them - had no check at all, on the argument that the
only firmware that streams is the ESP32 and the only ESP32 here is emulated.

It is emulated, and it streams. `tools/akconfig_net_check.py` boots the ESP32
image under QEMU with its port forwarded, waits for the firmware's own *hello*
rather than for the port to answer (QEMU accepts a connection before the guest
has a listener), and then runs the same `Window` against `127.0.0.1:5555` - the
shape of a person on a laptop watching an aircraft in the air. Six checks: it
connects and reads the table (94 parameters, the number this port is pinned to),
**the stream is allowed here and comes back at the rate asked for**, the live
pane fills with frames the board sent unasked, and the stream stops when it is
asked to.

That is the whole of the configurator's client surface now: a pipe, a socket,
and a window over each. The serial port is the one door left, and it needs a
board - `--port /dev/ttyACM0` is the same `Serial` transport the tools have
always had, and a board's USB console is what it reaches.

The network check needs QEMU and - for `esptool`, which merges the image -
ESP-IDF on the path, so it is not part of `make test`: `scripts/esp32-proto.sh`
runs it beside the port's own protocol check, `make net-window-test` runs it
alone, and it skips with a line wherever the emulator or Tk is missing.

## The F405 over the serial door, frame by frame

The door above is the one that needs a board, so on 2026-09-29 it was measured
against the Feather F405 with `f7ede87` in flash, using the app's own frame
format and the app's own reply layout - `hello` version byte, NUL-terminated
product, u16 parameter count, NUL-terminated changed count; `param get` status
byte, name, value; `param set` status byte and message; `status` five bytes,
three i16 tenths-of-a-degree, two i32 e7 degrees, four motor bytes.

| what the window sends | what the board answered |
| --- | --- |
| `HELLO` | `01` `aerialkit-f405` `5c 00` (92) `"0"` - protocol 1, and the product string is exactly the table's `captured_product` |
| `PARAM_GET` 0..91 | all 92 served, `rate_kp_roll 0.250` … `launch_timeout_s 5` |
| `PARAM_GET` 92 | status 1, refused - the end of the table, which is what the enumeration loop stops on |
| `PARAM_SET` 0 = the value already there | status 0, value unchanged on read-back |
| `PARAM_SET` 0 = `99999` | status 2, refused, value unchanged |
| `STATUS` | 22 bytes, all zero - see below |
| `version` typed after all of it | answers normally, so the console and the protocol are still sharing the port |

**All zero is the right answer here, and the console says so.** That status frame
is `flightState 0`, `linkLive 0`, no fix, no satellites, roll/pitch/yaw 0, lat/lon
0, four motors 0 - and the board's own `status` on the same connection says
`state: disarmed`, `attitude: not converged`, `motors: 0 0 0 0 per-mille`,
`links: 0 rc, 0 gps`. `proto_status()` fills every field from the flight core;
nothing in it is a stub. A bench board with no IMU, no receiver and no GPS in
failsafe reads as zeros in the live pane, and that is the board rather than the
seam.

**The metadata table covered a third of the board, and that was the bug.**
Written on 2026-09-29, this paragraph said the board serves 92 parameters and
`src/firmware/parameter-table.json` has 32 rows; that the 32 names were all real
names; and that the other 60 arrived with `meta: null`, "shown as name and value,
with no help text, range or units". Every clause of that was true, and the
arrangement was wrong anyway. The file was a build-time read of `ak_flight.c`
shown beside whatever board answered, and it was **32 rows against the firmware's
33** - `arm_accel_lpf_hz` was missing and every row from index 13 on was
displaced by one. Nothing failed for a whole release, because every consumer
joined the file to the board's reply by *name*: a name the file lacked produced a
row with no range, which is exactly what the paragraph above describes as normal.
A correct-looking reading of a wrong mechanism.

Milestone 3 removed the second authority rather than regenerating it. The board
describes itself over `AK_PROTO_CMD_PARAM_INFO` (0x0A) and
`AK_PROTO_CMD_PARAM_HELP` (0x0B) under `AK_PROTO_FEATURE_PARAM_INFO`, the app
walks that table by index and joins on index, and the file is now
`src/firmware/demo-table.json` - the demo board's *starting* table, which that
board then serves back over the same two opcodes. There is no fallback. On this
`f7ede87` board, whose `hello` ends before the capability word, every row now
reads `no description`, and the connection says which two opcodes went
unanswered. The shape of the mistake is recorded in
`apps/configurator/tests/demo-table.test.ts` and
`apps/configurator/tools/check-table-drift.py`, which is the check that would
have caught it.

**A receiver tab that had to be taught the same lesson as the table.** Milestone
5 added `AK_PROTO_CMD_RC_CHANNELS` (0x0D) under
`AK_PROTO_FEATURE_RC_CHANNELS`, and the web app's Receiver tab with it. The
opcode answers with a status byte and stops when the status is
`AK_PROTO_RC_NONE` - this board has no receiver port - and otherwise carries the
raw counts, the four sticks *as the firmware decoded them*, the switch byte and
the receiver's seven counters.

Three of those replies look alike and are not, so the wire keeps them apart and
the app keeps them apart in the same places:

| the board is saying | the bytes | how a reader tells |
| --- | --- | --- |
| no receiver port at all | one byte, `AK_PROTO_RC_NONE` | the status byte |
| a receiver that has never framed | a full frame, link flag clear, sticks at zero | `AK_PROTO_RC_LINK` clear |
| a handset sitting centred | a full frame, link flag set, sticks at zero | `AK_PROTO_RC_DECODED` set |
| a firmware that predates the command | one byte, `0x7F` | the status byte, and it is not `NONE` |

The fourth row is the one this app got wrong first, and it is the same mistake as
the metadata table's second authority in a different coat: `0x7F` is not
`AK_PROTO_RC_NONE`, and a parser that collapsed "I do not implement this
command" into "this board has no receiver port" would tell a person a *hardware*
fact about a board that may have a receiver plugged into it. `parseRcChannels`
raises on `0x7F` with the same sentence `parseParamInfoPage` does, and
`apps/configurator/tests/receiver.test.ts` pins all four rows.

**Sticks are decoded on the board and nowhere else.** `rc_min`, `rc_mid`,
`rc_max` and `rc_deadband` are parameters, and the reply carries the four stick
values already scaled to per-mille against them - roll, pitch and yaw over
-1000..1000 with zero inside the deadband, throttle over 0..1000. The app draws those
and the raw counts side by side and says which is which; it does not compute
either from the other. A second implementation of `ak_rc_decode`'s `centred()` in
TypeScript would agree with the firmware everywhere except at the deadband edge,
and the disagreement would be invisible - both numbers would look plausible. The
panel says so on screen, and names `ak_types.h` as the place the raw-channel-to-
stick mapping lives, because that mapping is not on the wire and the app is not
entitled to guess it.

**A sensors tab that is mostly a lesson in what "absent" means.** Milestone 6
added `AK_PROTO_CMD_SENSOR_INFO` (0x0E) under `AK_PROTO_FEATURE_SENSOR_INFO`,
and the web app's Sensors tab with it. One topic per request - imu, baro, range,
battery, gps - each with a body fixed per topic, so a reply's *length* is a check
a client can make rather than a description it has to trust. The largest is 59
bytes, so none of them can be a short frame.

The whole design is in the difference between two sentences this wire refuses to
merge:

| the board is saying | the bytes | who has to fix it |
| --- | --- | --- |
| this build does not answer for that topic | `AK_PROTO_SENSOR_NO_SUCH`, three bytes and stop | a firmware update |
| this build knows the question; this board has nothing fitted | `OK`, `present` clear, and **no body** | a socket |
| a reading, and a zero in it is a zero | `OK`, `present` set, then the body | nobody |
| a firmware that predates the command | one byte, `0x7F` | a firmware update, and a different sentence |

The middle two are the reason the body is left off rather than sent as zeros: a
body of zeros is a barometer reading zero pressure, which is a *failed* sensor,
and a screen that showed the same thing for a missing one would send somebody
looking for a driver when the part is simply not there. The fourth row is the
same mistake milestone 5's receiver had, and it is guarded the same way:
`parseSensorInfo` raises on `0x7F` with "this build predates `sensor info`" rather
than reading it as `NO_SUCH`, because `0x7F` is a claim about the build and a
board can carry that answer *and* the part.

Three fields are signed for the same reason: a rangefinder's `distance_mm` is
negative for "nothing in range" where a wall against the lens is 0; the battery's
`pin_mv` is negative for "no reading at the pin" where a divider reading 0 mV is
0; and `gps.home_distance_m` is negative when there is no home or no fix to
measure from. `gps.fix_type` is u-blox's own numbering, untranslated - MAVLink's
`GPS_FIX_TYPE` numbers the same ideas differently, so a table borrowed from the
other adapter would name every fix type wrong in a way that looks plausible. And
`baro.height_cm` is height above *the reference*, which is why `have_reference`
travels beside it: without one the number is meaningless rather than zero.

**The demo board got this wrong first, and a unit test is what caught it.** Its
sensor arm built the body correctly and returned it *without* the three-byte
status/topic/present header, so the first byte a client read as `status` was the
`i` of the driver name `icm42688p` - 0x69, and the app's own error sentence was
`status 105 is not one this protocol defines`. Nothing was wrong with the parser
or the firmware; the peer in the page was wrong, and `tests/sensor.test.ts` found
it because it drives the *shipped* demo board end to end rather than a hand-built
reply. That is the class of defect the demo board exists to make findable, and it
is worth naming: a hand-built payload would have agreed with the parser about a
layout neither of them had to write down twice.

**Sensors are polled as a round, and only while the tab is open.** All five topics
are read and then swapped in together with one arrival time, so a slow link
cannot draw a panel that is half of one moment and half of another; at 1 Hz, not
the receiver's 10, because a round is five round trips and the numbers are
pressures and voltages rather than a handset. A board whose capability word does
not claim the bit is not asked at all - five `0x7F` answers a second is five round
trips spent learning nothing - but the tab still *opens*, like the Receiver's, and
the panel reports the board's own answer, which is the more useful sentence. On
closing the link every reading is dropped: a driver name and a pack voltage left
on screen describe an aircraft the app is no longer hearing, and somebody would
go looking for a sensor that the *next* board does not have.

**No real sensor has answered any of this.** No board and no sensor: `AK_PROTO_SENSOR_OK`
with a body has been produced by the demo peer and by hand-built frames in
`tests/sensor.test.ts`, and never by silicon. The browser check visits the tab
against the demo board, which is a real protocol peer inside the page - so what is
proven is the app's end of the wire contract, and the board's end is proven only
by `firmware/tests/test_proto.c` and `tools/akproto_check.py`. A real IMU, baro,
rangefinder, pack or receiver has not been read back through this opcode.

**What was not run.** The browser: Web Serial needs a person to pick the port
from a list, and the handoff's §2 records the bridge path having been exercised
in headless Chrome against the WeAct. The app's *own* code against the Feather
was attempted through a node-side serial transport and got as far as
`EIO: i/o error, write` - because the board left the USB bus at that moment (no
`/dev/ttyACM*` twenty seconds later, and none since). So the table above is the
app's wire contract answered by real silicon, not the app itself driving this
board; the app's end-to-end path on this board is unproven and the run should be
repeated when a board is present.
