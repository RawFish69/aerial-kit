# AerialKit configurator

![AerialKit Rotor A](public/brand/aerialkit-selected.svg)

[Branding and logo assets](docs/BRANDING.md) · [Build and deployment](docs/BUILD-AND-DEPLOY.md)

A web page for looking at an AerialKit flight controller and changing its
parameters. It runs in a browser, talks the firmware's own config protocol, and
can be served as static files from anywhere.

A standalone component of the public Aerial Kit repository. See [Boundaries](#boundaries).

```sh
npm install
npm run dev              # http://localhost:5173
npm test                 # unit/integration suite; live SITL cases need AERIALKIT_SITL
npm run build            # static site in dist/
npm run verify:deploy    # build, serve over HTTPS at a subpath, check every asset
npm run verify:browser   # drive the built page in real Chrome (needs puppeteer-core)
```

For install/build/test/deployment in full, and the capability matrix — what the
app can do against which firmware over which transport, and how each cell was
verified — see [docs/BUILD-AND-DEPLOY.md](docs/BUILD-AND-DEPLOY.md).

## What it is for

Configuring a flight controller has one genuinely hard part, and it is not the
UI: **"the board said ok" is not the same sentence as "the aircraft changed."**
This app is built so that those never get merged.

Four facts are kept apart everywhere, in the code and on the screen:

| fact | who establishes it |
| --- | --- |
| **requested** | the person, by typing. Nothing has happened yet. |
| **echoed** | the wire. The board put the value in its parameter table. |
| **applied** | the board, in principle — but not on the wire. `null` today, and see below. |
| **persisted** | `param save`, and only that. |

A write that succeeds is reported as *echoed*. Whether the running firmware
then rebuilt what depends on it is a third fact, and the reply does not carry
it, so the app reports it as **not established** rather than picking an answer.
The sentence saying why sits under the row rather than in a manual.

## The armed gate

No write is sent while the board reports armed. That much is simple. The part
that matters is what happens when the app *does not know*:

- The armed state comes from status and telemetry frames. Nothing else.
- It goes stale after 2 s. A stale state becomes `unknown`, not `disarmed`.
- `unknown` closes the gate exactly like `armed` does, and the screen says so
  in a hatched banner rather than a quiet grey — a grey that reads as "probably
  fine" is the most dangerous thing this page could draw.
- The refusal is on the button's own `title`, so it is found by hovering as
  well as by reading.

The gate is enforced on the near side of the wire. Nothing is sent and then
rejected; the bytes never leave.

## Limitations it states about itself

Every open connection lists what it cannot establish, with the source file that
establishes the claim. The list is not hidden behind a disclosure and does not
only appear when something goes wrong.

- **A write's reply does not say whether the aircraft rebuilt for it.** The
  firmware *does* re-apply: `AK_PROTO_CMD_PARAM_SET` fires
  `ak_proto_io_t.on_change`, and `main.c` wires that to `parameters_changed()`,
  the same call the console's `set` makes. What the reply carries is a status
  byte and the table's own message, and nothing that reports the application
  fact — so a client cannot tell one firmware revision from another, and this
  app would rather say "not established" than read an answer out of a byte that
  does not hold one. See `src/session/limitations.ts`.
- **Setting a parameter is not gated on the aircraft's state; saving one is.**
  `save_parameters()` in `main.c` passes `ak_flight_config_writable(&flight)`
  into `ak_params_save`, and that returns true only while the aircraft is
  disarmed — the argument exists so no route can save without having asked.
  `AK_PROTO_CMD_PARAM_SET` has no such check, so a board that is armed would
  take a value over the config link. The gate you see on screen is this app's,
  and a different client would not have it.

**Both of those are now readings rather than memories.** The board's `hello`
carries a capability word — a `u32` of `AK_PROTO_FEATURE_*` bits appended after
the four fields the reply used to end at — and each limitation appears only when
the bit that would answer it is *clear*. The word is a property of the compiled
firmware, so a board that answers no optional command says so, and a board whose
`hello` ends before the field says something different: **`null` is not zero.**
`null` means "this firmware cannot say", and the app has separate wording for it,
because "this board has no capabilities" and "this board predates the question"
are different claims about an aircraft. `has()` therefore returns `false` for
`null`, and `reasonFor()` is the one place that knows the difference.

The word is also why the rail can be honest. A tab whose opcode the board does
not answer is still listed — disabled, with the sentence naming the opcode — so a
person sees "your firmware is older than this app" instead of a bare page. See
[the rail](#the-rail) below.

Both limitations were stated wrongly until 2026-09-30, and the earlier text is
worth recording rather than deleting: this list said the protocol had *no*
`on_change` field to fire and that neither `SET` nor `SAVE` checked flight
state. The first was false when it was written and stayed on screen for weeks,
because a paragraph about a symbol cannot be checked and nothing was checking
it. Every entry now carries a `citations` array of `path:symbol` pairs, and
`tools/check-citations.py` — part of `npm run verify:drift` — fails the build
when one stops resolving. Line numbers were replaced with symbol names for the
same reason: `ak_proto.c:211-247` stopped pointing at anything the day the file
changed, and no one noticed.
- **A board that cannot describe its parameters gets no ranges, and says so.**
  When the board's word lacks `PARAM_INFO` — or the word itself is absent — no
  parameter is shown with a range, a decimal count, a default or a help
  sentence. The names, indexes and values are still the board's own, read over
  `param get`; it is the *description* that is missing, and each row says
  `no description` rather than borrowing one. This app holds no fallback table,
  deliberately, and the entry names the two opcodes it asked for. See
  [the layout](#layout) for what that fallback cost.
- **A board with nowhere to save says so**, and the app stops claiming anything
  about persistence.
- **A console link cannot stream, and a refusal is a different fact from a
  promise.** The firmware answers a console subscribe with 0 by design: only the
  network link sets `can_stream` (`main.c`), because frames arriving among a
  person's keystrokes is a console nobody can use. The simulator does the same.
  Separately, `tools/akproto_sim.c` has no call to
  `ak_proto_telemetry_frame()`, so it could not push a frame even if it had
  agreed to. The app reports the 0 as the refusal it is; when a link *does*
  agree to a rate and then sends nothing, it records the agreement, waits,
  notices no frame came, and says that — rather than showing an empty graph.

## The rail

`src/ui/tabs.tsx` holds one array, `TABS`: the ordered list of sections, which
families each belongs to, and — for the ones that cannot be opened — the
sentence saying why. It replaced three `if` branches in `App.tsx` that had no
registry at all, which meant a panel appeared whether or not anything could back
it and a bare page gave no way to tell "your firmware is older than this app"
from "the app forgot to render something".

**A tab that cannot be backed is still in the rail, disabled, carrying the
reason.** There are two ways that happens, and they are different facts:

- **The board cannot.** Its capability word is missing the bit, or the word
  itself is absent. The sentence names the opcode, not the bit number: "this
  board's firmware does not answer `rc channels`" is something a person can go
  and search the firmware for. `reasonFor()` produces it, and it is the one
  place that distinguishes *predates the field* from *does not answer*.
- **This app cannot yet.** The opcode is specified in
  `aerialkit/docs/16-protocol.md` and the view was never written. The sentence
  says which half is missing, and appends the board's own answer when there is
  one, so a person with old firmware is not told to wait for a release that
  would not help them.

A disabled tab's `render` returns `null` and its `unavailable` function returns
the sentence; an enabled tab has a real `render` and no `unavailable`. That
pairing is the invariant, and it is why an enabled tab never opens onto nothing.

`docs/BUILD-AND-DEPLOY.md` carries the rail as a table, and
`tools/check-tabs.py` fails the build when the table and `TABS` disagree — the
labels, their order, and whether each opens. It deliberately does **not** compare
the reason text: those sentences live beside the code that decides them, and a
second copy in a table is a second thing to go stale.

## Firmware families

The same cable can carry an AerialKit board, a Betaflight board or a vehicle.
This app asks which before it does anything else, and the order it asks in is
not cosmetic — a flight controller's serial port carries the console *and* the
protocol, so a window that greeted every board with somebody else's protocol
would be typing at its own console.

| family | identity | live telemetry | parameters | writes | verified against |
| --- | --- | --- | --- | --- | --- |
| **AerialKit** | yes | yes, streamed | yes, the whole table | yes, gated | the firmware's own simulator, over the bridge |
| **Betaflight / INAV** | yes | yes, polled | **by name only** | **no — read-only** | captured frames from this repo's Betaflight stand-in; *no physical board* |
| **ArduPilot / PX4** | yes | yes, streamed | yes, the whole table | **no — read-only, one exception** | `aerialkit/tools/mavlink_fake_vehicle.py`, which encodes and decodes every frame with pymavlink; *no physical vehicle, and not ArduPilot SITL* |

**A link is called MAVLink only when a frame on it passed its checksum.** A
MAVLink checksum needs a constant per message id that is not on the wire, so
three bytes that look like a header prove nothing; a stray `0xfd` in console
noise stays *unrecognised*. A stream that checksums but carries no heartbeat is
reported as exactly that, rather than rounded to a vehicle.

**The MAVLink client is read-only, and that is a property of the type.**
`MavBoard` has no method that could arm, disarm, change a flight mode, upload a
mission or write a parameter, and a test asserts the *complete* list of methods
on it, so one added later cannot go unnoticed. It sends exactly two messages: a
parameter-list request, and a telemetry-rate request. The second is a write, and
the page, the code and the limitations list all say so.

Two things about the MSP column are worth stating plainly:

- **There is no "list the settings" request in MSP.** `MSP2_CLI_SETTING` takes a
  *name* and answers the text `name = value`; a ground station that shows a table
  ships its own list of names per release. So on a foreign board this app has a
  box to type a name in, not a table.
- **Read-only is enforced by the type, not by a flag.** `MspBoard` has no
  `write`, `edit`, `save` or `stream` method to call, and a test asserts it —
  a method added later by someone in a hurry would otherwise go unnoticed.

Detection costs one deadline when it misses: a board that speaks MSP answers our
hello with silence, so the AerialKit probe has to time out before MSP is tried
(~600 ms by default). That is the price of asking in the safe order.

## Transports

| | what it is | needs |
| --- | --- | --- |
| **Demo board** | a simulated AerialKit inside the page | nothing |
| **USB serial** | a board on a cable, via Web Serial | Chrome/Edge/Opera |
| **Local bridge** | the firmware simulator, or a port something else holds | `npm run bridge:setup` |

The demo board is a protocol peer, not a mock of the UI: it produces frames the
same decoder parses and answers commands the same client sends. Two rules stop
it passing for hardware, and both are enforced by tests — it answers to
`aerialkit-demo` rather than the captured `aerialkit-f405`, and it boots
*disarmed* where the firmware's simulator boots armed, so the gate can be seen
working.

Connecting is always a click. The app never opens a device because a page
loaded.

### The bridge

A browser cannot open a TCP socket or spawn a process. The bridge is a helper
that does both and relays bytes:

```sh
npm run bridge:setup                       # once; installs ws
npm run bridge:sim                         # the firmware simulator
npm run bridge -- --serial /dev/ttyACM0    # a port something else holds
npm run bridge -- --tcp 192.168.1.50:5760  # a board on the network
```

It does **not** parse the protocol and must never learn to. The moment it
interprets frames there are two implementations of the protocol and one is
always behind.

Two things it enforces rather than documents:

- **Loopback only.** It binds `127.0.0.1`. On a network, an unauthenticated byte
  pipe into a flight controller is a remote control for the aircraft on your
  desk. Binding elsewhere requires `--expose` and prints what that means.
- **It checks `Origin`.** WebSocket connections are not subject to the
  same-origin policy, so without this check any page a person visits could open
  `ws://127.0.0.1:8787` and start writing. Only pages served from this machine
  get through; a caller with no `Origin` at all is not a browser.

One client at a time, and that is not a limitation to work around: the protocol
correlates a reply to its request by the command byte alone, so two clients
writing at once would each receive the other's answers.

## Layout

```
src/protocol/    frames, CRC, message parsing, the request client, MSP, MAVLink, detection,
                 features.ts — the capability word and `has()`/`reasonFor()`
src/transport/   serial, bridge, demo board, and the Transport seam
src/session/     the four-fact model, the armed gate, the limitation list, foreign boards,
                 the read-only MAVLink board
src/ui/          the workspace: App.tsx, tabs.tsx — the rail registry — and the panels
src/firmware/    demo-table.json, the demo board's starting table — a fixture, never a source
tools/           capture-fixtures.py, capture-mavlink-fixtures.py, ak_table.py,
                 check-citations.py, check-table-drift.py, check-tabs.py,
                 verify-drift.sh, verify-deploy.sh, verify-browser.sh, browser-check.mjs
bridge/          the local helper — its own package, its own dependencies
docs/            BUILD-AND-DEPLOY.md — install, build, deploy, the rail as a table
tests/           protocol, session and UI tests across the protocols, the sessions and the interface,
                 and 10 more against a live ArduPlane SITL that are skipped unless
                 AERIALKIT_SITL names one
```

`src/firmware/demo-table.json` is a **fixture**, not a source. It is the table
the demo board is handed at boot, which it then serves back over `param info`
and `param help` — so even the demo's ranges come off the wire.

A real board's description is read the same way. **The range, help text, decimal
count and group of every parameter are carried by the protocol**, in
`AK_PROTO_CMD_PARAM_INFO` (0x0A) and `AK_PROTO_CMD_PARAM_HELP` (0x0B), added
2026-09-30 under the `AK_PROTO_FEATURE_PARAM_INFO` capability bit
(`aerialkit/docs/16-protocol.md`). An entry is a name, a type, a group, a
decimal count and a flags byte, then either two bounds spelled as text or a
maximum text length, then the default — nine fields with no room in them for a
unit, which is why there is no unit column. `AerialKitSession.refresh()` walks
the table by index after the value walk and keys every description to its index,
never to its name.

Until that date this app instead held a build-time snapshot of the firmware's
source in `src/firmware/parameter-table.json` and showed it beside whatever
board answered. It had gone 32 rows stale against a 33-row flight table —
`arm_accel_lpf_hz` was missing and every row after it was displaced by one — and
nothing failed for a release, because every consumer joined the file to the
board's reply by name and a name the file lacked produced a row with *no* range,
which reads exactly like a parameter that has none. Keeping a second authority
is what made that possible; there is no fallback table now. A row the board does
not describe says `no description`, and the connection says why.

## Boundaries

- **Not part of the NAS console.** This is a standalone app with its own
  dependencies, its own tests and its own static build. It reads nothing from
  the NAS, writes nothing to it, and shares no code with the console. It is
  published alongside the flight-controller firmware in the public Aerial Kit repository.
- **Nothing is deployed.** No public host, no port, no secret in the bundle.
  `dist/` is built locally and served from a file or a dev server.
- **No secrets, and nothing to leak.** There is no credential anywhere in this
  app: Web Serial's chooser is the permission boundary, and the bridge is
  loopback-only.
- **Subpath deploys** are a build-time decision, not a hardcoded `/`:
  `AK_BASE=/aerialkit/configurator/ npm run build`. Verified: that command emits
  `src="/aerialkit/configurator/assets/index-*.js"`, and the default build emits
  `src="/assets/index-*.js"`. The environment variable is the only mechanism —
  there is no `build:subpath` script, because there used to be one that ignored
  `AK_BASE` and produced a build that 404s on its own assets under the subpath
  its name implied. See
  [docs/BUILD-AND-DEPLOY.md](docs/BUILD-AND-DEPLOY.md#deploying-under-a-subpath).

## Licensing

GPL-3.0-or-later, matching the firmware. Both protocol implementations here are
written from the wire documentation and from captured frames, not copied from
any firmware's C. The MSP decoders are pinned by tests against frames captured
from this repository's own Betaflight stand-in (`tools/capture-msp-fixtures.py`),
which is itself written from Betaflight's source; the test that reads those
fixtures back does its own framing longhand, so a mistake in either reader shows
up as a checksum that does not match rather than as a test that passes for the
wrong reason.

The MAVLink implementation is written the same way, from the wire documentation,
and is pinned the same way and more strongly. Every message definition in it —
field order, field type, array length, and the `crc_extra` constant that is not
on the wire — is compared against **pymavlink**, the reference implementation
that Mission Planner, QGroundControl and MAVProxy are built on, by
`aerialkit/tools/mavlink_check.py`. The frames it builds are compared byte for
byte against pymavlink's, apart from v2's trailing-zero truncation, which is
pymavlink's own behaviour.

**What that reference is not.** ArduPilot SITL is the usual reference for a
MAVLink client, and this is the one place in the repository where the gap is a
permission rather than an absence: **an ArduPlane SITL binary has been built on
this machine** (`./waf configure --board sitl` and the build both succeeded), so
"it is not installed here" would be false. What has not happened is a **run**.
`tests/sitl.test.ts` holds the ten semantic tests written to use it, and they
are skipped by name unless `AERIALKIT_SITL` names a vehicle — so they report as
*skipped*, never as *passing*. **Nothing in this repository is verified against
real ArduPilot firmware.** The stand-in used instead is
`aerialkit/tools/mavlink_fake_vehicle.py`, which encodes every frame
it sends with pymavlink and decodes every byte it receives with pymavlink — so
the client is still being checked against the reference implementation's bytes,
but against a simulated vehicle rather than against ArduPilot's own firmware.
Every claim about "an ArduPilot vehicle" in this repository means that stand-in.

## Reviewing configuration changes

The header separates edits staged in this page from board values not yet saved
in flash. **Review edits** opens the staged rows from any section. Search and
**Group** use the descriptions read from the board; filtering never changes the
scope of **Send all staged** or **Discard all staged**. Discarding staged edits
changes only this page.

Send or discard staged edits before **Save to flash**. Saving persists only
values already on the board. The usual armed-state and freshness gate still
applies. The demo is labelled throughout and resets on disconnect.

USB is selected initially when the browser supports it; **Explore demo** is an
explicit alternative. **Why unavailable?** below the section list makes every
disabled section's reason available to keyboard and touch users.
