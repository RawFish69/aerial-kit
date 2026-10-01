> **Current source scope:** runtime code is in `firmware/`, with Feather F405 and ESP32DEV profiles. The public tree excludes the C host simulator; simulator-based bridge/SITL examples require an external build. See the [current guide](../../../docs/web-configurator.md).

> **Public export verification, 2026-10-01:** the current source passed 285 configurator tests (10 live SITL cases skipped), production/subpath builds, all source/table/dialect drift checks, and HTTPS subpath asset checks. The dated observations below are retained as historical records. See [publication results](../../../docs/PUBLICATION.md).

# Building, testing and deploying the configurator

This is the install/build/test/deployment half of assessment item **W4**
(`AERIAL-KIT-ASSESSMENT-2026-09-18.md:815`), which asks for "install/build/test/
deployment instructions plus the capability matrix" and a release-ready artifact.
The [capability matrix](#capability-matrix) is below.

Every number in this document was produced by the command printed next to it, on
this machine, on 2026-09-19. Anything that could not be run here is labelled
**not run** and says why. That distinction is the point of the document: a
deployment guide that quietly omits what it never checked is how a release
acquires a rumour.

## Requirements

| | version used here | why it matters |
| --- | --- | --- |
| Node.js | v18.19.1 | Vite 5 and Vitest 2 need ≥18 |
| npm | 9.2.0 | ships with the above |
| A browser | — | only to *use* the app; building and testing need none |

`npm install` at the package root installs everything the app and its tests need.
The bridge is a **separate package with its own dependencies** and is not
installed by that command — see [the bridge](#the-bridge-is-a-second-package).

```sh
cd apps/configurator
npm install
```

## Build

```sh
npm run build      # tsc --noEmit && vite build
```

Measured:

```
dist/index.html                     1.12 kB │ gzip:  0.59 kB
dist/assets/index-DIZwrI-L.css     12.13 kB │ gzip:  3.01 kB
dist/assets/index-BUmvpv7J.js     218.83 kB │ gzip: 68.53 kB │ map: 615.13 kB
✓ built in 1.75s
```

(exit 0. The `map` is the source map — `build.sourcemap` is on in
`vite.config.ts`, deliberately: a stack trace from a bench session is worth more
than the bytes. Turn it off for a public deploy if that is the trade you want,
but it is a decision, not a default that got left on.)

`npm run build` runs the typecheck first, so a build that succeeds is also a
build that typechecks. The output in `dist/` is a static site: three files and a
source map, no server, no runtime dependency on Node.

### Deploying under a subpath

Asset URLs are a **build-time** decision, not a hardcoded `/`:

```sh
AK_BASE=/aerialkit/configurator/ npm run build
```

`vite.config.ts` normalizes the value (forcing a leading and trailing slash), so
`AK_BASE=aerialkit/configurator` and `AK_BASE=/aerialkit/configurator/` produce
the same build. Verified, both directions:

| build | `index.html` emits |
| --- | --- |
| `npm run build` | `src="/assets/index-BUmvpv7J.js"` |
| `AK_BASE=/aerialkit/configurator/ npm run build` | `src="/aerialkit/configurator/assets/index-BUmvpv7J.js"` |

**There is deliberately no `build:subpath` script.** There used to be one, and it
was byte-identical to `build` — so running it with no `AK_BASE` set produced a
root-absolute build that 404s on its own assets under the subpath its name
implied. Measured, serving that build with the site mounted at the subpath:

```
  index                    -> 200
  js (/assets/index-*.js)  -> 404
```

A script whose name promises something its body does not do is worse than no
script: it is a deploy that looks deliberate and fails in the browser console.
The real mechanism is the environment variable, and it is the only one.

## Test

```sh
npm test           # vitest run — measured: 192 passed, 10 skipped, 13 files, exit 0
npm run typecheck  # tsc --noEmit — measured: exit 0
npm run test:watch # the same suite, watching
```

The count is not a milestone number, it is what `npm test` printed. It has been
wrong in this line more than once for the same reason each time — the paragraph
insisting it matched the command was itself the stale part — so the tally is
not repeated here; it only becomes another thing to keep current. The command is
the authority, this line is a convenience copy of its output, and
`make configurator-check` prints the same two counts on every run, which is
where a reader who needs them now should look.

The 10 skips are `tests/sitl.test.ts`, which needs a live SITL and says so in
its own output rather than passing quietly.

**The count is no longer the only thing standing between this suite and a green
run that means less than it says.** `make configurator-check`, which the CI
pipeline runs as its `configurator` stage, runs these same two commands in this
directory and then checks the one thing a count cannot express: that the number
of test files vitest *collected* equals the number of test files *on disk*. A
file that exists and is matched by nothing — a renamed glob, an `include`
pattern that stopped covering `.test.tsx` — is invisible in a green summary and
visible there. The gate does not assert a fixed number, precisely because of the
paragraph above: it prints the counts and fails on the disagreement.

### The checks that are not the suite

```sh
npm run verify:drift   # do the app's claims about the firmware still hold?
npm run verify:deploy  # does the built artifact work from the subpath it is built for?
npm run verify:browser # real Chrome, the whole connect/disconnect/reconnect workflow
```

`verify:drift` is three checks that all exist because the two halves drifted and
nothing noticed: `check-citations.py` resolves every `path:symbol` the app cites
about the firmware, `check-table-drift.py` compares the stored parameter table
against `ak_flight.c` and the board, and `check-dialect.py` compares this app's
hand-written MAVLink definitions against the published dialect.

**It exits 1 on this machine today, for two named reasons, and is deliberately
not wired into the CI stage yet.** Reporting either as a pass would be the class
of lie the rest of this directory exists to prevent:

- `check-table-drift.py` fails. As measured on 2026-09-30, the stored table has
  32 rows against **33** in `ak_flight.c`, and **92** on the board — the check
  compares against `ak_flight.c`, which is where the file's `captured_from`
  points and which is the *flight* table only. (The 92 is measured, not
  assumed: `python3 firmware/tools/akproto_firmware_check.py` prints
  `and says how many parameters it has 92 parameters` against
  `aerialkit-fw-sim 0 console`.) Against `ak_flight.c` the file is missing
  `arm_accel_lpf_hz` at index 13, so every row from there on is displaced by
  one. Nothing in the app breaks from this, because every
  consumer joins on `name`, which is exactly why it survived. Milestone 3
  replaces the file with metadata served by the board over `PARAM_INFO` and
  demotes it to the demo board's starting state; that is what turns this green.
  Regenerating the file is *not* the fix and is not a substitute for it.
- `check-dialect.py` cannot run here: it needs `pymavlink`, and the scratch
  environment that had it was cleaned up on 2026-09-30. A check that could not
  run is not a check that passed.

Both belong in the stage once those are true. Until then, running
`npm run verify:drift` by hand is the way to see the measurement.

## Deploy

The artifact is `dist/`. Any static file server can serve it; nothing in it needs
Node, a database, or a writable filesystem.

### What was actually run

W4 asks for verification "on the intended HTTPS hosting configuration". No public
host exists and none may be created (the goal's standing rule: *no public port, no
public deployment*), so what was verified is the configuration itself, on
loopback: the built site, over **HTTPS**, mounted at a **subpath**, fetched the
way a browser would fetch it.

```sh
./tools/verify-deploy.sh                      # exit 0 when the deploy is sound
./tools/verify-deploy.sh /some/other/subpath  # any mount point
```

The script builds for the subpath, lays the output out as a host mounting the app
at that path would, generates a throwaway certificate, serves it over TLS on
`127.0.0.1` (the stdlib `http.server` has no TLS flag, so its listening socket is
wrapped), and asserts each response. Measured, all checks passing:

```
  the js URL is under /aerialkit/configurator/     /aerialkit/configurator/assets/index-BUmvpv7J.js
  the css URL is under /aerialkit/configurator/    /aerialkit/configurator/assets/index-DIZwrI-L.css
  index                                            200
  javascript                                       200   text/javascript  218909 bytes
  stylesheet                                       200   text/css          12125 bytes
  scheme                                           HTTPS
  root-absolute /assets/... (demonstration)        404
```

**The script was mutation-checked**, so it is a check and not a green light:
making `normalizeBase()` always return `/` — reintroducing exactly the
root-absolute bug — produces

```
  FAIL  js URL is not under /aerialkit/configurator/
  FAIL  javascript            expected 200, got 404
  FAIL  js content-type       expected text/javascript, got text/html;charset=utf-8
  ...exit 1
```

One line of that output is a **demonstration and not a guard**: the root-absolute
probe 404s whether or not the build is correct, because nothing is mounted at
this server's root. It is printed so the shape of the mistake is visible; the
checks that actually catch it are the two URL assertions.

Three things this establishes and one it does not:

- The subpath build resolves **all** of its own assets under the subpath — the
  index, the script and the stylesheet, each with a correct `Content-Type`.
- The root-absolute build genuinely fails there (404), which is why the
  environment variable is not optional when deploying under a subpath.
- TLS was negotiated rather than plaintext.
- It does **not** establish anything about a real certificate, a real host, or
  HSTS. A self-signed cert on loopback proves the *page and its assets work over
  HTTPS*; it proves nothing about a CA, a redirect, or a header policy. Those are
  properties of a host that does not exist yet.

### Release-ready artifact

`dist/` from `npm run build` **is** the release artifact. It is reproducible from
a clean checkout with the two commands above, it carries no secret (there is no
credential anywhere in this app — see [Boundaries](../README.md#boundaries)), and
it is self-contained. Nothing was published, and nothing may be without the
owner's specific authorization at the time.

### Browser support

Web Serial is the only transport that reaches a board with no companion
installed, and it is the browser-dependent part:

| browser | Web Serial | the page itself |
| --- | --- | --- |
| Chrome, Edge, Opera (desktop) | yes | yes |
| Firefox, Safari | **no** | yes — demo board and bridge work |
| any mobile browser | **no** | yes — demo board and bridge work |

Only the **Chrome** row was observed. The Firefox/Safari/mobile rows are the same
capability fact (`'serial' in navigator` is false) and were exercised by removing
that API in Chrome — a statement about the app's behaviour when the API is
missing, not about those browsers. The measurements are below.

The gate is one runtime check, `serialSupported()` in `src/transport/types.ts`
(`'serial' in navigator`).

This was verified in a **real browser**, not inferred:

```sh
npm install --no-save puppeteer-core   # see below — deliberately not a dependency
./tools/verify-browser.sh              # builds, serves over HTTPS, drives Chrome
```

Measured, Chrome 152.0.7977.82 against the subpath build over HTTPS:

```
  ok  the page loaded and has a title          "AerialKit Configurator"
  ok  no failed requests
  ok  no uncaught page errors on load
  ok  the page is a secure context             (Web Serial requires one)
  ok  every script came from the subpath it was built for
  ok  the browser has Web Serial
  ok  a Connect button is present
  ok  the demo board identified itself
  ok  a parameter was read from it
  ok  a Disconnect button is present
  ok  it is back to the connect form
  ok  Connect is offered again
  ok  it reconnected and re-read the board
  ok  no console errors across the whole workflow
```

That is W4's **connect / disconnect / reconnect** workflow, run against the demo
board — a protocol peer inside the page, so **no cable and no physical board are
involved**, and none is claimed.

The browser without Web Serial was exercised as well, by deleting
`Navigator.prototype.serial` before the app loads, which is the state a Firefox
or Safari user is in:

```
  ok  'serial' in navigator                    false
  ok  the USB serial option is still shown, not hidden
  ok  it is marked unavailable                 "USB serial (not available in this browser)"
  ok  and it cannot be chosen
  ok  the demo board is still offered when the cable cannot be
  ok  the bridge is still offered when the cable cannot be
  ok  Connect is still usable for the transports that do work
```

Note what that check asserts, because the first version of it asserted something
else and was wrong: the "This browser has no Web Serial…" notice in the UI is
**unreachable from this state**. It renders only when the chosen transport *is*
`serial`, and the only way to choose that is the dropdown option that is disabled
precisely when the notice would apply. The app's real behaviour — option visible,
marked unavailable, disabled — is better than the assertion, and is what is
tested. The notice itself is covered where it is reachable, in
`tests/ui.test.tsx:307`. Whether it should be reachable at all is a UI question,
not a defect, and it is left as it is.

Both modes were **mutation-checked**: aimed at the root-absolute build served
under the subpath, the run exits 1 and names the failures —

```
  FAIL  no failed requests   https://…/assets/index-BUmvpv7J.js — net::ERR_ABORTED
  FAIL  every script came from the subpath it was built for   /assets/index-BUmvpv7J.js
  FAIL  a Connect button is present
```

### A vehicle that is not ours, in a real browser

The third mode is the one where the bytes the page reads were built by something
other than this repository. `./tools/verify-browser.sh` now runs a fourth pass:

```sh
# what the script does for this mode, spelled out
node bridge/index.js --port 8791 --stdio python3 \
    ../../../firmware/tools/mavlink_fake_vehicle.py --arm-after 10
node tools/browser-check.mjs "$URL" mavlink
```

`mavlink_fake_vehicle.py` encodes every frame it sends with pymavlink and decodes
every byte it receives with pymavlink, so what is under test is the page against
the reference implementation. Measured, Chrome 152.0.7977.82:

```
  ok  the bridge address field took what was typed into it   ws://127.0.0.1:8791/ak
  ok  the page says which autopilot answered                 from the heartbeat
  ok  and what airframe it is
  ok  and which framing is on the wire, not the version field
  ok  the armed state is read from a heartbeat
  ok  and the vehicle is reported disarmed
  ok  the detection line says it is read-only
  ok  no button on the page could change the vehicle
  ok  and every control on it is one of the seven it is meant to have
  ok  the page states what it cannot do                      MAV_LIMITATIONS
  ok  the live panel is empty before anything is asked for
  ok  a rate button is offered
  ok  the vehicle answered the rate request with an ack      COMMAND_ACK
  ok  and the ack says it was accepted
  ok  the vehicle is now streaming attitude
  ok  and position
  ok  and battery
  ok  and GPS
  ok  the trace says what was asked for
  ok  a Read parameters button is present
  ok  the whole table came back
  ok  all forty, with the last one closing the list
  ok  the vehicle armed, and the page followed it
  ok  the change was reported rather than absorbed
  ok  a Disconnect button is present
  ok  it is back to the connect form
  ok  Connect is offered again
  ok  it reconnected to the vehicle
  ok  and re-detected it as MAVLink rather than assuming
  ok  no console errors across the whole workflow
== mavlink: all checks passed
```

Three things about that list are the point of it:

- **The absence checks are a whitelist, not a blacklist.** "No button on the page
  could change the vehicle" scans for arm/disarm/takeoff/mission/upload/mode/write
  verbs, and can therefore only fail for a control somebody thought to name. The
  check after it lists *every* button and requires each to be one of the seven
  the page is supposed to have, so an "Advanced" or a "Calibrate" added later
  fails here.
- **The rate request is a real write, all the way through.** The page's frame
  goes over the WebSocket, through the bridge, into pymavlink's decoder, and the
  `COMMAND_ACK` that comes back was encoded by pymavlink. The live panel is
  asserted to be *empty* first, because the stand-in streams nothing until it is
  asked — which is what a real vehicle does.
- **The armed transition is the only check that needs time**, so the stand-in
  arms itself at t+10 s and the page has to follow it: the banner flips, and the
  trace has to say *the vehicle is now ARMED* rather than silently re-rendering.
  It is polled to a deadline, so a slow machine fails rather than passes.

**Two defects were found by this run and fixed**, both in the stand-in rather
than in the app, and both invisible to every test that existed:

1. `mavlink_fake_vehicle.py` set `base_mode = 1` for armed and commented it
   `MAV_MODE_FLAG_SAFETY_ARMED`. That constant is **128**; 1 is
   `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`. Nothing had ever asked the stand-in
   whether it was armed — `tools/mavlink_check.py` did not check arming, and
   neither client it drives reads `base_mode` — so the browser check reported an
   armed vehicle as disarmed, and the check *above* it ("and the vehicle is
   reported disarmed") was passing because the bit was wrong rather than because
   the vehicle was disarmed.
2. The stand-in never sent `COMMAND_ACK`, so the page's ack display — the
   thing that distinguishes "the vehicle accepted that" from "the vehicle never
   heard it" — could not be exercised against it at all. It now acks accepted
   commands with `MAV_RESULT_ACCEPTED` and unimplemented ones with
   `MAV_RESULT_UNSUPPORTED`.

Both are now checked where they belong, which is **not** the browser check: a
defect that only Chrome on a machine with `puppeteer-core` can see is one that
comes back. `tools/mavlink_check.py` gained a section that drives the stand-in
with `--arm-after 0.4` and asserts, against pymavlink's own constants, that the
armed flag is the bit that rises and that both a carried-out and an unimplemented
command come back acked. They read, on 2026-09-30:

```
  ok  the stand-in arms by raising the armed flag, not by setting some other bit (base_mode [0] before, [128] after)
  ok  and it acks a command it carries out, and one it does not ([(511, 0), (176, 3)])
```

and each was falsified on its own: restoring `base_mode = 1` reddens the first
and leaves the second green; deleting the `self.ack(...)` call reddens the second
and leaves the first green.

**Firefox: not run.** `/usr/bin/firefox` 156.0 exists on this machine but will
not launch headless — it fails under puppeteer *and* standalone
(`firefox --headless --screenshot` produces no file), which is a property of the
packaged browser on this box, not of the app. `verify-browser.sh` attempts it,
and reports `could not launch firefox … nothing was tested` with exit code 3,
which the wrapper does not count as a pass or as a failure. The
`chrome-noserial` mode above covers the *capability* branch; it says nothing
about Firefox's rendering, layout or event handling.

**`puppeteer-core` is deliberately not in `package.json`.** Version 25 requires
Node ≥ 20 and this package targets Node 18 (measured: `npm install --no-save
puppeteer-core` warns `EBADENGINE required: ^20.19.0 || ^22.12.0 || >=23, current:
v18.19.1`). Adding it would make `npm install` warn or fail for everyone, to
support a check most runs do not need. `npm install --no-save` keeps both
`package.json` and the lockfile untouched. When this package moves to Node 20+,
the right fix is to add it properly.

## Capability matrix

What the app can do, against what, and how each cell was established. The
verification column is the honest part: **`captured frames` and `simulator` are
not `hardware`.**

### By firmware family

| capability | AerialKit | Betaflight / INAV (MSP) | ArduPilot / PX4 |
| --- | --- | --- | --- |
| **identified** | yes | yes | yes — from a heartbeat, and only from one that checksums |
| **live telemetry** | yes, streamed | yes, polled | yes, streamed |
| **parameters listed** | yes, the whole table | **no — by name only** | yes, the whole table |
| **parameter write** | yes, gated | **no — read-only** | **no — read-only** |
| **parameter save** | yes, when the board has flash | no such command | **no such message is sent** |
| **parameter reset to build defaults** | yes — `param default` (0x0C), refused while armed, and refused on a board that predates the opcode | no such command | no such message is sent |
| **armed state** | yes, 2 s staleness bound | yes, 2 s staleness bound | yes, 2 s staleness bound, from the heartbeat |
| **telemetry rate change** | yes | n/a | **yes — the one write this page can make** |
| **verified against** | firmware's own simulator, over the bridge | captured frames from this repo's Betaflight stand-in — **no physical board** | pymavlink-encoded frames from `firmware/tools/mavlink_fake_vehicle.py`, driven in Chrome over the bridge — **no physical vehicle, and not ArduPilot SITL** |

Six notes the table cannot carry:

- **Every write on an AerialKit board is gated twice, and the two are not
  redundant.** The app refuses locally while its last reading says armed, so the
  common case costs no round trip and the person gets an answer immediately. The
  board refuses again, per request, against its own flight state — which is the
  only gate that covers the race the near side cannot: the aircraft can arm in
  the window between the app's last heartbeat and the bytes arriving. The two
  refusals carry the *same* status (5) and are told apart by the sentence beside
  it, `'it was never sent'` against `'the board refused it'`; a person reading
  "refused" needs to know which side said so, and a test asserts both.

- **The write column is not a policy that could be relaxed.** `MspBoard` has no
  `write`, `edit`, `save` or `stream` method to call; read-only is a property of
  the type, and a test asserts the methods are absent. A method added later by
  someone in a hurry would fail that test rather than silently reach an aircraft.
- **`MavBoard` is read-only the same way, and its test is stronger.** It asserts
  the *complete* list of method names on the prototype, so a method that is
  neither expected nor forbidden still fails. The one write the page can make —
  a `MAV_CMD_SET_MESSAGE_INTERVAL` telemetry-rate request — is named as a write
  in the limitations list, in the code, and on the panel that offers it.
- **MAVLink is never probed.** A vehicle heartbeats unasked, so detection listens
  passively and sends nothing until a heartbeat supplies a target system id. A
  link is called MAVLink only when a frame on it passed a checksum that needs a
  per-message constant not present on the wire — so console noise containing
  `0xfd` stays `unrecognised`, and a checksum-valid stream with no heartbeat is
  reported as exactly that rather than rounded to a vehicle.
- **MSP has no "list the settings" request.** `MSP2_CLI_SETTING` takes a *name*
  and answers `name = value`; a ground station showing a table ships its own list
  of names per release. So on a foreign board there is a box to type a name into,
  not a table — that is the protocol, not an unfinished feature.
- **`unrecognised` is a real answer, not a failure to try.** The detector sends
  the AerialKit hello, then MSP, and **only listens** for MAVLink. A link that
  answers none of the three is reported with the first byte that arrived so a
  person can see what is talking. Detection order is load-bearing: a flight controller's serial port
  carries the console *and* the protocol, so greeting every board with somebody
  else's protocol would be typing at its own console, and on a board whose console
  takes commands, typing is doing.

### By tab

The rail is `src/ui/tabs.tsx`. **A tab that cannot be backed is still in the
list, disabled, carrying the reason** — so a person looking at a page that
seems bare can tell "your firmware is older than this app" from "the app forgot
to draw something". The reason names the opcode, never the capability bit.

Two different things make a tab unopenable, and the registry says which: the
board cannot answer the opcode, or this app has no view for it yet. The second
is why a tab is listed here as `no` even though the wire side is specified —
an enabled tab whose view does not exist would open onto nothing and read as a
firmware fault.

`tools/check-tabs.py` compares this table against the registry, in order, and
fails on any disagreement.

<!-- tabs:begin -->
| Tab | Opens | Why not, when it does not |
| --- | --- | --- |
| Attitude | yes | — (STATUS is in every protocol version; the 3D view, horizon and trace poll it at 20 Hz while open) |
| Live | yes | — |
| Parameters | yes | — (search, not-default and staged filters, back up to and restore from a JSON file; a restore only stages) |
| Identity | yes | — |
| Diagnostics | yes | — |
| Receiver | yes | — (opens for any board: a firmware without `rc channels` (0x0D) answers `0x7F` and the panel says so) |
| Sensors | yes | — (opens for any board, like the Receiver: a firmware without `sensor info` (0x0E) answers `0x7F` and the panel says so) |
| Motors | yes | — (live outputs from STATUS; motor test needs `output info` (0x0F) and a test opcode on both sides) |
| Blackbox | no | needs no firmware change — `log get` already carries the records — so only the viewer is missing |
| Backup | no | the parameter file lives in Parameters; replaying the firmware's own `name=value` lines is not built |
| Preflight | no | needs `preflight` (0x12) on both sides |
| Mission | no | needs `mission` (0x14) on both sides |
| Firmware | no | the informational panel: what the board reported, and why flashing is refused |
<!-- tabs:end -->

### By transport

| capability | Demo board | USB serial (Web Serial) | Local bridge |
| --- | --- | --- | --- |
| **needs** | nothing | Chrome/Edge/Opera desktop | `npm run bridge:setup` |
| **reaches** | a protocol peer inside the page | a board on a cable | the firmware simulator, a held port, or a board on a network |
| **user gesture required** | no | **yes** — `requestPort()` is refused from a timer or mount | no |
| **writing possible** | yes, gated | yes, gated | yes, gated |
| **verified against** | the interface suite, `tests/ui.test.tsx` | **not run — no board on this machine** | the firmware simulator, end to end |

The demo board is a protocol peer, not a mock of the interface: it produces
frames the same decoder parses and answers commands the same client sends. Two
rules stop it passing for hardware, both enforced by tests — it answers to
`aerialkit-demo` rather than the captured `aerialkit-f405`, and it boots
*disarmed* where the firmware's simulator boots armed, so the gate can be seen
working rather than assumed.

**Web Serial is the one transport with no end-to-end verification**, and it is
the one that touches real hardware. Its logic is covered by unit tests against a
stubbed port; the chooser, the cable and the real board are not. This is the
honest state of W4's hardware half, and it is why the item stays open until a
bench session.

### The bridge is a second package

The bridge has its own `package.json` and its own dependencies (`ws`,
`serialport`) and is **not** installed by the root `npm install`:

```sh
npm run bridge:setup                       # once; npm --prefix bridge install
npm run bridge:sim                         # the firmware simulator
npm run bridge -- --serial /dev/ttyACM0    # a port something else holds
npm run bridge -- --tcp 192.168.1.50:5760  # a board on the network
```

It relays bytes and does not parse the protocol; the moment it interprets frames
there are two implementations of the protocol and one of them is always behind.
Two properties it enforces rather than documents: it binds **loopback only**
(`--expose` is required to do otherwise, and prints what that means), and it
**checks `Origin`**, because WebSocket connections are not subject to the
same-origin policy and without that check any page a person visits could open
`ws://127.0.0.1:8787` and start writing. `BridgeTransport` refuses a non-loopback
URL at construction for the same reason.

## What is not verified

Stated as a list, because each one is a way this document could be read as
promising more than it does:

1. **No physical board was connected** — not over Web Serial, not over the
   bridge. The Chrome and chrome-noserial workflows run against the demo board,
   which is a protocol peer inside the page; the MAVLink workflow runs against a
   Python stand-in. The Betaflight/INAV column of the matrix is verified against
   captured frames.
2. **No serial chooser was ever opened**, so the *permission* half of W4's
   "connect/disconnect/reconnect and permission workflows" is untested. It needs
   a board on a cable and a person to click through the picker.
3. **No Firefox, and no mobile browser.** Chrome 152 on Linux is the only
   browser any of this ran in. Firefox will not launch headless on this machine
   (see above); nothing was tried on a phone.
4. **No public deployment exists**, and the HTTPS check was loopback with a
   self-signed certificate. Nothing was checked about a CA, a redirect, or HSTS.
5. **No real autopilot was ever on the other end of the MAVLink client.**
   ArduPilot SITL is the reference a MAVLink client should be checked against,
   and an ArduPlane SITL binary **has been built on this machine** — `./waf
   configure --board sitl` and the build both succeeded — so the shortfall is a
   run that has not happened, not a binary that does not exist. `tests/sitl.test.ts`
   holds the ten semantic tests written for it and they are skipped unless
   `AERIALKIT_SITL` names a vehicle, so they report as *skipped* and never as
   *passing*. Everything the ArduPilot/PX4 column claims is
   against `firmware/tools/mavlink_fake_vehicle.py`, a stand-in that speaks
   through pymavlink — the same *encoder and decoder*, but a simulated vehicle.
   A real vehicle differs in ways this cannot reach: stream rates it sets for
   itself, message ids this app has no definition for, a signed link, a serial
   port that drops bytes, and a heartbeat phase that is not 1 Hz from boot.
6. **The npm audit surface was not assessed.** `npm install` was not run with
   `--audit` reporting here, and no dependency-vulnerability claim is made.
7. **`AK_BASE` was only ever exercised at `/aerialkit/configurator/`.** A
   different mount point is the same code path, but it was not measured.
8. **No handset has ever been on the other end of `rc channels` (0x0D).** The
   reply is checked three ways — against the firmware's own dispatch in
   `firmware/tests/test_proto.c`, against the real firmware over its console in
   `firmware/tools/akproto_firmware_check.py`, and against a stand-in and the
   demo board in this app's suite — and every one of those is a *synthetic*
   transmitter. The firmware's simulator frames a CRSF receiver with a fixed
   channel set, so what has been established is that the three decoders agree
   about a frame, not that the frame is what a handset sends. The two things on
   this board that no simulator can reach are the ones a bench session is for:
   whether `count` is eight on a real CRSF handset with sixteen channels
   configured, and whether a stick's raw count at full deflection lands where
   `rc_min`/`rc_max` say it does — which is the same question as whether the
   `sticks` per-mille figure is calibrated. `firmware/tools/akproto.py rc` is
   the tool: it prints the raw counts and the decoded sticks side by side, and
   the console's own `rc` prints the same numbers for the same instant, so a
   disagreement between a handset and either of them is visible while the stick
   is held over. Needs the owner's word and a board on a cable.
