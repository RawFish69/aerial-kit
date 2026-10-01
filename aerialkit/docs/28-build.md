# AerialKit - building and testing it on a Linux machine

This is the page that answers "what do I install, what do I type, and what
should come back" - for a person at a keyboard and for a pull request, which
are the same answer run in two places.

It exists because "the tests pass" had stopped meaning anything in particular.
`make test` runs the C checks, every SIL session, the fuzzer and seven Python
checks, and it does **not** reach `proto-test`, `window-test`,
`net-window-test`, `firmware-proto-check`, `sanitize`, `coverage`, `port`,
`stack-check`, `cross-check` or `check` - which is measurable rather than
rhetorical:

```bash
make -n test | grep -oE 'tools/[a-z_]+\.py|aerialkit-[a-z-]+' | sort -u
```

So a green `make test` meant whichever of the rest the person happened to
remember, and the ones nobody remembered are the ones that had never been run
anywhere. Finding 4 below is exactly that, and it was the first thing this page's
tooling found.

**One command runs everything this machine can run:**

```bash
cd aerialkit
make ci
```

That writes `ci-report/` and exits non-zero if anything failed. Read on for what
it needs, what it cannot do here, and the one thing it deliberately does not
fail on.

## What the machine needs

| Need | Version this page is written against | Why that one |
| --- | --- | --- |
| A host C compiler (`cc`) | gcc 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) | the host suite, the simulator, the fuzzer and every Python check's C half |
| `make` | GNU make | |
| `python3` | 3.12.3 | the client, the checks and the report. No third-party packages: everything here is the standard library, so there is no `requirements.txt` to go stale |
| `arm-none-eabi-gcc` **and** `arm-none-eabi-readelf` and `arm-none-eabi-nm` | Arm GNU Toolchain **13.2.rel1** (gcc 13.2.1, binutils 2.41) | the five ARM profiles, and `image_report.py` reads the images with `readelf`/`nm`, so a toolchain with the compiler alone is not enough |
| ESP-IDF | **v5.5** | the ESP32 profiles. Not installed on the machine this page was written on - see below |

On Ubuntu 24.04 the host half is:

```bash
sudo apt-get install build-essential python3
```

The cross toolchain is not a distribution package in any version worth having:
the pinned one lives in the workspace, one directory above this repository, and
the Makefile finds it by `CROSS` or by `PATH`. The path this repository's
neighbour uses:

```bash
export PATH=/path/to/arm-gnu-toolchain/bin:$PATH
arm-none-eabi-gcc --version | head -1
#   arm-none-eabi-gcc (Arm GNU Toolchain 13.2.rel1 (Build arm-13.7)) 13.2.1 20231009
```

`fc-firmware-workspace/targets/aerialkit-f405/target.conf` and the wing's own
`target.conf` are the canonical description of how the NAS builder builds these
images; `scripts/ci.sh` carries the same matrix for a host that has only the
toolchain, and the two have to agree. They are written down twice on purpose -
`target.conf` is consumed by the workspace's build harness, which is not here -
and a change to one is a change to both.

**`13.2.rel1` is pinned because the numbers move.** `arm-none-eabi-gcc` 12 and
13 emit different code for the same source, so a per-profile figure quoted with
no toolchain named is a figure that cannot be checked. `scripts/ci.sh` writes the
version it used into `ci-report/manifest.txt` before it builds anything, so a
number in a report can always be traced to the compiler that produced it.

### ESP-IDF, and what is not built here

The ESP32 profiles are an ESP-IDF project (`ports/esp32/`), not part of this
Makefile, and their images are measured by `idf.py size` rather than by
`tools/image_report.py`. `scripts/esp32-proto.sh`, `esp32-qemu.sh`,
`esp32-net-watch.sh` and `esp32-families.sh` build and boot them; all four take
`IDF_PATH` (default `~/esp-idf`) and skip with a line when it is absent.

**ESP-IDF is not pinned and that is a gap, not a decision.** The Makefile pins
`CROSS`; IDF is whatever `IDF_PATH` points at. What the ESP32 port was last
verified against is recorded only in the evidence it left: `docs/evidence/
esp32-network.txt` begins "Activating ESP-IDF 5.5" and the bootloader banner in
it reads `ESP-IDF v5.5`, so **v5.5** is the version those transcripts are from
and any other version is unverified. Pinning it belongs to the task that owns
the ESP32 port; it is written here so the absence is a named one rather than a
surprise.

## The stages

`make ci` runs these in this order, and `make ci CI_ARGS=--list` prints the
same list. `make ci CI_ARGS="--only test"` runs one stage; `--only suite` runs
the ten suite stages — `host`, `test`, `estimator-oracle`,
`attitude-kinematics`, `config-policy`, `config-recycle`, `timing`,
`boards`, `contract` and `control`; `--only protocol`
runs all four client halves. An
`--only` that matches nothing — a typo, or a name that used to exist — is an
error and exits 2, because a selection of zero stages would otherwise report
itself as a green run.

| Stage | Command | Needs | What it adds over the one before |
| --- | --- | --- | --- |
| *(manifest)* | - | - | written **first, before anything is built** - the revision, toolchain and source hashes, so a run that dies in stage one still says what it died on |
| `host` | `make host` | `cc` | the host build alone, which is the failure worth seeing on its own |
| `test` | `make test` | `cc`, `python3` | the largest single stage: the 2,000-odd checks, every SIL session, the fuzzer, the bench checklist, the docs check, the command smoke test and the suite-scope check. It is **not** the whole suite, which this page said until 2026-09-24: of the ten stages `--only suite` declares it reaches five, itself, `host`, and `boards`, `contract` and `control` through the checks in its own body. The other five are the oracles, and they are stages of their own so that a failure names which of them |
| `estimator-oracle` | `make estimator-oracle` | `cc` | the estimator against an independent quaternion truth, which is the one thing its own tests cannot be |
| `attitude-kinematics` | `make attitude-kinematics` | `cc` | the plant's attitude step against the same kind of truth - one layer down from the estimator's, and with the control that shows the old arithmetic fails |
| `config-policy` | `make config-policy` | `cc` | every route into the configuration, and who may persist while armed |
| `config-recycle` | `make config-recycle` | `cc` | the configuration ring through a power cut inside its erase |
| `timing` | `make timing` | `cc` | what the control loop does with a sensor interval it did not expect |
| `boards` | `make boards-check` | `make`, `python3` | the six board manifests read against each other, pin by pin, and the validator against fixtures - [30-boards.md](30-boards.md) |
| `contract` | `make contract-check` | `python3` | the cross-repository numbers - attitude units, the telemetry frame, the blackbox record, the NED→ENU rotation - recomputed by an implementation sharing no code with either producer - [31-contract.md](31-contract.md) |
| `control` | `make control-check python-syntax` | `cc`, `python3` | the control core through the host ABI a Python simulation drives it by: the C compiler's struct layouts against ctypes', the boundary rules, and a closed-loop experiment's reproducibility. The only stage that crosses a language boundary. Also compiles every tool, which is how the experiment - which cannot run here, needing numpy and the sibling checkout - is kept from rotting unread - [32-control-in-the-loop.md](32-control-in-the-loop.md) |
| `proto-test` | `make proto-test` | `cc`, `python3` | the protocol client against the firmware, over a pipe, plus the MSP and MAVLink clients |
| `window-test` | `make window-test` | `cc`, `python3` | the configurator window against a fake aircraft |
| `net-window-test` | `make net-window-test` | `python3` | the same window over a socket |
| `firmware-proto` | `make firmware-proto-check` | `cc`, `python3` | the firmware-side half of the client pair |
| `sanitize` | `make sanitize` | `cc`, `python3` | asan/ubsan over the suite: the host build, `make test` in its two halves - the register-model tests in their own process, under the shadow-gap concession, see finding 1 - the protocol checker, the fuzzer, and the five oracles. It reaches all ten stages `--only suite` declares; until 2026-09-24 it reached five, because like `coverage` it said "the whole suite" and meant `make test`, and the five it missed had never been under the sanitizers at all |
| `coverage` | `make coverage` | `cc`, `python3`, `gcov` | `make test` and the five oracles under `--coverage`, plus the two protocol checks, and the line map it leaves. All ten suite stages, because the map is read as a work queue and a stage left out puts lines on it that a test already covers - measured on 2026-09-24, the five oracles were worth six lines, and the six were the flight core's clock-reset branch |
| `profiles` | the five-image matrix | the ARM toolchain | every image built, measured against its own linker script, and compared with the linker's own arithmetic |
| `stack` | `make stack-check` | the ARM toolchain | the one question the host suite cannot be asked about a built image: how much stack it needs when it runs. The host build has eight megabytes and the F405 has tens of kilobytes, so an overflow is invisible here and presents on the bench as a board that stopped saying anything. The compiler's call graph gives a deepest resolved chain of 2,472 bytes and 8,584 with one largest-frame call charged at each of the 516 sites it cannot follow, against a 12,288-byte ceiling (`STACK_LIMIT`); the pessimistic reading is the one that fails the stage. It builds **one** image where `profiles` builds five, and that is a measurement rather than a saving: the fitted, tell-tale and GHF435 images read 2,472 and 8,584 too, and their function counts (426, 428, 433 against 423) are what show they are different images whose variants join neither the deepest chain nor the largest frame - [21-port-on-the-host.md](21-port-on-the-host.md) |
| `configurator` | `make configurator-check` | `node`, `npm`, and the app's own installed `node_modules` | the web app's own suite, and the only stage here that runs a program outside this subtree. `apps/configurator` is self-contained by design - its own package.json, its own vitest, its own README - and until this stage existed **nothing in this repository ran its tests**: `window-test` drives the Python window against a fake aircraft and `proto-test` drives the protocol client against the firmware over a pipe, and neither loads a line of the app's TypeScript. Measured when it was wired in, 2026-09-20: 186 tests passed, 10 skipped (`sitl.test.ts`, which needs a live SITL), 12 files, 5.6 s, and `tsc --noEmit` clean. It deliberately does **not** assert that count - a number written into a gate goes stale exactly the way the one in the app's own `docs/BUILD-AND-DEPLOY.md` did - and asserts instead the invariant a count cannot express: **the number of test files vitest collects equals the number of test files on disk**. A suite that is green about less than it says it is, because a renamed glob stopped matching, is the failure a green run hides, and this project has hit it twice in other instruments. `node_modules` is gitignored, so on a fresh checkout the stage is recorded as **not run** and names the path it lacks, the way `profiles` and `stack` name the ARM programs they are missing, and `make configurator-check` run by hand refuses and says `npm ci`. It is last because it shares nothing with the eighteen stages before it, and appending it keeps the order those are quoted in unchanged |

### Four ways a stage can end

The report distinguishes them, and the distinction is the point. Collapsing any
two of them is how a build report starts lying.

| | When | Exit code | Where it shows |
| --- | --- | --- | --- |
| **ok** | it ran and passed | 0 | `ran:` in `summary.txt` |
| **not run** | a program it needs is absent, or the check's own log says `not checked here` and printed no `ok` of its own | 0 | `not run:` in `summary.txt`, with the missing program |
| **failed, allowed** | it failed and its log contains the one pattern the allowance names | 0 | `allowed:` in `summary.txt`; the last line says so |
| **failed** | anything else | 1 | `failed:` in `summary.txt` |

Every one of those rules was paid for on the afternoon this script was written,
and each of them is a way a report was greener than the run it described.

**An allowance is a pattern, not a licence.** No stage is excused today; the
mechanism stays because this is a property of the script and not of any one
stage. `sanitize` used to be, and what ended its allowance is the argument for
the rule: the pattern was `shadow gap area`, it matched, and behind it sat a
second failure - the control check, which could not run at all - that the
allowance had been covering for as long as it existed. The first version of this
excused a stage *unconditionally*, on the theory that the reason was known, and
then swallowed a *different* stage's failure, which reported itself as "not
measured - no ARM cross toolchain" on a machine whose PATH had that toolchain on
it. The run exited zero. An allowance that does not check what it is excusing is
a blanket over the whole stage.

**Exiting zero is not the same as having checked anything.** Every tool here
skips with a line and a zero when its precondition is missing -
`net-window-test` printed `not checked here - no qemu-system-xtensa` and the
first run reported `net-window-test: ok in 0s`. A stage that printed no `ok` of
its own and said it did not check here is now recorded as **not run**. Both
conditions are needed, because `make test` legitimately contains a check that
skips a section of itself (`tools/reference_defaults_check.py` with no INAV
checkout) surrounded by two thousand `ok` lines; that stage ran, and its skips
are reported as a count with the lines quoted.

The four protocol stages are not redundant with `make test`, and the difference
is measurable:

```bash
make -n test | grep -oE 'tools/[a-z_]+\.py|aerialkit-[a-z-]+' | sort -u
```

That prints what `make test` actually invokes - six host binaries and seven
Python checks. It does not build `tools/akproto_sim.c`, does not run either
window client, and does not run the firmware-side check, so these four were
reachable only by someone who already knew they existed.

## The five profiles, and what each one costs

`profiles` is the stage that was never run as a set. It builds:

| Profile | Board | Arch | Part | Flags |
| --- | --- | --- | --- | --- |
| `aerialkit-f405` | `AERIALKIT_F405` | `stm32f405` | `stm32f405rg` | - |
| `aerialkit-f405-fitted` | `AERIALKIT_F405` | `stm32f405` | `stm32f405rg` | `-DAK_BOARD_BARO_FITTED=1 -DAK_BOARD_VBAT_FITTED=1` |
| `aerialkit-f405-telltale` | `AERIALKIT_F405` | `stm32f405` | `stm32f405rg` | `-DAK_BOOT_STAGE=1` and both fitted flags |
| `aerialkit-ghf435` | `AERIALKIT_GHF435` | `at32f435` | `at32f435rg` | - |
| `aerialkit-ghf435-telltale` | `AERIALKIT_GHF435` | `at32f435` | `at32f435rg` | `-DAK_BOOT_STAGE=1` |

The flags are not cosmetic. The `FITTED` pair is what a board with the
barometer and the pack divider soldered on compiles; `AK_BOOT_STAGE` is the
tell-tale image, which is the one that gets flashed when the console never comes
up. They are different images because they are different answers to "what is on
this board", and a report that measured only the bare one would be describing a
board nobody has.

**The three columns are checked against each other, and the board decides.**
`BOARD`, `ARCH` and `PART` each have their own default, so
`make BOARD=AERIALKIT_GHF435` on its own leaves `ARCH=stm32f405` and compiles the
AT32 board file against the STM32F405 headers: three errors *inside that board's
own pin definitions*, which read as defects in the board rather than as a missing
argument. Each ARM board declares the arch it is built against —

```c
#define AK_BOARD_ARCH at32f435      /* src/boards/AERIALKIT_GHF435/board.h */
```

— and `make` reads it and refuses the mismatch by name before compiling anything,
names the arch when `PART` belongs to another one, and refuses a `BOARD` that
names no directory at all (the misspelling which otherwise compiles the whole core
against an include path that is not there and leaves `-DAK_BOARD=…` objects in the
default `build/`).

The dangerous direction is the one that *succeeds*: `BOARD=AERIALKIT_F405
ARCH=stm32f405 PART=at32f435rg` links and writes a 109,964-byte `.bin` — F405 code
against the AT32 memory map — and only `make check` refuses it, as three vector
failures that read as receiver and DShot bugs. `make ci` never sees it, because
its profiles are correct; traps 202 has the measurements.

Measured, on this machine, by `make ci` — every figure below is also a line the
linker printed while building that image, and the two are compared byte for
byte:

| Profile | Flash | of 1,048,576 | RAM | of 131,072 |
| --- | --- | --- | --- | --- |
| `aerialkit-f405` | 104,500 | 9.97% | 78,452 | 59.85% |
| `aerialkit-f405-fitted` | 105,184 | 10.03% | 78,456 | 59.86% |
| `aerialkit-f405-telltale` | 105,720 | 10.08% | 78,784 | 60.11% |
| `aerialkit-ghf435` | 105,700 | 10.08% | 78,800 | 60.12% |
| `aerialkit-ghf435-telltale` | 106,244 | 10.13% | 79,128 | 60.37% |

Toolchain: Arm GNU Toolchain 13.2.rel1, gcc 13.2.1. Source: the hash in the
manifest, `ebc6bf9` plus the working tree it was measured from. **Those two
qualifiers are not decoration** — a per-profile figure quoted without the
compiler that produced it cannot be checked, because gcc 12 and 13 emit
different code for the same source.

**Sixty per cent of RAM, and the stack is not in it.** The stack lives in the
same SRAM and is not a section, so no number on this page includes it.
`scripts/check-stack.sh` answers that question from the compiler's call graph,
and for `aerialkit-f405` it says: the deepest chain of resolved calls is **2,488
bytes over 5 frames** (`Reset_Handler` → `ak_firmware_main` → `preflight_run` →
`arm_line` → `ak_flight_arm_check`), and **8,584 bytes** if every one of the 509
indirect call sites this analysis cannot follow is charged the largest single
frame. Against a 12,288-byte limit. A profile at sixty per cent of RAM with a
worst case of seven per cent of it in the stack has less headroom than sixty per
cent sounds like, and neither figure means much on its own.

Each is built into its own `OUT=` directory, checked with `make check` against
`scripts/image-facts/$(PART).txt`, and then measured by `tools/image_report.py`.
For a single profile by hand:

```bash
make image-report                          # the one this invocation was given
make BOARD=AERIALKIT_GHF435 ARCH=at32f435 PART=at32f435rg \
     PRODUCT=aerialkit-ghf435 image-report
```

**Why not `make size`.** `size` prints `text`, `data` and `bss`, which is the
wrong shape for the question: `data` is counted once as flash (it is stored
there) and once as RAM (it is copied there), and `bss` is the sum of section
sizes without the alignment padding the linker reserved between them. The linker's
own `--print-memory-usage` is authoritative but only exists in the build log.
`image_report.py` reads the ELF's program headers instead - each `PT_LOAD` has a
load address and file size (flash) and a virtual address and memory size (RAM) -
and the region is the span from its origin to the furthest end any of those
reaches. That span includes the padding, so it agrees with
`--print-memory-usage` byte for byte.

That agreement is not asserted, it is measured, twice: `make image-report-check`
holds the arithmetic to the numbers a real link printed, on committed fixtures
under `tests/fixtures/image-report/`, with no cross toolchain needed; and
`scripts/ci.sh` compares every one of the five profiles against the line the
linker printed while building it. Either one failing means every per-profile
figure the project quotes is wrong.

(The map fixture is `stm32f405rg.map.txt` and not `.map` because `.gitignore`
has `*.map` for the linker map of every build. A fixture with an ignored
extension is a fixture that never gets committed, and the check that reads it
then fails in a fresh clone and nowhere else.)

The stage also writes `artifacts.txt`: the sha256 and the size of every image it
measured, and of the `.bin`/`.hex` beside it where the build makes one. The
figures in `profiles.txt` describe the image whose hash is in that file and no
other — two builds of one revision are two images until their hashes say
otherwise, which is a mistake this workspace has already made once (a NAS build
and a local build of the same revision are not the same bytes).

Each row of `profiles.txt` also carries the revision **and the build stamp** the
image itself carries, read back out of its `.rodata` — the two lines the console
banner prints at boot. That is not the hash and does not replace it. The
revision is `git describe --always --dirty`, which records *that* something was
uncommitted and never *what*, so two builds that share no bytes carry the same
one; the stamp is what tells those two apart, and the hash is what identifies
either. `04-traps.md` §156 is the day a run-book quoted one of those builds as
the other, and the revision field it was quoting could not have caught it.

The report is not a statement about the stack, and the numbers above show why
that matters: the stack lives in the same SRAM and is not a section, so it is in
none of them. `scripts/check-stack.sh` is the other half — see the measured
figures under the profile table.

## The container form

Nothing here needs a container, and that is deliberate: the only two things that
are not distribution packages are the ARM toolchain and ESP-IDF, both of which
are directory trees that a `PATH` or an `IDF_PATH` points at, so both are
pinnable on a bare host by unpacking them at a fixed path. A container would add
a second way for the build to differ from the machine that flashes the aircraft,
which is the thing that has already gone wrong in this workspace (a NAS build and
a local build of one revision are two images).

What CI actually does is the two-stage form of the same idea:

```yaml
# .github/workflows/aerialkit.yml
- run: sudo apt-get install -y build-essential python3
- run: make ci
```

with the cross toolchain fetched and unpacked at a pinned version first. The
workflow is the authority on how that is done; this page is the authority on
what `make ci` means.

**The workflow has not been run.** There is no GitHub runner on the machine this
was written on, so what has been validated is `scripts/ci.sh` - thoroughly, twice,
once from a clean clone - and not the YAML that calls it. Two things in it are
unverified and would be the first to check when it does run: that Arm's published
`<tarball>.sha256` is in the `<hash>  <filename>` form `sha256sum -c` expects
(the step renames the download to its real name before checking, precisely
because the checksum file names it), and that `ubuntu-24.04`'s `cc` is the 13.x
the manifest records. Everything else in the job is `make ci` and an annotation.

## Findings

Severity is the project's own: **high** is a claim the project makes that is not
true, **medium** is a number or a guarantee that is weaker than it reads,
**low** is a defect with no consequence for the aircraft.

| # | Severity | Finding | Status |
| --- | --- | --- | --- |
| 1 | medium | `make sanitize` could not pass on x86-64: the host port test models ARM's System Control Space at `0xE000E000`, which is inside AddressSanitizer's shadow gap, so every access there was reported as a crash whatever the mapping said. Fixing that exposed a *second* failure behind it, in the control check, that nothing had ever reached. | **fixed** on 2026-09-20 - `ASAN_OPTIONS=protect_shadow_gap=0` for the suite, and a preloaded runtime plus leak suppressions for the Python process that loads the instrumented library. The concession the first one makes is measured, not assumed, and **bounded to the six tests that need it** on 2026-09-24: `make sanitize` now runs the suite in two invocations, 1384 checks with the gap protected and 885 with it conceded (the 2026-09-29 measurement; 1336 and 786 when the split was made), where it previously ran the whole suite under the concession. See below |
| 2 | low | `ak_proto_init` left `can_stream` uninitialised. Invisible on the flight firmware (`static`, so BSS) and visible in `tools/akproto_sim.c`, which declares its parser on the stack - a subscribe could answer with a rate the simulator cannot send, so `akproto_cli_check.py` timed out or not depending on the stack. | **fixed** - `ak_proto_init` now zeroes the whole struct; `tests/test_proto.c` pins the property, not the field list. The fix is 24 bytes *smaller* (104,524 → 104,500 of flash for `aerialkit-f405`, measured through `scripts/ci.sh --only profiles` with the file stashed and unstashed) |
| 3 | low | ESP-IDF is not pinned in this repository. The last verified version is v5.5, recorded only in `docs/evidence/esp32-network.txt`. | open, named above |
| 4 | medium | `make proto-test`'s companion check had never passed on this machine: the Pi companion could not find `akproto.py`, because none of the three places it looks resolves in this repository's layout and nothing set `AKPROTO_PATH`. `make test` does not run `make proto-test`, so nothing said so. | **fixed** - the Makefile passes `AKPROTO_PATH=$(CURDIR)/tools`, the same way it already passed `AK_SIM` and `AK_PRODUCT` |
| 5 | medium | The `profiles` stage never ran: `rm -rf "$out"` with no `mkdir -p` on its parent, so the first build's log redirect failed and the loop body never executed. The stage died one second in. | **fixed** - `mkdir -p "$OUT/build"`. The *second* half of the finding is below, and it is the more interesting one |
| 6 | low | `net-window-test` reported `ok in 0s` having printed only `not checked here - no qemu-system-xtensa`. A stage that did nothing was counted as a stage that passed. | **fixed** - the zero-with-no-`ok` rule above |
| 7 | low | `--only protocol` was in this page and in `ci.sh`'s own help text, and selected no stages at all: the group was a heading, not a name the matching answered to. The run reported `ran: none` and `aerialkit CI: ok`. | **fixed** - `--only` takes stage names and two groups, and an unknown one exits 2 |

### What running it the first time found in itself

Findings 5, 6 and 7 are about this page's own tooling, and they are here because
they are the same failure the tooling exists to catch, one level up: a report
that is greener than the run it describes.

The first *complete* run reported `profiles: FAILED after 0s` and then, in the
same breath, `profiles: allowed to fail - needs the ARM cross toolchain`. The
ARM toolchain was on `PATH` - the manifest written by that same run names it and
its version. The allowance was unconditional, so it excused a stage whose
failure had nothing to do with the reason written on it, and the run exited
zero. Two defects, each hiding the other: `stage_profiles` never ran, and the
mechanism that should have made that loud made it quiet.

`counts.txt` then printed `profiles: not measured - no ARM cross toolchain`,
which was a *third* wrong statement, assembled by `stage_counts` from the
absence of `profiles.txt` plus a guess about why.

All three are fixed, and the shape of the fix is the same in each: say what was
observed and not what was inferred. A stage that cannot run because a program is
missing says which program. A stage that failed says what the failure was, and
an allowance that does not match it is not an allowance. A report that has no
profiles says it has no profiles and points at the stage that would have made
them.

Finding 7 arrived the same way, from trying to use the thing: `--only protocol`
was written in the help text and matched nothing, and the run it produced said
`ran: none` and `ok`. A selector that resolves to the empty set is a mistake
rather than a request for nothing, so it is an error now.

### Finding 1 in full, because it is the one that was met first

`make sanitize` failed, and it failed before any of this page's tooling existed.
The failure:

```
AddressSanitizer: unknown-crash on address 0x0000e000e000
WRITE of size 4096
    #2 reset_registers   tests/test_arch.c:148
    #3 test_the_clock_tree   tests/test_arch.c:178
```

`tests/test_arch.c:67` defines `PRIVATE_BASE 0xE000E000UL` and `PRIVATE_SIZE
0x1000UL`. That address is the ARM System Control Space, and on x86-64 it is
inside ASan's shadow gap - LowMem ends at `0x7fff7fff`, the gap runs from
`0x8fff7000` to `0x2008fff6fff`, and a byte's shadow is `(address >> 3) +
0x7fff8000`, so `0xE000E000` asks for shadow at `0x9bff9c00`, which is in the
gap. With ASan's default layout nothing maps shadow there, the lookup faults,
and the handler reports it as `unknown-crash` against an address the test owns
and had just mapped. It is not a bug in the firmware, and it is not a bug in the
test: it is the sanitizer's memory layout. `tests/test_arch_at32.c:42` maps the
same window, for the same reason; those two files are the only ones in the tree
that reach into the gap, and `0x40000000`, `0x50000000` and `0x08000000` - the
other three windows they and the other arch tests map - all put their shadow in
the low shadow, where they are checked normally.

Reproduced minimally, without this repository at all, in thirty lines that
`mmap` `0x40000000`, `0x50000000` and `0xE000E000` and write a page to each:

| build | `0x40000000` | `0x50000000` | `0xE000E000` |
| --- | --- | --- | --- |
| plain | ok | ok | ok |
| `-fsanitize=address` | ok | ok | **aborts** |
| `-fsanitize=undefined` | ok | ok | ok |

Proven pre-existing by stashing the two files this work touched, rebuilding
`build-asan/aerialkit-tests`, and getting the identical failure.

**Fixed, and not by turning the instrument off.** The obvious fix is to stop
instrumenting the arch tests, and it is the wrong one: the sanitizers are what
catch an out-of-bounds write in exactly that code, and a carve-out would silence
the instrument everywhere it is most useful in order to make one line green.
What the Makefile does instead is set `ASAN_OPTIONS=protect_shadow_gap=0`, which
is ASan's own documented answer for an application that maps memory in the gap
(the `SAN_ENV` block). The arch tests are still compiled and checked with
everything else.

**What that concession costs, measured rather than assumed.** The first version
of the Makefile comment claimed it was narrow - that an access to an *unmapped*
address in the gap would still fault. That claim was a prediction about libasan's
internals, so it was tested, and it is **false**. With the option on, ASan maps
the gap as ordinary zeroed memory: its shadow reads back as "nothing poisoned",
so an access anywhere in the gap is permitted. Two probes:

| probe | default | `protect_shadow_gap=0` |
| --- | --- | --- |
| write to `0xE000E000`, the window the arch test owns | unknown-crash | ok |
| write to `0xA0000000`, in the gap and mapped by nothing | unknown-crash | **ok** |
| write to `0x9bff9c00`, the shadow of the page above | unknown-crash | **ok** |
| heap-buffer-overflow, `p[16]` on a 16-byte allocation | heap-buffer-overflow | heap-buffer-overflow |
| write to `0x1000`, unmapped low memory | SEGV | SEGV |

The last two rows are the reason this is acceptable rather than merely
convenient: everything outside the gap is checked identically with the option on
and off, and every ordinary memory error in this suite - a heap overflow, a
stack overflow, a stale pointer into freed memory, a bad low address - lives
there. The gap is twenty-four terabytes at addresses between `0x8fff7000` and
`0x02008fff6fff`, above anything this program allocates. What is actually lost is
a wild pointer whose value happens to land in that band.

**The residue, now bounded to the six tests that need it.** That residue was
removed on 2026-09-24 by running the tests that need the option in an invocation
of their own and everything else without it. The runner half was already in the
tree: `tests/test_aerialkit.c` takes `--only-registers` and `--no-registers`,
and `make test` passes neither, so the ordinary build is unchanged. Measured on
this tree on 2026-09-29, `--only-registers` runs 885 checks and `--no-registers`
1384, against 2269 for the whole run, zero failed in all three, and
885 + 1384 = 2269 - the split is real and lossless. (It was 786/1336/2122 when
the split was made; the numbers move whenever a test does, which is why they are
written down with a date. `tests/test_board_feather.c` is the most recent mover
and it moved only the register half: it maps the register pages, so all 54 of its
checks run under `--only-registers`, and the suite went 2,215 - the reading in
trap 204 - to 2,269.) The wiring is
`TEST_ARGS`, defaulting to empty, and a `sanitize` target that now makes three
runs instead of one:

| invocation | `SAN_ENV` | checks |
| --- | --- | --- |
| `make test ... TEST_ARGS=--no-registers` | not set | 1384, gap **protected** |
| `$(SAN_OUT)/aerialkit-tests --only-registers` | set | 885, gap conceded |
| `proto-test`, then `fuzz` | not set | 71, then the parser fuzz |

**The order is the part that matters.** The 1384 checks run first and with the
gap protected, which is a setting they have never been run under - before this
they inherited the concession from the top of the stage. `$(SAN_ENV)` was also
removed from the `proto-test` and `fuzz` lines for the same reason, and from
`CONTROL_LIB_ENV`, where `ASAN_OPTIONS=protect_shadow_gap=0` had been carried by
the same reflex that put it on the whole of `sanitize`. That last one was
measured out rather than assumed: run both ways, the two logs are byte-identical,
98 lines each, `akcontrol_check: 71 checks, 0 failed` in both.

**Falsified, because a split that is not load-bearing is decoration.** Run with
the option removed from the register invocation - `make sanitize SAN_ENV=` - the
stage exits 2 at `Makefile:1471`, having passed the contract check and the
control check on the way:

```
Address 0x0000e000e000 is located in the shadow gap area.
==3421588==ABORTING
make: *** [Makefile:1471: sanitize] Error 1
```

The abort lands on the register half and nothing before it, which is what makes
the two-invocation shape evidence rather than a preference: those six tests need
the option, and no other check in the suite does.

Two things were measured while working the change out, and both cost more to
find than they look:

  - **The set is six tests, and "the two arch files that map the block" is the
    answer that looks right and is not.** What belongs in it is the tests that
    *run against* the model: the fault record reads its status registers out of
    the private page, and all three board tests drive a whole boot sequence
    against the peripheral one. Selecting only the two arch files gave `1273
    checks, 2 failed` against `2054 checks, 0 failed` for the whole run as it
    stood then, with `test_board_f405` and `test_board_ghf435` at *zero* checks
    each - they had
    been asking whether an earlier test had mapped the pages and returning
    quietly when the answer was no, which from outside reads as a pass.
  - **Each of them needs the clock state as well as the mapping.** The board
    tests run `ak_board_init()`, and `ak_i2c_init` derives its bus timing from
    `ak_clk_apb1_hz()`: a bus timed for the internal oscillator's 16 MHz does
    not complete a DPS310's transactions. Two checks in
    `tests/test_board_ghf435.c` - *and the part opens through it*, *with a
    sample that says it is valid* - had been passing on a clock that
    `test_arch_at32()` left running in another file three tests earlier.

  Both are measurements taken while working the change out, and both are why the
  register half is six tests and not two files: selecting by file gives three
  board tests that report zero checks and call it a pass.

**And there was a second failure behind it.** With the arch crash fixed the suite
ran to the end for the first time, and the stage then failed at the very next
thing it does. `tools/akcontrol_check.py` loads the control core as a shared
library through ctypes, and under `make sanitize` that library is built by the
sanitizing compiler while the interpreter is not:

```
==2100438==ASan runtime does not come first in initial library list;
you should either link runtime to your application or manually preload it with LD_PRELOAD
```

`build-asan/libaerialkit-control.so` carries 22 `__asan_*` dynamic symbols, which
is why. The Makefile's `CONTROL_LIB_ENV` block preloads the runtime for that one
invocation - exactly what the message asks for - and `tools/lsan.supp` handles
what follows from it: LeakSanitizer then watches CPython, which does not free
everything it allocates before it exits. On this machine that is 150 allocations
in 149,106 bytes, every frame of them in the interpreter and not one in the
library. They are suppressed by name rather than by setting `detect_leaks=0`,
because a leak that starts in the control library must have a library frame at
the top of its stack, so a pattern naming CPython cannot hide one - whereas
turning leak detection off would hide all of them.

Neither of these was reachable while the stage aborted at check 230, which is the
whole argument for the count below.

**The count is why the first one was worth fixing rather than excusing.** The
abort was at check 230 of 2118. The stage was not failing at the end of the
suite, it was failing near the beginning, so 1888 checks - the mixer, the flight
core, the estimators, the rest of everything - had never once been through the
sanitizers. "Allowed to fail" recorded the failure and not how far it got, and
the distance is what said how much was not being checked. `ci.sh` therefore no
longer carries a pattern for this stage: `stage sanitize "make cc python3" ""`
and a failure of any kind is a real failure. `make ci CI_ARGS="--only sanitize"`
treats it as a hard failure, because then it is the whole point of the run.

### Finding 4, which is the argument for this page existing

The first *complete* `make ci` run failed at `proto-test`, on this:

```
File ".../projects/flight-computer/companion/fc_link.py", line 44, in _load_akproto
    raise ImportError(
ImportError: akproto.py not found; set AKPROTO_PATH to the firmware's tools directory
```

The Pi companion does not reimplement the wire: it loads the firmware's own
client. It looks in three places, and none of them resolves here -
`$AKPROTO_PATH` (unset), `projects/aerialkit/tools` relative to itself (that
layout assumes the companion and the firmware are siblings *under* `projects/`,
and this repository's firmware is at the root), and
`aerial-kit/aerialkit/tools` (a home directory that does not exist
on the machine this was found on). The import raises before the script has read
its own `argv`, so the path the Makefile already hands it could never have
helped.

This is a cross-repository contract with nothing checking it, and it had been
broken for as long as the layout has been this way. Nobody knew, because
`make test` does not run `make proto-test` - the check was reachable only by
someone who already knew to type it. The fix is one variable the build already
knows the value of: `AKPROTO_PATH=$(CURDIR)/tools`, passed the same way the two
variables above it already were.

**That is what this page is for.** Not the 2,047 checks - those were green
before - but the four stages no one was running.

## Reproducing the historical baseline

The 2026-09-18 assessment lists, as evidence inspected but not rerun:
*"DeepSeek 2,042 checks / 42 sessions / sanitizer / 96.9% coverage"*
(`AERIAL-KIT-ASSESSMENT-2026-09-18.md:677`). Three of the four reproduced
exactly when this was first checked; the fourth did not, and the reason turned
out to be a defect in this repository rather than a difference of environment.
It was fixed on 2026-09-20 - all four reproduce now, and the last row says how.

```bash
make ci
grep -E 'checks|sessions|coverage|sanitizers' ci-report/counts.txt
```

| Claimed | Reproduced | Where it comes from |
| --- | --- | --- |
| 2,042 host checks | 2,042 before the fix above, **2,047** after (the five new ones are `test_parser_initialisation`, asserting a property rather than naming a field) | `0 failed` on the line `aerialkit-tests` prints |
| 42 sessions | 42, plus one `fuzz: PASS` | `grep -c '^sim: PASS'` |
| 96.9% coverage | 96.9% | `coverage: the firmware, 77 files, 8335 of 8601 lines executed (96.9%)` |
| sanitizer green | **reproduced 2026-09-20**: the suite runs under asan and ubsan to `2118 checks, 0 failed`, `akcontrol_check: 70 checks, 0 failed`, `fuzz: PASS`, exit 0. Not reproduced before that date | `make sanitize`; finding 1 below |

The counts in `counts.txt` are read out of the logs of the run that produced
them, not retyped, so a run cannot report a number it did not measure. The
revision, the three toolchain versions and a hash over the source that was
compiled are in `manifest.txt` beside them.

**That sentence was false until 2026-09-19, and in the direction that looks like
caution.** Each block in `stage_counts` was gated on `[ -f "$OUT/sanitize.log" ]`
and its siblings, and `ci-report/` is never cleared between runs — so a
`--only suite` run printed the *previous* run's coverage figure and sanitizer
verdict under a heading reading "what this run produced", eleven lines above its
own `skipped:` line naming both stages as not run. The gates now test membership
in `STAGES_RUN`, the run's own record of what executed, so the sentence above is
enforced rather than asserted. Two consequences worth knowing when reading a
report:

- a stage that did not run says `not run this time`, not `not run` — the
  distinction the line has to carry is *this run* against some other one;
- a stage that failed within its allowance says `FAILED, and excused - <reason>`,
  so `counts.txt` read on its own cannot report a bare `FAILED` about the
  sanitizers when the summary three inches away records it as allowed. No stage
  is excused today, so that branch is unexercised now (finding 1) - a report that
  reaches it is saying something that has not happened since 2026-09-20.

## Validated from a clean checkout, because a working tree is not evidence

```bash
git clone --no-hardlinks /path/to/source-checkout /tmp/clean-checkout
git -C /tmp/clean-checkout checkout 0b0c1d4
cd /tmp/clean-checkout/aerialkit
PATH=<toolchain>/bin:$PATH ./scripts/ci.sh --out ../ci-report
```

A clone is the right shape for this and `git status` on a working tree is not: a
clone has exactly the tracked content at that commit, so anything the build
reaches for that is ignored - `upstream/pymavlink-2.4.49/`, `projects/` - is
absent there and present here. That difference is what this is looking for. It
is how the 2026-09-18 landing rehearsal found the MAVLink defect (04-traps.md
§24), where a check ran against a vehicle that did not exist.

**What came back at `0b0c1d4`, 661 tracked files, tree clean:**

| | working tree (`ebc6bf9-dirty`) | clean clone (`0b0c1d4`) |
| --- | --- | --- |
| host checks | 2,047, 0 failed | **2,047, 0 failed** |
| sessions | 42 passed | **42 passed**, `fuzz: PASS` (seed 0x5eed2026) |
| coverage | 96.9%, 77 files, 8,335 of 8,601 | **identical** |
| stages | 8 ran, 1 allowed, 1 not run | **8 ran, 1 allowed, 1 not run** |
| exit code | 0 | **0** |
| flash, `aerialkit-f405` | 104,500 | **104,492** |
| flash, `-fitted` | 105,184 | **105,176** |
| flash, `-telltale` | 105,720 | **105,712** |
| RAM, all five profiles | 78,452 ... 79,128 | **identical to the byte** |

Nothing crashes and nothing has to be faked: the checks that need the ignored
trees say so and skip (7 lines in `proto-test`, 1 in `test`), which is the defect
the 2026-09-18 rehearsal found staying fixed.

### The eight bytes, which are not a measurement error

Every profile is 8 bytes larger in the working tree than in the clone, and the
cause is worth knowing before somebody reads a per-profile figure as a property
of the source:

```bash
grep -n 'REV' Makefile | head -1
# 26:REV ?= $(shell git -C $(CURDIR) describe --always --dirty ...)
```

`git describe --always --dirty` returns `0b0c1d4-dirty` in a working tree with
uncommitted changes and `0b0c1d4` in the clone: thirteen characters against
seven. That string is compiled into the image as `-DAK_REV`, so the six extra
characters plus two of alignment are 8 bytes of flash, and nothing else differs -
RAM is identical in all five profiles, because the string is not in it.

Two consequences, and the second is the reason this section exists:

* a per-profile flash figure is only meaningful beside the build it was measured
  from. `tools/image_report.py` puts the revision and the build stamp in every
  row, `manifest.txt` records the revision and `artifacts.txt` the hash, so the
  figure and its provenance travel together — and of those three, the stamp is
  the one that separates two builds of one revision and the hash is the one that
  identifies either. The revision alone does neither, which is measured rather
  than argued in `04-traps.md` §156: a run-book sent an operator to a build five
  hours older than the one it described, and the two carried the same revision;
* comparing a number measured in a dirty tree against one measured in a clean
  clone will show a difference that is not a change in the code. 104,500 and
  104,492 are the same firmware.

## What this page does not claim

* Nothing here is a statement about hardware. Every stage runs on the host or in
  QEMU; the F405 and the GHF435 have never been flashed from this machine. The
  one measured bench qualification is a separate task and has not happened.
* `make ci` measures five profiles and says nothing about whether they are the
  right five. The matrix is a claim about which boards this firmware ships for,
  and it lives in `scripts/ci.sh` and in two `target.conf` files.
* The `profiles` stage is not a substitute for `scripts/check-stack.sh`,
  `make port` (does the portable core still compile for xtensa/riscv?) or
  `make cross-check` (the same suite as x86_64 sees it). All three are separate
  targets with their own preconditions, and none of them is in `make test`.
