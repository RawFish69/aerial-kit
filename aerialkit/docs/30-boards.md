# AerialKit - the board manifests, checked against each other

    make boards          # the seven real boards
    make boards PINS=1   # and every pin every board claims
    make boards-check    # the validator itself, against fixtures

A board header is not a build file. `src/boards/ESP32S2DEV/board.h` does not
compile anything, and nothing in this repository reads two of its lines together
— so it can say that the status LED is on GPIO 15 and that the first servo is on
GPIO 15, and every check the project runs will pass. This page is about the
check that reads them together, what it found on its first run, and what it does
not prove.

It exists because of one of those: the S2's header gives the LED and servo 1 the
same pad, in the configuration a **bare devkit** builds. That is not a corner
case. Driving servo 1 drives the status line, `AK_BOARD_SERVOS` is `2u`, and
`soldered` is not in the picture — a board with nothing attached to it behaves
this way.

## What a manifest claims, and who was reading it

The macros are read in four test files and one board test asserts a few of them
by number:

    tests/test_board_f405.c          AK_BOARD_LED_PIN, driven and asserted
    tests/test_board_ghf435.c        AK_BOARD_MOTOR1_PIN == GPIOB 6, and the LED
    tests/test_board_feather.c       the servo pads, driven on TIM4 CH3/CH4
    tests/test_arch_esp32_output.c   MOTOR1..4, SERVO1..2, on the RMT and LEDC
    tests/test_arch_esp32_fault.c    AK_BOARD_LED_GPIO, driven

Every one of them takes the manifest's word for the pin: it drives the pad the
header names, or restates one header line as an expectation. None compares two
of a board's own resources, which is the only way a header that gives one pad to
two functions can be seen — and every build, image, profile and structural check
in the tree is downstream of a manifest that is assumed to be self-consistent.

## What it checks

    two pins may share a pad only if both are bus lines of the same kind
    (SCL with SCL, MOSI with MOSI) *and* both name the same peripheral.

The second half is what makes it a rule rather than a spelling test. A
barometer and a rangefinder on one I2C pair is not a collision, it is a bus, and
the manifests say so by naming the same peripheral: `AK_BOARD_BARO_I2C` and
`AK_BOARD_RANGE_I2C` are both `I2C1_BASE` on the F405 and both port 0 on the
ESP32 boards. Read backwards, that rule is a check of its own — the same two
pins naming *different* peripherals is a collision, and `twobuses.h` below is
that fixture.

A resource that declares `AK_BOARD_<RES>_FITTED` is checked only when it is
fitted. A devkit's header carries the pins a sensor *would* use; on a board with
`AK_BOARD_IMU_FITTED 0` there is no sensor on that pad and no conflict either.
So every board is checked twice — as written, and with every fitted flag forced
to 1 — because the fitted configuration is the one nobody builds by default and
the one `scripts/esp32-proto.sh` builds.

## What it found

Five rejections across four distinct pads, on the first run. The C3's is the one
the 2026-09-18 assessment named (F6); the S2's three fell out of writing this
file, and the S2's GPIO 15 is counted twice because it is shared in *both*
configurations.

| Board | Configuration | Pad | What is on it |
| --- | --- | --- | --- |
| `ESP32S2DEV` | **as written** | GPIO 15 | the status LED and servo 1 |
| `ESP32S2DEV` | fitted | GPIO 15 | the same two — the bare case again, with the sensor bus added |
| `ESP32S2DEV` | fitted | GPIO 18 | the receiver's input and the sensor bus's clock |
| `ESP32S2DEV` | fitted | GPIO 5 | the sensor's chip select and motor 2 |
| `ESP32C3DEV` | fitted | GPIO 10 | the receiver's input and the sensor's chip select |

Three of the five are the fitted configuration, which is the one nobody builds
by default and the one `scripts/esp32-proto.sh` builds. The two ARM boards are
clean in both.

**None of them is fixed here, on purpose.** The repair to four of them is a pin,
and a pin is a fact about hardware: the three fitted-only ones need somebody with
the devkit and a sensor to decide which of the two functions moves, and choosing
for them would be inventing that fact. What the check buys is that a *new*
collision fails the build while these stay visible — printed on every run, with
the reason each one is still there.

The S2's LED/servo pair is the exception and should be read first: it is not a
fitted-configuration case, it is the configuration a bare S2 devkit builds, and
it is the one to fix. That needs the board in hand, which is why it is written
down rather than changed.

## What it does not prove

This is the larger half, and it is a check on the *manifest*, not on the
hardware or the silicon:

* **Whether a pin exists, or can do the job.** GPIO 34-39 on an ESP32 are
  input-only and have no pull-ups; GPIO 6-11 on many modules are the SPI flash.
  Both are facts about a module's datasheet. A board that puts VBAT on 34 is
  correct; one that puts an LED there is not, and this cannot tell them apart.
* **Whether the peripheral is available.** Two resources can name one SPI
  instance and one timer, and nothing here counts instances or channels. Those
  limits are checked where they are used — `AK_BOARD_MOTORS` against the
  mixer's need, the RMT channels against the outputs.
* **Whether the pin reaches the pad.** A devkit's header, a soldered wire and a
  connector are not in the file. This is not only a bench question: on
  2026-09-29 the Feather F405 was driving its servos out of the arch layer's
  TIM2 pads, PA0 and PA1, and Adafruit's variant file does not define either — a
  pad on the package that the breakout never brought out. No pin *conflict*
  existed for this check to find; the collision was between the arch's choice
  and the breakout, and only the board's own `ak_board_output_shape()` can say
  it. That number is the board's statement about its header, which is exactly
  the kind of fact a manifest is for — see traps 204. The board file that makes
  that statement is now executed as well as compiled (`tests/test_board_feather.c`),
  which narrows this gap without closing it: the test checks the pads the board
  file drives, and a pad the breakout never brought out is still not in any file
  the host can read.
* **Anything about a board that is not in `src/boards/`.**

## Why it is a reader and not a program

There is no compiled per-board check here because on this machine there is
nothing to compile one with: no `IDF_PATH`, no `~/esp-idf`, no `idf.py`, no
`arm-none-eabi-*`. All seven board headers fail the host preprocessor on their
first `#include` —

    AERIALKIT_F405    board.h:4:10: fatal error: arch.h: No such file or directory
    ESP32DEV          board.h:4:10: fatal error: esp.h: No such file or directory

— so a compiled check would report "not run" for all seven boards, which is the
shape `docs/28-build.md` records for the sanitizer that cannot run on this host.

A reader, though, is exactly the kind of thing that agrees with itself. So the
parse is deliberately narrow and **refuses** what it does not understand: it
knows `#define`, `#ifdef`, `#ifndef`, `#else` and `#endif`, and the pin
spellings these seven boards use. An `#elif`, an `#if` that is not a bare name, a
value that is neither a GPIO number nor an `AK_PIN(port, pin)` pair, or a
conditional that never closes makes it stop and say so, and the process exits 2.
A validator that silently skips what it cannot read reports "no collisions" for
a board it never read — the same shape as the failure it exists to catch.

## The check's own check

`make boards-check` runs the validator against boards whose answer was decided
before they were written: eight fixtures in `tests/fixtures/boards/`, and the
seven real boards. It reports:

    board_resources_check: 36 checks, 0 failed

Three of the checks are worth naming, because they are the ones that could pass
vacuously:

* **Each real board's pin count is asserted** (22, 20, 16, 20, 18, 20, 14). A
  validator that read nothing would agree with itself perfectly, and every other
  line in that function would still pass.
* **A fixture is checked on its own**, not alongside the tree: a fixture run
  that also loaded the seven real boards would exit 1 for one of the five known
  debts, and every assertion about the fixture would be an assertion about the
  tree.
* **The four fixtures that are not collisions end in a refusal anyway** —
  `elif.h`, `computed.h`, `expression.h`, `unbalanced.h`.

And the check was falsified in both directions, the way the counters in
`docs/29-timing.md` were:

| Falsification | What was removed | Result |
| --- | --- | --- |
| A | the collision detection itself — `collisions()` returns nothing | **7 checks failed**: the fixtures stop failing and the C3 and S2 stop being reported |
| B | the `FITTED` gate — every pin treated as fitted | **2 checks failed**, including "the C3 bare does not collide": the C3 is reported broken as shipped, which is the misreading this file was written after |

## Running it

`make boards` exits 0 when the only shared pads are the five known ones, 1 when
a new one appears, and 2 when it is handed a manifest it cannot read. The full
run today, on this host:

    board_resources: 7 boards, 14 configurations, 5 known rejection(s), 0 new
    board_resources_check: 36 checks, 0 failed

`boards` is a stage of `scripts/ci.sh` too, so it runs in the whole-suite report
and its two lines appear in `ci-report/counts.txt`:

    boards:         36 checks, 0 failed
                    7 boards, 14 configurations, 5 known rejection(s), 0 new

`make test` runs the fixture half as part of the suite, which is why a change
that breaks the rule fails the ordinary build rather than only the CI report.
