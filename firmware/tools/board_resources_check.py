#!/usr/bin/env python3
"""tools/board_resources.py, held to boards whose answer is known.

    make boards-check

The validator makes one claim - no two resources on a board want the same pad,
except where the pad is a bus - and it makes it by reading manifests rather than
by compiling anything, because on this machine nothing compiles: the ESP32
headers want the IDF, the ARM headers want CMSIS, and neither is installed. A
reader is exactly the kind of thing that agrees with itself, so every check here
is either a real board with a known answer or a fixture whose answer was decided
before the fixture was written.

**The four fixture boards that must fail.** `clash.h` is the plain case and the
one that proves the check is looking at all: an LED and a motor on one pad.
`twobuses.h` is the legal-sharing rule read backwards - the barometer and the
rangefinder on the same two pins but naming *different* I2C peripherals - and it
exists because "both lines are SCL" would otherwise be enough to allow anything.
`fitted.h` is the ESP32C3DEV's shape: fine bare, colliding when a sensor is
soldered on. `elif.h`, `computed.h`, `expression.h` and `unbalanced.h` are not
collisions at all; they are the four ways a manifest could say something this
reader does not understand, and each has to end in a refusal, because a checker
that skips what it cannot read answers "no collisions" for a board it never
read.

**And the tree's own boards**, which are the real subject: the two ARM boards
and the two ESP32 boards that are not the C3 or the S2 must come back clean in
both configurations, and the C3 must come back clean *bare* and colliding
*fitted*. That last pair is what the FITTED gate is for and it is the check that
would have caught the misreading this file was written after: on the first run,
before the flag was honoured, the C3 was reported broken as shipped, which is
not true of a devkit with nothing soldered to it.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIXTURES = os.path.join(ROOT, "tests", "fixtures", "boards")
VALIDATOR = os.path.join(HERE, "board_resources.py")

sys.path.insert(0, HERE)
import board_resources as br          # noqa: E402

failures = 0
checks = 0


def expect(name, condition, detail=""):
    global failures, checks
    checks += 1
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def run(fixture, *args):
    """The validator over one fixture manifest.

    A fixture is a file, not a board directory: nothing here should be mistaken
    for something a build could use, and `--header` is how it is pointed at one.
    """
    proc = subprocess.run([sys.executable, VALIDATOR, "--header",
                           os.path.join(FIXTURES, fixture)] + list(args),
                          capture_output=True, text=True, cwd=ROOT)
    return proc


def check_a_board_that_must_fail():
    proc = run("clash.h")
    expect("a fixture board with two signals on one pad fails", proc.returncode == 1,
           "(exit %d)" % proc.returncode)
    expect("and the two macros are named in the failure",
           "AK_BOARD_LED_GPIO" in proc.stdout and "AK_BOARD_MOTOR1_GPIO" in proc.stdout)


def check_the_legal_sharing_rule():
    proc = run("clean.h")
    expect("two I2C devices on one pair, naming one peripheral, is a bus",
           proc.returncode == 0, "(exit %d)" % proc.returncode)
    proc = run("twobuses.h")
    expect("the same two pins naming two peripherals is not", proc.returncode == 1,
           "(exit %d)" % proc.returncode)


def check_the_fitted_gate():
    """The fixture is checked twice, and the two answers differ.

    Read the output per configuration rather than the exit code: this fixture
    *should* fail, in its fitted configuration, so the run as a whole exits 1
    and an exit-code check here could not tell the bare case passing from the
    bare case never running.
    """
    proc = run("fitted.h")
    lines = proc.stdout.splitlines()
    bare = [l for l in lines if l.startswith("board_resources: FAIL")
            and "(as written)" in l]
    expect("a bare board does not collide with a sensor that is not fitted",
           not bare, bare[0][:70] if bare else "")
    # `--header` has to *replace* the boards directory. Adding to it instead
    # would leave this run exiting 1 for one of the tree's five known debts, and
    # every fixture check above would then be reading the tree's answer rather
    # than the fixture's.
    expect("and a header is checked on its own, not alongside the tree",
           "1 boards," in proc.stdout or " 1 board," in proc.stdout,
           proc.stdout.strip().splitlines()[-1][:70])
    fitted = [l for l in lines if l.startswith("board_resources: FAIL")
              and "all parts fitted" in l]
    expect("and the collision appears once the sensor is fitted", bool(fitted),
           fitted[0][:70] if fitted else "no fitted configuration was reported")
    expect("so the fixture fails, for that reason only", proc.returncode == 1,
           "(exit %d)" % proc.returncode)


def check_it_refuses_what_it_cannot_read():
    for fixture, why in (
            ("elif.h", "#elif"),
            ("computed.h", "an #if with an expression"),
            ("expression.h", "a pin that is not a number"),
            ("unbalanced.h", "a conditional that never closes"),
    ):
        proc = run(fixture)
        expect("%s is refused rather than skipped" % why, proc.returncode == 2,
               "(exit %d)" % proc.returncode)
        expect("and %s says what it did not understand" % fixture,
               "REFUSED" in proc.stdout or "REFUSED" in proc.stderr,
               (proc.stdout + proc.stderr).strip().splitlines()[-1][:70])


def check_the_TREES_OWN_boards():
    """The boards that must be clean, and the ones that are known not to be.

    The count below is pinned on purpose: adding a board to `src/boards/` is a
    deliberate act, and a new header that the validator silently skipped would
    otherwise be a board whose collisions nobody looked for. It went 6/12 to
    7/14 when `FEATHER_F405` arrived - the same part as the WeAct board on
    Adafruit's breakout, whose header brought one more board and one more
    fitted configuration with it.
    """
    proc = subprocess.run([sys.executable, VALIDATOR], capture_output=True, text=True,
                          cwd=ROOT)
    expect("every board in src/boards/ is read", "7 boards, 14 configurations" in
           proc.stdout, proc.stdout.strip().splitlines()[-1][:70])
    expect("and no collision is unknown to the tree", proc.returncode == 0,
           "(exit %d)" % proc.returncode)

    expect("the C3 bare does not collide",
           "ESP32C3DEV (as written)" not in proc.stdout)
    expect("the C3 fitted does",
           "ESP32C3DEV (all parts fitted)" in proc.stdout)
    expect("the S2's LED and servo collide as shipped",
           "ESP32S2DEV (as written)" in proc.stdout)

    for board in ("AERIALKIT_F405", "AERIALKIT_GHF435", "ESP32DEV",
                  "ESP32S3DEV"):
        for variant in ("as written", "all parts fitted"):
            expect("%s (%s) is clean" % (board, variant),
                   "%s (%s)" % (board, variant) not in proc.stdout)

    # Not a check on the validator but on the claim above it: a board whose pins
    # are not read at all would pass every line in this function.
    seen = {}
    for board, header in br.load(os.path.join(ROOT, "src", "boards")):
        for variant, overrides in br.configurations(header):
            pins, _ = br.check_board(header, overrides)
            seen[board] = max(seen.get(board, 0), len(pins))
    for board, count in sorted(seen.items()):
        expect("%s's pins were read, not skipped" % board, count >= 10,
               "(%d pins)" % count)


def main():
    print("board_resources_check: the validator, against boards with known answers")
    check_a_board_that_must_fail()
    check_the_legal_sharing_rule()
    check_the_fitted_gate()
    check_it_refuses_what_it_cannot_read()
    check_the_TREES_OWN_boards()
    print("board_resources_check: %d checks, %d failed" % (checks, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
