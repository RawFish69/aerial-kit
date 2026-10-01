#!/usr/bin/env python3
"""Board manifests, checked against each other.

    make boards          # the six real boards
    make boards-check    # this file, against boards whose answer is known
    make boards PINS=1   # and every pin every board claims

A board header is a claim about hardware: this pin is the receiver's input,
that pin is the inertial sensor's chip select. Nothing checked those claims
against each other, so a header could give two functions one pad and every
build, test and image check in this repository would pass. Two of them do:

    ESP32C3DEV   RC_RX_GPIO 10  and  IMU_CS_GPIO 10
    ESP32S2DEV   RC_RX_GPIO 18  and  IMU_SCK_GPIO 18

The first is the one the 2026-09-18 assessment named (F6); the second fell out
of writing this file. Both are in the *fitted* configuration - a devkit with a
sensor soldered on - which is the configuration nobody builds by default and
the one `scripts/esp32-proto.sh` builds.

What it checks: no two resources on a board claim the same pad, except where
sharing is what the pad is for. A barometer and a rangefinder on one I2C pair is
not a collision, it is a bus, and the manifests say so by naming the same
peripheral (`AK_BOARD_BARO_I2C` and `AK_BOARD_RANGE_I2C` are both `I2C1_BASE` on
the F405, both port 0 on the ESP32s). So the rule is:

    two pins may share a pad only if both are bus lines of the same kind
    (SCL with SCL, MOSI with MOSI) *and* both name the same peripheral.

Everything else is a collision: a chip select, a UART line, an LED, a motor, an
ADC channel. A bus line sharing with a signal is a collision too - that is the
C3, where the receiver's input lands on the sensor's chip select.

What it does not check, and this is the larger half. It is a check on the
*manifest*, not on the hardware or the silicon:

* **Whether a pin exists, or can do the job.** GPIO 34-39 on an ESP32 are
  input-only and have no pull-ups; GPIO 6-11 on many modules are the SPI flash.
  Both are facts about a module's datasheet, not about this file, and a board
  that puts VBAT on 34 is correct while one that puts an LED there is not.
* **Whether the peripheral is available.** Two resources can name one SPI
  instance and one timer, and nothing here counts instances or channels. The
  chip's own limits are checked where they are used - `AK_BOARD_MOTORS` against
  the mixer's need, the RMT channels against the outputs.
* **Whether the pin reaches the pad.** A devkit's header, a soldered wire and a
  connector are not in the file.
* **Anything about a board that is not in `src/boards/`.**

The parse is deliberately narrow. It understands `#define`, `#ifdef`,
`#ifndef`, `#else` and `#endif`, and the pin spellings the six boards here use.
Anything else - a computed `#if`, an `#elif`, a value it cannot reduce to a
number or a `AK_PIN(port, pin)` pair - makes it **stop and say so** rather than
skip the line. A validator that silently skips what it does not understand
reports "no collisions" for a board it never read, and that is the same shape
as the failure it exists to catch (trap 38, and trap 63 for the reading
case).

`tools/board_resources_check.py` is how this file is held to that, and
`docs/30-boards.md` is the page: the rule, the five known collisions, what
this does not prove, and the two falsifications.
"""

import argparse
import os
import re
import sys

# The last word of a pin macro, after the board prefix and the resource name.
# `AK_BOARD_RC_RX` -> RX, `AK_BOARD_IMU_CS_GPIO` -> CS, `AK_BOARD_LED_PIN` ->
# PIN. The two generic ones carry no kind of their own.
PIN_TAILS = {
    "GPIO", "PIN", "TX", "RX", "SCL", "SDA", "SCK", "MISO", "MOSI", "CS",
    "DM", "DP",
}

# The lines a bus is made of, and which bus. Everything not here is a signal
# that one resource owns by itself.
SHARED = {
    "SCL": "i2c",
    "SDA": "i2c",
    "SCK": "spi",
    "MISO": "spi",
    "MOSI": "spi",
}

# What names the peripheral a resource's bus lines belong to, tried in order.
BUS_MACROS = ("_I2C", "_SPI", "_PORT")

_BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)
_LINE_COMMENT = re.compile(r"//[^\n]*")


class Refused(Exception):
    """The manifest says something this file will not guess at."""


class Pin:
    def __init__(self, macro, resource, tail, key, bus, where, fitted=True):
        self.macro = macro
        self.resource = resource
        self.tail = tail
        self.key = key
        self.bus = bus
        self.where = where
        self.fitted = fitted

    def __str__(self):
        return "%s is %s" % (self.macro, describe(self.key))


def describe(key):
    if key[0] == "gpio":
        return "GPIO %s" % key[1]
    return "%s pin %s" % (key[0].replace("_BASE", ""), key[1])


def pin_key(value, where):
    """Reduce a macro's right-hand side to something comparable.

    Two spellings in this tree: the ESP32 boards write a bare GPIO number, the
    two ARM boards write `AK_PIN(GPIOB_BASE, 13)`. Ports are compared by the
    text of the port expression, which is the same text in both boards that use
    one, so `GPIOB_BASE` on the F405 and `GPIOB_BASE` on the GHF435 are
    different boards and never meet.
    """
    text = value.strip()
    text = re.sub(r"([0-9])[uUlL]+\b", r"\1", text)
    if re.fullmatch(r"[0-9]+", text):
        return ("gpio", int(text))
    m = re.fullmatch(r"AK_PIN\(\s*([A-Za-z0-9_]+)\s*,\s*([0-9]+)\s*\)", text)
    if m:
        return (m.group(1), int(m.group(2)))
    raise Refused("%s is %r, which is neither a GPIO number nor an "
                  "AK_PIN(port, pin) pair" % (where, value.strip()))


def strip_comments(text):
    return _LINE_COMMENT.sub("", _BLOCK_COMMENT.sub("", text))


def parse(path, overrides):
    """The board's macros, with the conditionals that guard them resolved.

    Returns {name: value} for the configuration `overrides` selects. The
    conditionals in this tree are the include guard and the `#ifndef
    AK_BOARD_X_FITTED` defaults, so an override is exactly a `-D` on a build.
    """
    with open(path) as fh:
        raw = strip_comments(fh.read())

    macros = dict(overrides)
    stack = []          # (taken, kind, name, line)
    for lineno, line in enumerate(raw.splitlines(), 1):
        stripped = line.strip()
        if not stripped.startswith("#"):
            continue
        parts = stripped[1:].split(None, 1)
        if not parts:
            continue
        directive, rest = parts[0], (parts[1].strip() if len(parts) > 1 else "")

        if directive in ("ifdef", "ifndef", "if"):
            if directive == "if":
                if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", rest):
                    raise Refused("%s:%d: `#if %s` is not a bare name, and this "
                                  "file does not evaluate expressions" %
                                  (path, lineno, rest))
                taken = rest in macros
            else:
                name = rest.split()[0] if rest else ""
                if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
                    raise Refused("%s:%d: `#%s %s` is not a bare name" %
                                  (path, lineno, directive, rest))
                taken = (name in macros) == (directive == "ifdef")
            if directive == "if" and rest in ("0", "1"):
                taken = rest == "1"
            stack.append([taken, directive, rest, lineno, False])
        elif directive == "else":
            if not stack:
                raise Refused("%s:%d: `#else` with no `#if`" % (path, lineno))
            if stack[-1][3] == "else":
                raise Refused("%s:%d: a second `#else` for one `#if`" %
                              (path, lineno))
            stack[-1][0] = not stack[-1][0]
            stack[-1][4] = True
        elif directive == "elif":
            raise Refused("%s:%d: `#elif` is not understood here; the boards in "
                          "this tree do not use it" % (path, lineno))
        elif directive == "endif":
            if not stack:
                raise Refused("%s:%d: `#endif` with no `#if`" % (path, lineno))
            stack.pop()
        elif directive == "define":
            if not rest:
                raise Refused("%s:%d: `#define` with nothing after it" %
                              (path, lineno))
            bits = rest.split(None, 1)
            name = bits[0]
            if "(" in name:
                continue            # a function-like macro: not a resource
            if any(not taken for taken, _, _, _, _ in stack):
                continue            # this configuration does not define it
            macros[name] = bits[1].strip() if len(bits) > 1 else "1"
        elif directive in ("undef", "include", "pragma", "error", "warning"):
            continue
        else:
            raise Refused("%s:%d: `#%s` is not a directive this file knows" %
                          (path, lineno, directive))

    if stack:
        raise Refused("%s: %d `#if`/`#ifdef` never closed" % (path, len(stack)))
    return macros


def resources(macros, path):
    """Every pin the configuration claims, with the bus it belongs to."""
    out = []
    for name, value in sorted(macros.items()):
        if not name.startswith("AK_BOARD_"):
            continue
        parts = name[len("AK_BOARD_"):].split("_")
        if len(parts) < 2 or parts[-1] not in PIN_TAILS:
            continue
        if parts[-1] in ("GPIO", "PIN") and parts[-2] in PIN_TAILS:
            tail, resource = parts[-2], "_".join(parts[:-2])
        else:
            tail, resource = parts[-1], "_".join(parts[:-1])

        # A resource that says whether it is fitted can be declared without
        # being there: the devkit manifests carry the pins a sensor *would*
        # use, and the flag is what makes them real. On a board with
        # AK_BOARD_IMU_FITTED 0 there is no sensor on GPIO 10 and no conflict
        # either - so an unfitted resource is carried, named, and left out of
        # the comparison.
        fitted = True
        flag = "AK_BOARD_%s_FITTED" % resource
        if flag in macros:
            text = macros[flag].strip().rstrip("uUlL")
            if text not in ("0", "1"):
                raise Refused("%s in %s is %r; this file only understands 0 and "
                              "1" % (flag, path, macros[flag]))
            fitted = text == "1"

        bus = None
        for suffix in BUS_MACROS:
            macro = "AK_BOARD_%s%s" % (resource, suffix)
            if macro in macros:
                bus = (suffix, macros[macro].strip())
                break
        out.append(Pin(name, resource, tail,
                       pin_key(value, "%s in %s" % (name, path)), bus,
                       "%s:%s" % (path, name), fitted))
    return out


def collisions(pins):
    """Pairs of resources that want the same pad, excluding legal bus sharing."""
    by_key = {}
    for pin in pins:
        if not pin.fitted:
            continue
        by_key.setdefault(pin.key, []).append(pin)

    found = []
    for key, group in sorted(by_key.items(), key=lambda kv: str(kv[0])):
        if len(group) == 1:
            continue
        kinds = {SHARED.get(p.tail) for p in group}
        buses = {p.bus for p in group}
        if None not in kinds and len(kinds) == 1 and len(buses) == 1 \
                and None not in buses:
            continue                    # one bus, one kind of line: a bus
        found.append((key, group))
    return found


def explain(key, group):
    who = " and ".join("%s (%s)" % (p.macro, p.resource) for p in group)
    why = []
    for p in group:
        if p.tail not in SHARED:
            why.append("%s is a %s, which one resource owns" %
                       (p.macro, p.tail))
        elif p.bus is None:
            why.append("%s names no peripheral, so sharing it cannot be told "
                       "from a collision" % p.macro)
    if not why:
        why.append("they are bus lines of different kinds or different buses")
    return "%s are both %s: %s" % (who, describe(key), "; ".join(why))


def load(boards_dir, only=None):
    boards = []
    for name in sorted(os.listdir(boards_dir)):
        header = os.path.join(boards_dir, name, "board.h")
        if os.path.isfile(header) and (only is None or name in only):
            boards.append((name, header))
    return boards


def configurations(header):
    """The variants a board is checked in: as written, and fully fitted."""
    with open(header) as fh:
        text = strip_comments(fh.read())
    fitted = sorted(set(re.findall(r"#define\s+(AK_BOARD_[A-Z0-9_]*_FITTED)\b",
                                   text)))
    out = [("as written", {})]
    if fitted:
        out.append(("all parts fitted", {name: "1" for name in fitted}))
    return out


def check_board(header, overrides):
    macros = parse(header, overrides)
    pins = [p for p in resources(macros, header) if p.fitted]
    return pins, collisions(pins)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--boards-dir", default=None)
    ap.add_argument("--board", action="append", default=None,
                    help="check only this board (repeatable)")
    ap.add_argument("--header", action="append", default=None,
                    help="check a single manifest by path (repeatable); for "
                         "fixtures, which are files rather than board "
                         "directories")
    ap.add_argument("--show-pins", action="store_true",
                    help="print every pin every board claims")
    args = ap.parse_args(argv)

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    boards_dir = args.boards_dir or os.path.join(root, "src", "boards")

    # A `--header` names a manifest the caller chose, so it replaces the
    # directory rather than joining it: a fixture checked alongside the six real
    # boards would exit 1 for a reason that has nothing to do with the fixture,
    # and every assertion about it would be an assertion about the tree.
    boards = []
    if args.header is None or args.boards_dir is not None:
        boards = load(boards_dir, args.board)
    for header in args.header or []:
        boards.append((os.path.splitext(os.path.basename(header))[0], header))

    known = KNOWN_REJECTIONS
    new, expected = [], []
    checked = 0
    for name, header in boards:
        for variant, overrides in configurations(header):
            checked += 1
            try:
                pins, found = check_board(header, overrides)
            except Refused as exc:
                print("board_resources: REFUSED %s (%s): %s" %
                      (name, variant, exc))
                return 2
            if args.show_pins:
                for pin in pins:
                    print("  %-16s %-34s %s" % (name, pin.macro,
                                                describe(pin.key)))
            for key, group in found:
                entry = (name, variant, key, group)
                if key in known.get(name, {}):
                    expected.append((entry, known[name][key]))
                else:
                    new.append(entry)

    for entry, reason in expected:
        name, variant, key, group = entry
        print("board_resources: known  %s (%s): %s" %
              (name, variant, explain(key, group)))
        print("                          %s" % reason)
    for name, variant, key, group in new:
        print("board_resources: FAIL   %s (%s): %s" %
              (name, variant, explain(key, group)))

    print("board_resources: %d boards, %d configurations, %d known rejection(s),"
          " %d new" % (len(boards), checked, len(expected), len(new)))
    if new:
        print("board_resources: a pad two resources want is a board that cannot "
              "work; see tools/board_resources.py for what this does and does "
              "not prove", file=sys.stderr)
        return 1
    return 0


# The collisions this tree knows about and has not fixed, by board and pad.
#
# None of them is fixed *on purpose*. The repair is a pin, and a pin is a fact
# about hardware: the three fitted-only ones need somebody with the devkit and a
# sensor to decide which of the two functions moves, and choosing for them would
# be inventing that fact. What this list buys is that a *new* collision fails the
# build while these stay visible - printed on every run, with the reason they are
# still here - which is the shape `docs/28-build.md` uses for the sanitizer that
# cannot run on this host.
#
# The fourth is different from the other three and should be read first: it is
# not a fitted-configuration collision, it is the configuration a *bare* S2
# devkit builds, and it is the one to fix.
KNOWN_REJECTIONS = {
    "ESP32C3DEV": {
        ("gpio", 10):
            "fitted only, and the assessment's own example (F6): the receiver's "
            "input and the sensor's chip select on one pad. The C3 has two UARTs "
            "and one of them is the console, so the receiver is on the only port "
            "left; GPIO 10 is where the header puts the sensor's chip select. "
            "The bare devkit is fine - this appears the moment a sensor is "
            "soldered on.",
    },
    "ESP32S2DEV": {
        ("gpio", 15):
            "**the one that is broken as shipped**: the status LED and the first "
            "servo output, on a board that declares `AK_BOARD_SERVOS 2u`. This "
            "is not a fitted-configuration case - a bare S2 devkit builds this "
            "way - so driving servo 1 drives the LED line, and the fix is one "
            "pin or the other, which needs the board in hand.",
        ("gpio", 18):
            "fitted only: the receiver's input and the sensor bus's clock. The "
            "S2's RC section and its IMU section were written from two devkit "
            "pinouts and never read together; this class is the C3's.",
        ("gpio", 5):
            "fitted only: the sensor's chip select and the second motor. Found "
            "by this file along with the two above, which is the argument for "
            "having it: three collisions on one board, none of them reachable "
            "by any build, test or review this repository runs.",
    },
}


if __name__ == "__main__":
    sys.exit(main())
