#!/usr/bin/env python3
"""Every name the firmware registers has to fit the buffer a load reads it into.

The bug this exists to prevent was measured, not imagined.

`ak_params_deserialize` reads a name out of the saved record into a buffer of
AK_PARAM_NAME_MAX and looks it up by exact match. A name longer than that was
truncated *before* the lookup, never matched, and was counted as a name from a
newer build - so the parameter kept its default through every power cycle while
the load returned success and the boot report blamed the record. `set
arm_accel_lpf_hz 40` took, `save` took, and the value came back as 25.000.

Eleven of the ninety-two names this board registers were over the old limit of
16. A host test can only walk the table it builds, and the board's table is
built in `ak_firmware_main`, so this reads the registration sites themselves -
every board's, not the one the host binary happens to build.

The source read is a *superset*: it also finds the four `wifi_*` names the radio
build registers in `net.c`, which no F405 calls. So the read alone cannot say it
is complete, and `--board` is the half that can: it asks the built firmware which
names it has and requires every one to have been found here. That is not
decoration. The first version of this file read a flat `const char *names[3]` as
one row of rows and took `row[0]`, so it found one name out of three, missed six
of the board's ninety-two, and reported a clean tree - which is the failure this
whole file is about.

**It refuses to guess.** An argument it cannot evaluate is an error and not a
skip: a checker that shrugs at a syntax it has not seen reports a clean tree
and means nothing by it. That is the failure mode this file exists to avoid in
the firmware, so it is not one to reproduce here.

    python3 tools/param_name_check.py [--self-test]
"""

from __future__ import annotations

import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / "src"

# The limit is read from the header rather than written here, so the two cannot
# drift: raising it in ak_params.h is the one act that changes what fits.
HEADER = SRC / "core" / "ak_params.h"
LIMIT_DEFINE = re.compile(r"^#define\s+AK_PARAM_NAME_MAX\s+(\d+)\s*$", re.M)

# The registration functions, however they are reached. `add_float` and its
# siblings are the short macros ak_flight.c's table is written with; naming both
# spellings is what makes this see the whole table and not half of it.
CALL = re.compile(r"\b(?:ak_params_add_(?:float|u32|text)|add_float|add_u32|add_text)\s*\(")

# A name built by a loop, from an array literal: `names[i][1]`, `lat_name[i]`.
LOOP_INDEXED = re.compile(r"(\w+)\s*\[\s*i\s*\]\s*\[\s*(\d+)\s*\]\s*$")
LOOP_FLAT = re.compile(r"(\w+)\s*\[\s*i\s*\]\s*$")

ARRAY_DECL = re.compile(
    r"\bchar\b[^;=]*?\b(\w+)\s*\[[^]]*\]\s*(?:\[[^]]*\]\s*)*=\s*\{")


class Unreadable(Exception):
    """A registration site this checker cannot read. Never a pass."""


def limit() -> int:
    text = HEADER.read_text()
    match = LIMIT_DEFINE.search(text)
    if not match:
        raise Unreadable(f"no AK_PARAM_NAME_MAX in {HEADER}")
    return int(match.group(1))


def strip_comments(text: str) -> str:
    """Comments out, and every newline kept.

    A block comment replaced by a single space collapses the lines inside it and
    slides everything after it upward, so every line number this reports would
    be short by however many lines the comment above it happened to have. A
    check that sends you to the wrong line is worse than one that says nothing.
    """
    def blank(match: re.Match) -> str:
        return "\n" * match.group(0).count("\n")

    return re.sub(r"//[^\n]*", "", re.sub(r"/\*.*?\*/", blank, text, flags=re.S))


def arguments(text: str, open_paren: int) -> tuple[list[str], int]:
    """Top-level comma-separated arguments, and the index of the closing paren."""
    spans, depth, i, begin = [], 0, open_paren, open_paren + 1
    while i < len(text):
        char = text[i]
        if char == '"':
            i += 1
            while i < len(text) and text[i] != '"':
                i += 2 if text[i] == "\\" else 1
        elif char in "([{":
            depth += 1
        elif char in ")]}":
            depth -= 1
            if depth == 0:
                spans.append((begin, i))
                return [text[a:b].strip() for a, b in spans], i
        elif char == "," and depth == 1:
            spans.append((begin, i))
            begin = i + 1
        i += 1
    raise Unreadable("a registration call that never closes")


def arrays(roots: list[pathlib.Path]) -> dict[str, list[list[str]]]:
    """Every string-array literal in the tree, so a loop-built name can be read.

    The rows are what the braces say, so the shape is kept deliberately: a flat
    `const char *bias_names[3]` is *one* row of three names, while a nested
    `const char *names[2][3]` is two rows of three. Reading a flat array as rows
    and taking `row[0]` is how this checker first ran and it found one name out
    of three - which is the exact failure it exists to catch, so the two shapes
    are told apart here rather than guessed at below.
    """
    found: dict[str, list[list[str]]] = {}
    for path in roots:
        try:
            text = strip_comments(path.read_text())
        except OSError:
            continue
        for match in ARRAY_DECL.finditer(text):
            open_brace = text.index("{", match.start())
            depth, i = 0, open_brace
            while i < len(text):
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                    if depth == 0:
                        break
                i += 1
            body = text[open_brace + 1:i]
            rows = re.findall(r"\{([^{}]*)\}", body)
            # A flat array has no inner braces; its whole body is the one row.
            found[match.group(1)] = (
                bool(rows),
                [re.findall(r'"([^"]*)"', row) for row in (rows or [body])],
            )
    return found


def names_at(path: pathlib.Path, text: str,
             table: dict[str, list[list[str]]]) -> list[tuple[str, int]]:
    """Every (name, line) a registration in this file states."""
    out: list[tuple[str, int]] = []
    for match in CALL.finditer(text):
        line_start = text.rfind("\n", 0, match.start()) + 1
        if text[line_start:match.start()].lstrip().startswith("#define"):
            continue  # the macro's own definition; its call sites are found
        args, close = arguments(text, match.end() - 1)
        # A definition's body opens right after its argument list; a call ends
        # in `;` or continues an expression. `ak_params_add_float`'s own
        # definition is spelled like a call site, so without this the three
        # definitions in ak_params.c read as three unreadable names.
        if text[close + 1:].lstrip()[:1] == "{":
            continue
        line = text.count("\n", 0, match.start()) + 1
        where = f"{path.name}:{line}"
        if len(args) < 3:
            raise Unreadable(f"{where}: a registration with fewer than three "
                             f"arguments: {'(' + ', '.join(args) + ')'}")
        third = args[2]

        if third.startswith('"'):
            out.append((third[1:-1], line))
            continue

        # `names[i][1]`: the name is element `1` of each row, which only means
        # anything for an array whose rows are the braces.
        indexed = LOOP_INDEXED.search(third)
        if indexed:
            source, column = indexed.group(1), int(indexed.group(2))
            if source not in table:
                raise Unreadable(f"{where}: no array {source!r} to read "
                                 f"{third!r} from")
            nested, rows = table[source]
            if not nested:
                raise Unreadable(f"{where}: {source} is a flat array, so "
                                 f"{third!r} indexes a character of a string "
                                 f"and not a name")
            for row in rows:
                if column < len(row):
                    out.append((row[column], line))
            continue

        # `names[i]`: the name is each element of a flat array. On a nested one
        # this is a row rather than a name, and a row is not something that can
        # be checked against a length, so it is refused instead of approximated.
        flat = LOOP_FLAT.search(third)
        if flat and flat.group(1) in table:
            nested, rows = table[flat.group(1)]
            if nested:
                raise Unreadable(f"{where}: {flat.group(1)} has rows, so "
                                 f"{third!r} is a row of names and not one "
                                 f"name - spell out the column")
            for row in rows:
                for name in row:
                    out.append((name, line))
            continue

        raise Unreadable(f"{where}: cannot read a parameter name out of "
                         f"{third!r}. Teach this checker the syntax rather "
                         f"than letting it pass - a name it cannot read is a "
                         f"name it cannot bound.")

    return out


def source_names(root: pathlib.Path) -> dict[str, list[str]]:
    """Every name a registration in the source spells, and where."""
    paths = sorted(root.rglob("*.c")) + sorted(root.rglob("*.h"))
    table = arrays(paths)
    found: dict[str, list[str]] = {}
    for path in sorted(root.rglob("*.c")):
        text = strip_comments(path.read_text())
        for name, line in names_at(path, text, table):
            found.setdefault(name, []).append(
                f"{path.relative_to(root.parent)}:{line}")
    return found


def board_names(binary: pathlib.Path) -> list[str]:
    """The names a built board actually registers, asked over the protocol.

    A static read of the source is a *superset* of any one board's table - the
    radio build's four `wifi_*` names are registered in `net.c` and no F405
    calls them - so the source set alone cannot say whether it is complete. It
    can say only that it is complete *if* nothing was missed, which is the
    assumption this function removes: every name the board reports must be one
    the source read found, and the difference is the measurement.
    """
    import os
    import sys as _sys

    if not binary.exists():
        raise Unreadable(
            f"no board at {binary} to ask - `make host` builds it, and "
            f"without it the source read is a number and not a measurement")

    _sys.path.insert(0, str(HERE))
    os.environ.setdefault("AK_SIM", str(binary))
    import bench_port_check
    bench_port_check.SIM = str(binary)
    import akproto

    cable = bench_port_check.Cable()
    try:
        cable.start_simulator()
        client = akproto.open_client(_Namespace(host=None, port=cable.device))
        count = akproto.parse_hello(client.request(akproto.HELLO))["count"]
        names = []
        for index in range(count):
            payload = client.request(akproto.PARAM_GET, bytes([index]))
            if payload[0] != 0:
                continue
            names.append(akproto.c_string(payload, 1))
        return names
    finally:
        cable.close()


class _Namespace:
    """What `akproto.open_client` reads off an argparse result: two fields."""

    def __init__(self, host, port):
        self.host = host
        self.port = port


def report(root: pathlib.Path, out, board: pathlib.Path | None = None) -> int:
    top = limit()
    found = source_names(root)
    long_names = sorted((n for n in found if len(n) >= top),
                        key=lambda n: -len(n))

    print(f"param_name_check: {len(found)} name(s) registered in the source, "
          f"AK_PARAM_NAME_MAX is {top}", file=out)

    if long_names:
        print(f"\n  {len(long_names)} name(s) too long to be read back out of "
              f"a saved record:", file=out)
        for name in long_names:
            print(f"    {len(name):2d}  {name:<24} {found[name][0]}", file=out)
        print(f"\n  A name is read into {top} bytes and matched exactly. One "
              f"longer than that\n  is truncated, never matches, and takes its "
              f"parameter's value with it -\n  silently, on every boot, while "
              f"the load reports success.", file=out)
        return 1

    print(f"  ok   every name the source registers fits {top} bytes, so every "
          f"one of them can be loaded", file=out)

    if board is None:
        print("  note no board was named, so nothing confirms this read is "
              "complete", file=out)
        return 0

    # The half that is a measurement rather than a reading: a board says which
    # names exist, and every one of them has to have been in the source set.
    # Without this the check passes exactly as well on a syntax it silently
    # failed to read - which is the failure it was written for.
    known = set(board_names(board))
    missed = sorted(n for n in known if n not in found)
    print(f"  {'ok  ' if not missed else 'FAIL'} {board.name} registers "
          f"{len(known)} name(s), and the source read found "
          f"{len(known) - len(missed)} of them", file=out)
    if missed:
        print(f"\n  the board registers name(s) no registration in the source "
              f"accounts for:", file=out)
        for name in missed:
            print(f"    {len(name):2d}  {name}", file=out)
        print(f"\n  Either a registration is spelled some way this checker "
              f"cannot read -\n  which is a gap in the checker and not in the "
              f"firmware - or a name is\n  arriving from somewhere other than "
              f"the table.", file=out)
        return 1

    extra = sorted(n for n in found if n not in known)
    if extra:
        # Not a failure: the source is a superset by design, and the names it
        # holds that this board does not are the other boards'.
        print(f"  ok   the {len(extra)} name(s) the source has and this board "
              f"does not are another build's: {', '.join(extra)}", file=out)
    return 0


SELF_TEST_SOURCE = """
static ak_param_t items[2];
static float gain;
unsigned build(ak_param_t *items, unsigned n)
{
    n = ak_params_add_float(items, n, NAME, "help", &gain, 1,
                            0.0f, 1.0f, AK_PARAM_GROUP_RATES);
    return n;
}
"""


def self_test() -> int:
    """The check, against sources it has to fail on and one it must not.

    A check that has never been shown to fail is a check nobody can tell apart
    from a function that returns zero, so the three cases below are the whole
    argument for trusting the fourth: exactly at the limit, one short of it,
    and a name written some way this checker has not been taught.
    """
    import io
    import tempfile

    top = limit()
    failures = 0
    cases: list[tuple[str, str, int]] = [
        # (what it is, the third argument as written, the exit status it must give)
        ("a name exactly at the limit",
         '"' + "n" * top + '"', 1),
        ("a name one character short of it",
         '"' + "n" * (top - 1) + '"', 0),
        ("a name this checker cannot read",
         "name_from_somewhere_else(i)", 1),
    ]

    with tempfile.TemporaryDirectory() as directory:
        root = pathlib.Path(directory)
        (root / "src" / "core").mkdir(parents=True)
        (root / "src" / "core" / "ak_params.h").write_text(
            f"#define AK_PARAM_NAME_MAX  {top}\n")

        for what, third, want in cases:
            (root / "src" / "t.c").write_text(
                SELF_TEST_SOURCE.replace("NAME", third))
            # A refusal to read is a failure like any other, so it is caught
            # here rather than allowed to end the self-test early and skip the
            # cases after it.
            try:
                got = report(root / "src", io.StringIO())
            except Unreadable:
                got = 1
            if got != want:
                print(f"  self-test: {what} gave {got}, wanted {want}",
                      file=sys.stderr)
                failures += 1

    print(f"  self-test: {len(cases)} case(s), a long name refused, a name "
          f"that fits taken, and\n  an unreadable one refused rather than "
          f"skipped  {'ok' if failures == 0 else 'FAILED'}")
    return failures


def main(argv: list[str]) -> int:
    if "--self-test" in argv:
        return 1 if self_test() else 0

    board = None
    if "--board" in argv:
        try:
            board = pathlib.Path(argv[argv.index("--board") + 1])
        except IndexError:
            print("param_name_check: --board needs the binary to ask",
                  file=sys.stderr)
            return 1

    try:
        return report(SRC, sys.stdout, board)
    except Unreadable as error:
        print(f"param_name_check: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
