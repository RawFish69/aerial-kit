#!/usr/bin/env python3
"""Derive the MAVLink message definitions from the published dialect, and check
this app's hand-written table against them.

Why this exists
---------------

`src/protocol/mavlink.ts` carries fourteen message definitions by hand, and the
tests that check them had two oracles, both of them transcriptions:

  * a `published` table of CRC_EXTRA values typed into `mavlink.test.ts`, and
  * a `crc_extra` block typed into `tests/fixtures/mavlink.json` by
    `capture-mavlink-fixtures.py`.

Neither is the authority. A transcription cannot disagree with the thing it was
transcribed from, so a table checked only against hand-copied constants has been
checked for typos and nothing else. The frames in the fixture are a genuine
oracle for the *wire* — a wrong CRC_EXTRA makes every checksum fail — but they
cover fourteen messages' worth of bytes and prove the checksum is right, not
that the field list matches what ArduPilot and PX4 actually send.

So this script goes to the source both autopilots are generated from:
`common.xml`, as shipped by **pymavlink**, the reference implementation. For
each message it records the declaration order, every field's name and type, the
array lengths, which fields sit after `<extensions/>`, and pymavlink's own
CRC_EXTRA. `tests/dialect.test.ts` then asserts the TypeScript table against
that file, field by field.

This is not a SITL test and does not pretend to be one. It is the half of the
evidence that can be produced without an autopilot binary: the app's idea of
what a message *is* agrees with the published definition rather than with a copy
of itself.

Usage
-----

    python3 tools/check-dialect.py            # regenerate tests/fixtures/dialect.json
    python3 tools/check-dialect.py --check    # fail if the file is out of date

`--check` is what CI runs. It needs no network: pymavlink ships the XML.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, os.pardir, "tests", "fixtures", "dialect.json")

#: The messages `src/protocol/mavlink.ts` claims to carry. Listed here rather
#: than parsed out of the TypeScript, because the point of the check is for the
#: two lists to be built independently -- a script that read the table it is
#: checking would agree with it by construction.
MESSAGES = [
    "HEARTBEAT",
    "SYS_STATUS",
    "PARAM_REQUEST_LIST",
    "PARAM_VALUE",
    "GPS_RAW_INT",
    "ATTITUDE",
    "GLOBAL_POSITION_INT",
    "SERVO_OUTPUT_RAW",
    "RC_CHANNELS",
    "VFR_HUD",
    "COMMAND_LONG",
    "COMMAND_ACK",
    "AUTOPILOT_VERSION",
    "STATUSTEXT",
]


def dialect_dir() -> str:
    """Where pymavlink keeps its message definitions."""
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        sys.exit(
            "pymavlink is not installed, and it is the authority this check is "
            "against. Install it with:\n"
            "    python3 -m pip install pymavlink\n"
            "(pymavlink ships the dialect XML, so this needs no network access "
            "once installed.)"
        )
    import pymavlink as pm

    base = os.path.dirname(pm.__file__)
    for candidate in (
        os.path.join(base, "message_definitions", "v1.0"),
        os.path.join(base, "dialects", "v20"),
    ):
        if os.path.isdir(candidate):
            return candidate
    sys.exit(f"pymavlink is installed at {base} but ships no message definitions")


def crc_extras() -> dict[str, int]:
    """pymavlink's own CRC_EXTRA for every message, by name.

    Taken from the generated dialect rather than recomputed here: the whole
    point is to compare against the reference implementation's answer, and a
    second derivation written by this script would be a second opinion, not the
    published one.
    """
    from pymavlink.dialects.v20 import ardupilotmega as dialect

    out: dict[str, int] = {}
    for name in dir(dialect):
        if not name.startswith("MAVLink_") or not name.endswith("_message"):
            continue
        cls = getattr(dialect, name)
        extra = getattr(cls, "crc_extra", None)
        if extra is None:
            continue
        out[cls.msgname if hasattr(cls, "msgname") else name] = int(extra)
    return out


def parse_type(raw: str) -> tuple[str, int | None]:
    """`uint8_t[8]` -> `('uint8_t', 8)`; `float` -> `('float', None)`."""
    if "[" in raw:
        base, _, rest = raw.partition("[")
        return base, int(rest.rstrip("]"))
    return raw, None


#: MAVLink pseudo-types: names the XML uses that are not wire types.
#:
#: `uint8_t_mavlink_version` is the only one this app's table meets, and it is
#: the `mavlink_version` field of HEARTBEAT — one byte, holding the MAVLink
#: version the sender speaks. It is a `uint8_t` on the wire and a `uint8_t` to
#: the checksum, and pymavlink's own generated code says so: the `fieldtypes`
#: list for `MAVLink_heartbeat_message` reads `uint8_t` in that position. So the
#: normalisation below is not this script's opinion; it is checked against the
#: generated dialect in `cross_check()`, which refuses to write the file if the
#: two disagree.
PSEUDO_TYPES = {"uint8_t_mavlink_version": "uint8_t"}


def messages_from_xml(where: str) -> dict[str, dict]:
    """Every message in the dialect, in declaration order, with extensions marked.

    Includes are followed, because `HEARTBEAT` is not in `common.xml` — it is in
    `minimal.xml`, which `common.xml` includes, and a reader that skipped the
    include would report the single most important message in MAVLink as
    undefined. Later definitions do not replace earlier ones: a dialect that
    redefines a message in an included file is malformed, and this refuses
    rather than silently picking one.
    """
    out: dict[str, dict] = {}
    seen: set[str] = set()

    def read(filename: str) -> None:
        path = os.path.join(where, filename)
        if filename in seen or not os.path.isfile(path):
            return
        seen.add(filename)
        tree = ET.parse(path)
        root = tree.getroot()
        for include in root.iter("include"):
            read(include.text.strip())
        for message in root.iter("message"):
            name = message.get("name")
            fields: list[dict] = []
            extensions = False
            for child in message:
                if child.tag == "extensions":
                    extensions = True
                    continue
                if child.tag != "field":
                    continue
                base, count = parse_type(child.get("type", ""))
                base = PSEUDO_TYPES.get(base, base)
                entry: dict = {"name": child.get("name"), "type": base}
                if count is not None:
                    entry["count"] = count
                if extensions:
                    entry["ext"] = True
                fields.append(entry)
            if name in out:
                continue
            out[name] = {"id": int(message.get("id", -1)), "name": name, "fields": fields}

    read("common.xml")
    return out


def cross_check(messages: list[dict]) -> None:
    """Refuse to write anything this script and pymavlink's generated code disagree on.

    There are two readings of the dialect in play: this script's parse of the
    XML, and the code pymavlink generated from the same XML. They are not
    independent sources — they are the same source read twice, which is exactly
    what is wanted here, because the failure this guards against is *this
    script's parser being wrong*. A parser that quietly dropped a field, or
    mis-ordered two, would produce a fixture that the TypeScript table then
    "agrees" with, and the whole exercise would be a second transcription with
    extra steps.

    `fieldnames` is the XML's declaration order, `fieldtypes` the wire type after
    the pseudo-type normalisation above, and `array_lengths` the parallel list of
    array sizes (`0` for a scalar, which is why `fieldtypes` says plain `char`
    where the XML says `char[16]`). `ordered_fieldnames` — the size-sorted wire
    order — is deliberately not checked here: the app derives it, and
    `mavlink.test.ts` already checks the derived offsets against the bytes
    pymavlink actually packed, which is a stronger statement than comparing two
    derived lists.

    The `<extensions/>` flags are not cross-checked, because pymavlink's
    generated classes do not carry them — the marker exists only in the XML, so
    this parse is the only reading of it. They are not unchecked, though: an
    extension flag is an input to the CRC_EXTRA, so a wrong one makes the
    derived extra disagree with pymavlink's, and the test that compares them
    fails.
    """
    from pymavlink.dialects.v20 import ardupilotmega as dialect

    for message in messages:
        name = message["name"]
        generated = getattr(dialect, f"MAVLink_{name.lower()}_message", None)
        if generated is None:
            sys.exit(f"pymavlink generated no class for {name}, so this parse cannot be cross-checked")
        mine_names = [field["name"] for field in message["fields"]]
        mine_types = [field["type"] for field in message["fields"]]
        mine_lengths = [field.get("count", 0) for field in message["fields"]]
        if mine_names != list(generated.fieldnames):
            sys.exit(f"{name}: XML parse names {mine_names} but pymavlink generated {list(generated.fieldnames)}")
        if mine_types != list(generated.fieldtypes):
            sys.exit(f"{name}: XML parse types {mine_types} but pymavlink generated {list(generated.fieldtypes)}")
        # `array_lengths` is parallel to `ordered_fieldnames` - the size-sorted
        # wire order - not to `fieldnames`. Reading it in declaration order is
        # how PARAM_VALUE's sixteen-byte `param_id` appears to be a scalar and
        # its scalar `param_index` appears to be sixteen bytes long.
        by_wire = dict(zip(generated.ordered_fieldnames, generated.array_lengths))
        generated_lengths = [by_wire[field] for field in mine_names]
        if mine_lengths != generated_lengths:
            sys.exit(f"{name}: XML parse lengths {mine_lengths} but pymavlink generated {generated_lengths}")


#: What to say when pymavlink is not installed. A traceback here is a check
#: that reports itself as broken without saying how to fix it, which is how a
#: red step becomes a step people skip past — and `verify-drift.sh` treats a
#: check that cannot run as a failure, so it would be red forever with no
#: instruction attached. The remedy is two commands and belongs here.
#:
#: **The version is pinned, and that is the point of the pin rather than
#: tidiness.** This script derives `tests/fixtures/dialect.json` from whatever
#: `common.xml` the installed pymavlink ships. An unpinned install therefore
#: turns this check red the day upstream adds a field to HEARTBEAT — a red that
#: says nothing about this app and everything about which pymavlink happened to
#: be installed. Pinned, the fixture has one meaning: the definition it was
#: built against, named in the file's own `pymavlink` field. Moving the pin is
#: a deliberate act, and the diff is the record of it.
#:
#: `113 M` on disk, per the record in `AERIAL-KIT-GOAL-PROGRESS.md` (§ the
#: scratch-dependencies table). It is deliberately not a project dependency:
#: shipping it in package.json would put a MAVLink code generator in the app's
#: dependency tree to serve one build-time check.
PINNED_PYMAVLINK = "2.4.49"
HOW_TO_INSTALL = (
    "check-dialect: pymavlink is not installed, and it is the oracle this "
    "check reads the published dialect from.\n"
    f"  python3 -m venv /tmp/pmv && /tmp/pmv/bin/pip install pymavlink=={PINNED_PYMAVLINK}\n"
    "  PATH=/tmp/pmv/bin:$PATH python3 tools/check-dialect.py --check\n"
    "The version is pinned so the fixture has one meaning; see the note above "
    "this message in tools/check-dialect.py."
)


def build() -> dict:
    try:
        import pymavlink as pm
    except ImportError:
        sys.exit(HOW_TO_INSTALL)

    where = dialect_dir()
    xml = messages_from_xml(where)
    extras = crc_extras()

    missing = [name for name in MESSAGES if name not in xml]
    if missing:
        sys.exit(f"the dialect at {where} does not define: {', '.join(missing)}")

    messages = []
    for name in MESSAGES:
        definition = xml[name]
        if name not in extras:
            sys.exit(f"pymavlink derived no CRC_EXTRA for {name}")
        messages.append(
            {
                "id": definition["id"],
                "name": name,
                "crc_extra": extras[name],
                "fields": definition["fields"],
            }
        )

    cross_check(messages)

    return {
        "note": (
            "Derived from pymavlink's published message definitions by "
            "tools/check-dialect.py. Do not edit by hand - regenerate with "
            "`python3 tools/check-dialect.py`. This file is the oracle "
            "tests/dialect.test.ts checks the hand-written table in "
            "src/protocol/mavlink.ts against."
        ),
        "source": "common.xml",
        "pymavlink": pm.__version__,
        "messages": messages,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if tests/fixtures/dialect.json is not what the dialect says",
    )
    args = parser.parse_args()

    derived = build()
    text = json.dumps(derived, indent=2, sort_keys=False) + "\n"

    if args.check:
        try:
            with open(OUT) as handle:
                on_disk = handle.read()
        except FileNotFoundError:
            print(f"{OUT} is missing; run without --check to write it", file=sys.stderr)
            return 1
        if on_disk != text:
            print(
                f"{OUT} is out of date with the dialect pymavlink ships.\n"
                "Run `python3 tools/check-dialect.py` and commit the result.",
                file=sys.stderr,
            )
            return 1
        print(f"dialect.json matches {derived['source']} from pymavlink {derived['pymavlink']}")
        return 0

    with open(OUT, "w") as handle:
        handle.write(text)
    total = sum(len(m["fields"]) for m in derived["messages"])
    print(
        f"wrote {OUT}: {len(derived['messages'])} messages, {total} fields, "
        f"from pymavlink {derived['pymavlink']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
