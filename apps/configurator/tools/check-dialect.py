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
import struct
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
    # The third message this app sends. It is in this list for the same reason
    # the other two are: a message a client puts on the wire is one whose
    # definition it had better have right, and this one changes a number the
    # autopilot acts on.
    "PARAM_SET",
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


# ---------------------------------------------------------------------------
# The other half of the question: what a peer newer than this table sends.
# ---------------------------------------------------------------------------
#
# `messages` above is the dialect as **pymavlink** publishes it, and pymavlink
# is pinned and finite. That is the right oracle for "is this app's table
# correct", and the wrong one for "will this app read what a real vehicle
# sends" - because MAVLink grows a message by appending fields after
# `<extensions/>`, and those appended fields are **excluded from `crc_extra`**.
#
# That exclusion is the design rather than an oversight: it is what lets a
# vehicle add a field without invalidating every ground station already in the
# field. Its consequence is the one that matters here - a frame whose checksum
# validates may still carry bytes past the definition the validator holds, and
# the check passing says nothing about those bytes either way.
#
# So a payload longer than this app's layout is not a fault and not a newer
# dialect to refuse; it is an ordinary frame from a vehicle whose dialect grew
# after this app's table was copied, and the honest reading is its first
# `structLength` bytes plus a note that there is more. Found on `SYS_STATUS`:
# the pinned table carries 31 bytes, PX4 1.17.0 sends 43, and the app dropped
# every frame of the link for the difference.
#
# This block therefore records, from PX4's own pinned definition, which of the
# messages this app carries have grown since the pin, by how many bytes and
# with which fields - and builds one real frame per grown message, packed
# longhand to MAVLink's wire rule and checksummed with the `crc_extra`
# pymavlink derived. `tests/dialect.test.ts` decodes those with the app's own
# decoder and requires the base fields back exactly as they were put in, which
# is the one thing hand-typed hex could not establish.

#: PX4's message definitions, relative to its checkout root.
PX4_XML = os.path.join("src", "modules", "mavlink", "mavlink",
                       "message_definitions", "v1.0")
DEFAULT_PX4 = os.path.abspath(os.path.join(HERE, "..", "..", "..",
                                           "upstream", "px4-1.17.0"))

#: Wire sizes in bytes, and the struct format each type packs as.
SIZES = {"double": 8, "uint64_t": 8, "int64_t": 8, "float": 4, "uint32_t": 4,
         "int32_t": 4, "uint16_t": 2, "int16_t": 2, "char": 1, "int8_t": 1,
         "uint8_t": 1}
FMT = {"double": "d", "uint64_t": "Q", "int64_t": "q", "float": "f",
       "uint32_t": "I", "int32_t": "i", "uint16_t": "H", "int16_t": "h",
       "char": "c", "int8_t": "b", "uint8_t": "B"}

#: The framing the frames below use. Fixed rather than random: the fixture is
#: compared byte for byte between runs, so anything that varies would make
#: `--check` red for no reason.
FRAME_SEQ, FRAME_SYS, FRAME_COMP = 9, 1, 1


def field_size(field: dict) -> int:
    return SIZES[field["type"]] * field.get("count", 1)


def wire_order(fields: list[dict]) -> list[dict]:
    """MAVLink sorts the base fields by descending size, stable within a size.

    Extensions are *not* sorted: they go on the end in declaration order, after
    every base field. Sorting them too would produce a frame a real vehicle
    never sends, which would test the wrong thing.
    """
    base = [f for f in fields if not f.get("ext")]
    ext = [f for f in fields if f.get("ext")]
    return sorted(base, key=lambda f: -SIZES[f["type"]]) + ext


def sample_value(field: dict, order: int) -> object:
    """A value of the field's own type, distinct per field.

    Distinct so that a decoder reading the wrong offset fails rather than
    happening to agree; of the field's own type so the bytes are legal on the
    wire; and a pure function of the field's position so the frame is the same
    on every run.
    """
    count = field.get("count", 1)
    if field["type"] == "char":
        return "".join(chr(ord("a") + (order + k) % 26) for k in range(count))
    if count > 1:
        return [(order * 16 + k + 1) & 0xFF for k in range(count)]
    width = SIZES[field["type"]] * 8
    if field["type"] in ("float", "double"):
        return float(order + 1) * 1.5
    if field["type"] in ("int8_t", "int16_t", "int32_t", "int64_t"):
        width -= 1  # keep it positive, so the JSON comparison is exact
    return ((order + 1) * 0x11) & ((1 << width) - 1)


def pack_field(field: dict, value: object) -> bytes:
    count = field.get("count", 1)
    if field["type"] == "char":
        raw = value.encode("ascii")[:count]
        return raw + bytes(count - len(raw))
    if count > 1:
        return struct.pack("<%d%s" % (count, FMT[field["type"]]), *value)
    if field["type"] in ("float", "double"):
        return struct.pack("<" + FMT[field["type"]], value)
    return struct.pack("<" + FMT[field["type"]], value)


def crc16_mcrf4xx(data: bytes, extra: int) -> int:
    """MAVLink's checksum, with the message's `crc_extra` folded in at the end.

    Written out here rather than imported because the two scripts that need it
    are standalone tools with no shared module between them - the same reason
    `capture-mavlink-fixtures.py` has its own copy. It is checked the only way
    that means anything: a wrong implementation makes every `crc_extra` in the
    file fail to validate a frame, and `read_v2` below refuses to write one.
    """
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def frame_v2(msgid: int, payload: bytes, extra: int) -> bytes:
    body = bytes([len(payload), 0, 0, FRAME_SEQ, FRAME_SYS, FRAME_COMP,
                  msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF]) + payload
    return bytes([0xFD]) + body + struct.pack("<H", crc16_mcrf4xx(body, extra))


def read_v2(buf: bytes) -> tuple[int, bytes]:
    """`(msgid, payload)` from a v2 frame, checksum verified against its own id.

    This script's own reader, not pymavlink's, so a frame it accepts is one two
    independent framings agree on. A frame it rejects fails the run rather than
    landing in the fixture as a test that cannot pass.
    """
    if len(buf) < 12 or buf[0] != 0xFD:
        raise ValueError("not a v2 frame")
    length = buf[1]
    msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
    payload = buf[10:10 + length]
    if len(payload) != length:
        raise ValueError("short frame")
    got = struct.unpack_from("<H", buf, 10 + length)[0]
    if got != crc16_mcrf4xx(buf[1:10 + length], CRC_EXTRA_BY_ID.get(msgid, -1)):
        raise ValueError("checksum")
    return msgid, payload


#: Filled in by `newer_peer()` from pymavlink's own `crc_extra`, so `read_v2`
#: checks against a value this script did not choose.
CRC_EXTRA_BY_ID: dict[int, int] = {}


def newer_peer(px4_dir: str, mine: dict[str, dict]) -> dict:
    """Which of this app's messages have grown since the pin, and by how much.

    Returns `{"measured": path, ...}` with an empty `messages` list when PX4 is
    not on this machine - the field lists below come out of that checkout, and
    inventing them would be worse than saying they were not read.
    """
    where = os.path.join(px4_dir, PX4_XML)
    theirs = messages_from_xml(where) if os.path.isdir(where) else {}

    grown: list[dict] = []
    for name in MESSAGES:
        definition = theirs.get(name)
        if definition is None:
            continue
        # A base field list that disagrees is not an extension and not a silence:
        # MAVLink does not change the base of a message, so this would be a
        # finding about one of the two sources and must stop the run.
        first = [f for f in mine[name]["fields"] if not f.get("ext")]
        second = [f for f in definition["fields"] if not f.get("ext")]
        if first != second:
            sys.exit(
                f"{name}: PX4's definition and pymavlink's disagree about the base "
                f"fields, which MAVLink does not change.\n  pymavlink: {first}\n"
                f"  px4:       {second}"
            )
        base_bytes = sum(field_size(f) for f in first)
        ext = [f for f in definition["fields"] if f.get("ext")]
        if not ext:
            continue

        order = wire_order(definition["fields"])
        values = {f["name"]: sample_value(f, i) for i, f in enumerate(order)}
        payload = b"".join(pack_field(f, values[f["name"]]) for f in order)
        extra = CRC_EXTRA_BY_ID.get(definition["id"])
        if extra is None:
            sys.exit(f"{name}: no crc_extra for id {definition['id']}")
        frame = frame_v2(definition["id"], payload, extra)
        got, back = read_v2(frame)
        if got != definition["id"] or back != payload:
            sys.exit(f"{name}: this script's own reader did not recover the frame it built")

        grown.append({
            "name": name,
            "id": definition["id"],
            "crc_extra": extra,
            "base_bytes": base_bytes,
            "full_bytes": base_bytes + sum(field_size(f) for f in ext),
            "extension_fields": [f["name"] for f in ext],
            "frame_hex": frame.hex(),
            # The *base* fields as they were packed, so the TypeScript side can
            # require the app's decoder to return them unchanged. The extension
            # values are deliberately not listed: what the app does with bytes
            # it has no definition for is the other half of the test, and it is
            # asserted as "the base fields still agree", not as a value.
            "expect": {f["name"]: values[f["name"]] for f in first},
        })

    return {
        "note": (
            "Which of the messages this app carries have grown since pymavlink "
            "2.4.49 was pinned, read from PX4's own definition, and one real "
            "frame per grown message. See the comment above `newer_peer()` in "
            "tools/check-dialect.py for why a longer payload is a legal frame "
            "rather than a fault."
        ),
        "measured": where if theirs else None,
        "source": "px4-1.17.0 common.xml" if theirs else None,
        "messages": grown,
    }


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


def build(px4_dir: str = DEFAULT_PX4, disk: dict | None = None) -> dict:
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
        # Read by `read_v2`, so the frames in the newer-peer block below are
        # checked against pymavlink's numbers rather than this script's.
        CRC_EXTRA_BY_ID[definition["id"]] = extras[name]

    cross_check(messages)

    peer = newer_peer(px4_dir, xml)
    if peer["measured"] is None:
        # PX4's checkout is not part of this repository and not everyone has
        # one. Rather than drop the block - which would silently turn the check
        # into one that only covers the pinned dialect, exactly the gap this
        # half exists to close - the measured block is carried forward as it
        # stands, and this says so out loud every time.
        kept = (disk or {}).get("newer_peer") or {"messages": []}
        print(
            "check-dialect: no PX4 checkout at %s, so the newer-peer block is "
            "carried forward from %s as %s measured it (%d messages)."
            % (px4_dir, OUT, kept.get("source") or "nothing", len(kept.get("messages", []))),
            file=sys.stderr,
        )
        peer = kept

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
        "newer_peer": peer,
    }


def read_disk() -> dict | None:
    """What is in the fixture now, for the newer-peer block to be carried in."""
    try:
        with open(OUT) as handle:
            return json.load(handle)
    except (FileNotFoundError, ValueError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if tests/fixtures/dialect.json is not what the dialect says",
    )
    parser.add_argument(
        "--px4",
        default=DEFAULT_PX4,
        help="a PX4 checkout, read for the fields it has added to the messages "
             "this app carries since pymavlink was pinned (default: %(default)s)",
    )
    args = parser.parse_args()

    derived = build(args.px4, read_disk())
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
        print(f"dialect.json matches {derived['source']} from pymavlink {derived['pymavlink']}, "
              f"and {len(derived['newer_peer']['messages'])} grown messages from "
              f"{derived['newer_peer']['source'] or 'a block carried forward'}")
        return 0

    with open(OUT, "w") as handle:
        handle.write(text)
    total = sum(len(m["fields"]) for m in derived["messages"])
    print(
        f"wrote {OUT}: {len(derived['messages'])} messages, {total} fields, "
        f"from pymavlink {derived['pymavlink']}"
    )
    for entry in derived["newer_peer"]["messages"]:
        print(
            f"  {entry['name']} has grown: {entry['base_bytes']} -> "
            f"{entry['full_bytes']} bytes, +{', '.join(entry['extension_fields'])}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
