#!/usr/bin/env python3
"""Capture real protocol frames from the firmware's own simulator.

The web app's tests must not check the app against frames the app built. This
drives the *firmware's* simulator with the *firmware's* client and writes down
what actually came back, byte for byte, so `tests/fixtures/aerialkit.json` is
evidence rather than a second copy of my own assumptions.

    python3 apps/configurator/tools/capture-fixtures.py

It looks for the simulator at `$AK_SIM`, else `firmware/build-host/aerialkit-sim`
relative to the workspace root. Supply an externally built simulator with `AK_SIM`.

Three files come out of one run:

  - `tests/fixtures/aerialkit.json` — the frames, and the parameter table the
    board actually sent.
  - `src/firmware/demo-table.json` — the *demo board's* starting table: the same
    rows with the firmware's own groups, ranges, help text and decimal places
    read out of `ak_flight.c` by `ak_table.py`, under a product string that is
    deliberately not the real one.
  - a non-zero exit if the names read from the source do not match the names
    that came off the wire, in order. That is the drift alarm: change the
    firmware's table without re-running this and the app's build fails rather
    than quietly offering a range for a parameter that no longer exists.

**This file used to produce the app's metadata for real boards, and it does not
any more.** Since milestone 3 the group, the ranges, the decimals, the default
and the help text arrive over `param info` and `param help`, and
`AerialKitSession` reads them off the wire. What remains here is a fixture: the
demo board has to start from *some* table, and a checked-in file that is
verified against the firmware it was captured from is a better starting point
than a table written by hand. The distinction that matters is that nothing
attaches these rows to a board that did not send them — the old design did, and
that is how a 32-row file sat beside a 92-parameter board without anything
failing.

(This file's docstring named the output `src/transport/demo-parameters.json`
until 2026-09-20 and `src/firmware/parameter-table.json` until 2026-10-01. The
first path has never existed; nothing reads a docstring, which is why it went
unnoticed.)
"""

import json
import os
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
APP = HERE.parent
WORKSPACE = APP.parent.parent
AERIALKIT = WORKSPACE / "firmware"
SIM_CANDIDATES = [
    os.environ.get("AK_SIM"),
    str(AERIALKIT / "build-host" / "aerialkit-sim"),
]

sys.path.insert(0, str(HERE))
import ak_table  # noqa: E402  (needs the path above)


def find_simulator() -> str:
    for candidate in SIM_CANDIDATES:
        if candidate and pathlib.Path(candidate).is_file():
            return candidate
    sys.exit(
        "no aerialkit-sim found; set AK_SIM to an externally built simulator.\n"
        "looked in:\n  " + "\n  ".join(c for c in SIM_CANDIDATES if c)
    )


def main() -> int:
    sim = find_simulator()
    sys.path.insert(0, str(AERIALKIT / "tools"))
    import akproto  # the firmware's own client

    process = subprocess.Popen(
        [sim], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )
    client = akproto.Client(
        process.stdout.read,
        lambda data: (process.stdin.write(data), process.stdin.flush()),
    )

    captured = {}

    def record(name, command, payload=b"", note=""):
        reply = client.request(command, payload)
        captured[name] = {
            "request": akproto.build(command, payload).hex(),
            "reply": client.last_frame.hex(),
            "note": note,
        }
        return reply

    hello = record("hello", akproto.HELLO, note="version, product, parameter count")
    version = hello[0]
    product = akproto.c_string(hello, 1)
    count = int.from_bytes(hello[2 + len(product) : 4 + len(product)], "little")

    record("status", akproto.STATUS, note="flight state, link, fix, attitude, position, motors")
    record("param_get_0", akproto.PARAM_GET, b"\x00", note="the first parameter")
    record(
        "param_get_past_end",
        akproto.PARAM_GET,
        bytes([count]),
        note="an index one past the table: status 1, and nothing else",
    )
    record(
        "param_set_refused",
        akproto.PARAM_SET,
        b"\x00" + b"999999",
        note="a value above rate_kp_roll's 0.000..3.000: the table's own words come back",
    )
    # A bare index is *not* a successful write. The firmware wants an index and
    # at least one byte of value and refuses anything shorter with "no such
    # parameter" - which is the shape a truncated write arrives in, so it is
    # worth a fixture of its own rather than being mistaken for one.
    record(
        "param_set_short",
        akproto.PARAM_SET,
        b"\x00",
        note="an index with no value: the shape is refused, not the value",
    )
    record(
        "param_set_ok",
        akproto.PARAM_SET,
        b"\x03" + b"1.5",
        note="rate_ki_roll written to 1.5, inside its 0.000..2.000",
    )
    record("param_save", akproto.PARAM_SAVE, note="persist, or say there is nowhere to")
    record("log_source_fast", akproto.LOG_SOURCE, b"\x00", note="the fast ring in RAM")
    record("log_source_flash", akproto.LOG_SOURCE, b"\x02", note="the log that survives the battery")
    record("log_get_0", akproto.LOG_GET, b"\x00\x00", note="one 51-byte record")
    record(
        "log_source_absent",
        akproto.LOG_SOURCE,
        b"\x09",
        note="a source the device does not have is refused, not answered as empty",
    )

    # Telemetry. The console link *refuses* to stream, and that is the
    # firmware's behaviour rather than a property of the simulator:
    # `ak_proto_init()` zeroes the whole struct (ak_proto.c), so `can_stream` is
    # 0, and the only place that ever sets it is the network link (main.c) -
    # where the comment says why: the console is the wire a person types at, and
    # frames arriving among their keystrokes is a console nobody can use. So the
    # reply below carries 0, and 0 is an answer rather than an absence.
    #
    # This comment said the simulator "answers with the rate it was asked for"
    # until 2026-09-20, and the fixture said the same thing in its anomalies
    # block. That was true once, of a bug: ak_proto_init used to list its fields
    # by hand and `can_stream` was missing from the list, so a subscribe reply
    # was whatever the stack held - which on one build was 10.
    telemetry_reply = record(
        "telemetry_rate",
        akproto.TELEMETRY,
        b"\x0a",
        note="asked for 10 Hz over the console; refused, see anomalies",
    )
    agreed = telemetry_reply[0]

    # Every parameter, so the app can be checked against a whole real table.
    parameters = []
    for index in range(count):
        payload = client.request(akproto.PARAM_GET, bytes([index]))
        if payload[0] != 0:
            break
        name = akproto.c_string(payload, 1)
        value = akproto.c_string(payload, 2 + len(name))
        parameters.append({"index": index, "name": name, "value": value})

    process.kill()

    # The firmware's own metadata for those same names, read from the source.
    try:
        table = ak_table.extract(
            AERIALKIT / "src" / "core" / "flight" / "ak_flight.c", AERIALKIT / "src"
        )
    except ak_table.TableError as error:
        sys.exit(f"could not read the parameter table from the firmware: {error}")

    wire_names = [item["name"] for item in parameters]
    source_names = [item["name"] for item in table]
    if wire_names != source_names:
        for at, (wire, source) in enumerate(zip(wire_names, source_names)):
            if wire != source:
                sys.exit(
                    f"the firmware's table and the board disagree at index {at}: "
                    f"the board sent {wire!r}, ak_flight.c declares {source!r}.\n"
                    "Re-run after checking which side changed."
                )
        sys.exit(
            f"the firmware declares {len(source_names)} parameters and the board "
            f"sent {len(wire_names)}; the table has changed and the app has not."
        )

    merged = [
        {**table[item["index"]], "value": item["value"], "default": item["value"]}
        for item in parameters
    ]

    out = APP / "tests" / "fixtures" / "aerialkit.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(
            {
                "captured_from": os.path.basename(sim),
                "protocol_version": version,
                "product": product,
                "parameter_count": count,
                "frames": captured,
                "parameter_table": parameters,
                "anomalies": [
                    {
                        "what": "the console link refuses a telemetry stream, and could not have sent one if it had agreed",
                        "evidence": (
                            f"telemetry subscribe asked for 10 and the reply carried {agreed} - a "
                            "refusal, not a rate; no telemetry frame followed"
                        ),
                        "why": (
                            "Two independent reasons, and they are worth keeping apart. First, "
                            "the firmware's: `ak_proto_init()` zeroes the whole struct, so "
                            "`can_stream` is 0, and main.c sets it on the network link and not on "
                            "the console - where the comment gives the reason, that the console is "
                            "the wire a person types at and frames arriving among their keystrokes "
                            "is a console nobody can use. A console subscribe is therefore "
                            "answered 0 by design. Second, the simulator's: "
                            "tools/akproto_sim.c never calls ak_proto_telemetry_frame(), so even a "
                            "link that agreed to a rate could not push a frame. This entry used "
                            "to attribute the reply to uninitialised stack memory, which was true "
                            "of an older ak_proto_init and is true of nothing now."
                        ),
                        "app_consequence": (
                            "the app must treat 'no stream' and 'frames not arriving' as two "
                            "different facts: a 0 is a refusal and is reported as one, while a "
                            "non-zero rate followed by silence is a promise that was not kept. It "
                            "says which, and takes its values from polling STATUS either way"
                        ),
                    }
                ],
            },
            indent=2,
        )
        + "\n"
    )

    # The demo board's starting table, as a module the app can import.
    #
    # `captured_product` is the board the values came from, and it is provenance
    # rather than an authority now: nothing attaches these rows to a board that
    # did not send them. `demo_product` is what the demo board calls itself, and
    # it is deliberately not the real one, so nothing built from this file can
    # pass for hardware.
    table_out = APP / "src" / "firmware" / "demo-table.json"
    table_out.parent.mkdir(parents=True, exist_ok=True)
    table_out.write_text(
        json.dumps(
            {
                "_comment": (
                    "The demo board's starting table. Generated by "
                    "tools/capture-fixtures.py: the names and boot values come off "
                    "the running firmware's wire and the groups, ranges, help text "
                    "and decimals out of ak_flight.c. This is a fixture, not a "
                    "board's metadata - a real board's description arrives over "
                    "`param info` and `param help`. Do not edit by hand; re-run the "
                    "capture."
                ),
                "captured_product": product,
                "captured_from": os.path.basename(sim),
                "demo_product": "aerialkit-demo",
                "protocol_version": version,
                "parameters": merged,
            },
            indent=2,
        )
        + "\n"
    )

    print(f"wrote {out} ({len(captured)} frames, {len(parameters)} parameters)")
    print(f"wrote {table_out} (metadata from ak_flight.c)")
    print(f"the firmware's table and the board agree on all {len(source_names)} names, in order")
    return 0


if __name__ == "__main__":
    sys.exit(main())
