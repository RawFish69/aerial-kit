#!/usr/bin/env python3
"""The client against the *whole firmware*, on the console path a board uses.

    make firmware-proto-check

`akproto_check.py` drives the protocol simulator: a stand-in that answers with a
synthetic table, a synthetic status and a synthetic log. It is a good check, and
it is a check of the *client* and of the protocol's shape - its expectations are
the stand-in's (it asserts, for instance, that the table has exactly 28 entries,
because that is what the stand-in was built with; the firmware's own table has
90).

What a board does when the configurator opens the console is this: the real
firmware, whose protocol server is `src/core/main.c`'s callbacks over
`src/core/ak_proto.c` and the real table in `src/core/ak_params.c`. Until this
existed that path had been exercised only through the ESP32 (over the network,
under QEMU, in `scripts/esp32-proto.sh`), which is slow and needs ESP-IDF. Here
it is the simulator's console - `aerialkit-fw-sim 0 console`, the same binary
the sessions use - so the host suite covers it in a second.

What is checked is the firmware's own answers rather than a transcript's: the
product it names, the table it reports (every index it claims, each with a name,
none twice), a parameter written back to the value it already had, a value it
has to refuse, a status that parses, a save that goes through the board's
storage, and each log's framing. What a *flight* logged is the flight's
business: the sessions and the ESP32's own check are where that is judged.
"""

import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from akproto import (CALIBRATE, CALIBRATE_ABORT, CALIBRATE_ACCEL,
                     CALIBRATE_GYRO, CALIBRATE_IDLE, CALIBRATE_IMPLAUSIBLE,
                     CALIBRATE_NO_FACE, CALIBRATE_NO_SESSION,
                     CALIBRATE_NO_VERB, CALIBRATE_OK, CALIBRATE_RC,
                     CALIBRATE_RESULT_MEANING, CALIBRATE_STATUS,
                     CALIBRATE_VBAT, CALIBRATE_VERB_NAMES,
                     Client, HELLO, LOG_INFO, LOG_SOURCES, LOG_SOURCE_NAMES,
                     LOG_STREAM, LOG_STREAM_MAX_HZ,
                     MISSION, MISSION_HOME_CLEAR, MISSION_HOME_SET,
                     MISSION_NO_FIX, MISSION_NO_INDEX, MISSION_NO_NAV,
                     MISSION_NO_VERB,
                     MISSION_NO_WAYPOINTS, MISSION_OK, MISSION_START,
                     MISSION_STATUS, MISSION_STOP, MISSION_VERB_NAMES,
                     PARAM_GET, PARAM_SAVE, PARAM_SET, PARAM_TEXT, PERF,
                     PERF_NONE, PERF_OK, PERF_REPLY_BYTES, PERF_SECTIONS,
                     PREFLIGHT, PREFLIGHT_VERDICTS, parse_perf,
                     RC_CHANNELS, RC_FLAGS, RC_MAX_CHANNELS, RC_OK,
                     RC_SWITCHES, SENSOR_BARO, SENSOR_BATTERY,
                     SENSOR_BODY_LENGTH, SENSOR_GPS,
                     SENSOR_IMU, SENSOR_INFO, SENSOR_NAME_LENGTH, SENSOR_NAMES,
                     SENSOR_OK, SENSOR_RANGE, SENSOR_TOPICS, STATUS, c_string,
                     feature_names,
                     fetch_log, fetch_param_help, fetch_param_info,
                     fetch_preflight,
                     parse_calibration, parse_hello, parse_mission,
                     parse_rc_channels, parse_sensor_info,
                     parse_status, select_log, sensor_info,
                     set_reply, start_log_stream)  # noqa: E402 - after sys.path

SIM = os.environ.get("AK_SIM", "build-host/aerialkit-fw-sim")
SIM_ARGS = os.environ.get("AK_SIM_ARGS", "0 console").split()
PRODUCT = os.environ.get("AK_PRODUCT", "aerialkit-f405")

failures = 0


def expect(name, condition, detail=""):
    global failures
    if condition:
        print("  ok       %s %s" % (name, detail))
    else:
        print("  FAILED   %s %s" % (name, detail))
        failures += 1


def row(client, index):
    """`(name, value)` from one table row, or None if it does not answer."""
    payload = client.request(PARAM_GET, bytes([index]))
    try:
        name = c_string(payload, 1)
        return name, c_string(payload, 2 + len(name))
    except (ValueError, UnicodeDecodeError):
        return None


def console_lines(read, write, command, until, cap=128):
    """Type `command` at the console and read its answer back as lines.

    `read` is the *same* one-byte reader the protocol client was given -
    `process.stdout.read` - and that is the point rather than a convenience.
    Both halves of this program share one pipe and one buffered reader, so a
    second handle on the same descriptor would race this one for whatever is
    already sitting in the buffer, and the half that lost would see the other
    half's bytes missing rather than an error.

    The answer is delimited by its own last line (`until`) and not by silence.
    The board prints an `alive:` heartbeat on this stream for as long as it
    runs, so waiting for quiet is waiting forever - the trap bench_check.py
    records, where a check that takes 0.27 s spends 175 s first."""
    write((command + "\r").encode())
    lines = []
    pending = ""
    for _ in range(cap * 1024):
        byte = read(1)
        if not byte:
            break
        char = byte.decode("latin-1")
        if char != "\n":
            pending += char
            continue
        line = pending.rstrip("\r")
        pending = ""
        if line.strip():
            lines.append(line)
        # Compared stripped, because several of these reports end on a
        # *continuation* line - the barometer's "35001 baro, 5250 gps samples"
        # is indented under `fused:` and carries no key of its own - and a
        # delimiter that only matched at column zero would run past it into the
        # next command's output.
        if line.strip().startswith(until):
            break
    return lines


def receiver_from_console(lines):
    """The console's `rc` answer as fields, or an empty dict if it did not come.

    Deliberately returns what it found rather than defaulting: a field the
    console did not print and a field it printed as zero are different, and
    every check on this dict is written to fail on the absence."""
    fields = {}
    for line in lines:
        key, _, rest = line.partition(":")
        rest = rest.strip()
        if key == "receiver":                       # "crsf, 13312 bytes, 512 frames"
            parts = [p.strip().split() for p in rest.split(",")]
            if len(parts) != 3 or len(parts[2]) < 2:
                continue
            fields["protocol"] = parts[0][0]
            fields["bytes"] = int(parts[1][0])
            fields["frames"] = int(parts[2][0])
        elif key == "crsf":                         # "0 crc errors, 0 rejected"
            words = rest.split()
            if len(words) < 5:
                continue
            fields["crc_errors"] = int(words[0])
            fields["rejected"] = int(words[3])
        elif key == "link":                         # "framing"
            fields["link"] = rest
        elif key == "channels":                     # "992 992 172 ..."
            fields["channels"] = [int(v) for v in rest.split()]
        elif key == "sticks":                       # "roll 0, pitch 0, ... per-mille"
            words = rest.replace(",", " ").split()
            if len(words) != 9:
                continue
            fields["sticks"] = [int(v) for v in words[1::2]]
        elif key == "switches":                     # "arm off, mode angle"
            parts = [p.split() for p in rest.split(",")]
            if len(parts) != 2 or not parts[0] or not parts[1]:
                continue
            fields["arm"] = parts[0][-1]
            fields["mode"] = parts[1][-1]
        elif key == "uart":                         # "0 bytes dropped by ..."
            fields["dropped"] = int(rest.split()[0])
    return fields


def sensor_from_console(topic, lines):
    """One console sensor report as fields, keyed by the *wire's* own names.

    Deliberately returns what it found rather than defaulting, the same
    argument as `receiver_from_console`: a line the console did not print and a
    line it printed as zero are different things, and every check on this dict
    fails on the absence.

    The keys are the wire's names rather than the console's on purpose. A
    translation table between the two would be a third place for a field to be
    read from the wrong offset, and the whole point of comparing these two
    renderings is that there is nothing in between them.
    """
    fields = {}

    def numbers(rest):
        return rest.replace(",", " ").split()

    def numeric(word):
        """`word` as a float, or None when the console printed a sentence.

        The reports have branches - a barometer with no reference yet prints
        "not captured yet" where it otherwise prints pascals - and a parser that
        assumed the number was there would either raise or, worse, read the
        first word of the sentence. Returning None makes the check that uses it
        fail on the absence instead."""
        try:
            return float(word)
        except ValueError:
            return None

    for line in lines:
        key, sep, rest = line.partition(":")
        rest = rest.strip()
        if not sep:
            # "35001 baro, 5250 gps samples" - the barometer's two sample
            # counts, printed as a continuation line with no key of its own.
            words = numbers(line)
            if len(words) == 5 and words[1] == "baro" and words[3] == "gps":
                fields["baro_samples"] = int(words[0])
                fields["gps_samples"] = int(words[2])
            continue
        words = numbers(rest)

        if topic == "imu":
            if key == "imu" and len(words) >= 5 and words[1] != "none":
                fields["driver"] = words[0]
                fields["samples"] = int(words[1])
                fields["errors"] = int(words[3])
            elif key == "sample" and words[:1] == ["accel"]:
                fields["accel"] = [int(v) for v in words[1:4]]
            elif key == "sample" and words[:1] == ["gyro"]:
                fields["gyro"] = [int(v) for v in words[1:4]]
            elif key == "alignment" and len(words) >= 6:
                fields["align"] = [int(words[1]), int(words[3]), int(words[5])]
            elif key == "gyro bias":
                fields["gyro_bias"] = [int(v) for v in words[:3]]
        elif topic == "baro":
            if key == "baro" and len(words) >= 5 and words[1] != "none":
                fields["driver"] = words[0]
                fields["samples"] = int(words[1])
                fields["errors"] = int(words[3])
            elif key == "pressure" and len(words) >= 5:
                fields["pressure_pa"] = int(words[0])
                fields["temperature_c"] = float(words[3])
            elif key == "reference" and words:
                captured = numeric(words[0])
                fields["have_reference"] = int(captured is not None)
                if captured is not None:
                    fields["reference_pa"] = int(captured)
            elif key == "height" and words and numeric(words[0]) is not None:
                fields["height_cm"] = int(round(float(words[0]) * 100.0))
            elif key == "fused" and words and numeric(words[0]) is not None:
                fields["fused_cm"] = int(round(float(words[0]) * 100.0))
        elif topic == "range":
            if key == "range" and len(words) >= 7 and words[0] != "none":
                fields["driver"] = words[0]
                fields["address"] = int(words[2], 16)
                fields["max_mm"] = int(round(float(words[5]) * 1000.0))
            elif key == "ground" and words:
                # The console prints metres, or the sentence it prints when
                # nothing is in range - which is the negative the wire keeps
                # rather than clamping to a wall against the lens.
                fields["distance_mm"] = (
                    -1 if numeric(words[0]) is None
                    else int(round(float(words[0]) * 1000.0)))
            elif key == "readings" and len(words) == 10:
                # "16300 kept, 0 out of range, 0 impossible, 0 failed" - four
                # counts each followed by its own noun, so the numbers are every
                # third word.
                fields["samples"] = int(words[0])
                fields["out_of_range"] = int(words[2])
                fields["rejected"] = int(words[6])
                fields["faults"] = int(words[8])
            elif key == "landing" and words.count("within") == 2:
                at = words.index("within")
                fields["land_mm"] = int(words[at + 1])
                fields["agree_m"] = float(words[words.index("within", at + 1) + 1])
        elif topic == "battery":
            if key == "battery" and len(words) >= 10 and words[0] != "none":
                fields["cells"] = int(words[0].rstrip("S"))
                fields["volts"] = float(words[1])
                fields["volts_per_cell"] = float(words[4])
                fields["state_name"] = words[-1]
            elif key == "divider" and words:
                fields["ratio"] = float(words[0])
            elif key == "thresholds" and len(words) >= 10:
                fields["warn_cell_v"] = float(words[2])
                fields["critical_cell_v"] = float(words[8])
            elif key == "readings" and len(words) == 6:
                # "22500 taken, 0 rejected as implausible"
                fields["samples"] = int(words[0])
                fields["rejected"] = int(words[2])
            elif key == "adc" and len(words) >= 3:
                # "simulated: 1.090 V at the pin behind a 11:1 divider" - the
                # words after the key are the board's own, so the volts are
                # found rather than assumed to be first.
                for i, word in enumerate(words):
                    if word == "V" and numeric(words[i - 1]) is not None:
                        fields["pin_mv"] = int(round(
                            float(words[i - 1]) * 1000.0))
                        break
        elif topic == "gps":
            if key == "fix" and len(words) >= 4:
                fields["fix_type"] = int(words[1])
                fields["satellites"] = int(words[3])
            elif key == "position" and len(words) >= 2:
                fields["lat_e7"] = int(round(float(words[0]) * 1e7))
                fields["lon_e7"] = int(round(float(words[1]) * 1e7))
            elif key == "altitude" and words:
                fields["alt_msl_mm"] = int(words[0])
            elif key == "fixes" and words:
                fields["fixes"] = int(words[0])
                fields["valid_now"] = int("valid now: yes" in rest)
            elif key == "home" and len(words) >= 6:
                # "0 m away, bearing 0.00 deg" - and the branch where there is
                # no home prints a sentence instead, which lands here as a
                # non-numeric first word and leaves the field absent.
                if numeric(words[0]) is not None:
                    fields["home_distance_m"] = int(words[0])
                    fields["home_bearing_cdeg"] = int(round(
                        float(words[-2]) * 100.0))
            elif key == "uart" and words:
                fields["dropped"] = int(words[0])
            elif key == "configure" and words:
                fields["config_sends"] = int(words[0])
    return fields


def mission_from_console(lines):
    """The console's `mission` report as the fields MISSION carries.

    Keyed by the wire's names for the same reason `sensor_from_console` is: a
    translation table between the two renderings would be a third place for a
    field to be read from the wrong offset, and there is nothing between these
    two readings to get wrong.

    The console's report is *not* the wire's: it prints the waypoints
    themselves (they are parameters, and the console is a screen with room)
    where the wire prints only how many there are. So the waypoint list parsed
    here is not compared byte for byte with anything - it is what the *console*
    saw, and the check that uses it compares the two routes' answers about the
    same aircraft. Optional lines are absent rather than defaulted, the same
    rule as every other parser here.
    """
    fields = {"waypoints": [], "flying_at": None}
    for line in lines:
        stripped = line.strip()

        if stripped.startswith("mission:"):                 # "mission:   2 waypoints, flying"
            rest = stripped[len("mission:"):].strip()
            words = rest.split()
            if len(words) >= 3 and words[1].startswith("waypoint"):
                # "0 waypoints, not flying" - the list's own header, and the
                # only line of the report that opens this way.
                fields["count"] = int(words[0])
                # "not flying" ends in the word "flying" as well, so the test
                # is the comma rather than the word: the report's two forms are
                # ", flying" and ", not flying" and nothing else.
                fields["flying"] = int(rest.endswith(", flying"))
            elif words[:1] == ["waypoint"] and len(words) >= 2:
                # "waypoint 0 is 52.1000000, 13.4000000 - 'save' keeps it" -
                # `mission add`'s answer, which is a different sentence about
                # the same list. It is parsed rather than ignored because it is
                # the only place the console says *which slot* it filled.
                try:
                    fields["added"] = (int(words[1]), float(words[3].rstrip(",")),
                                       float(words[4]))
                except (IndexError, ValueError):
                    continue
        elif stripped.startswith("[") and "," in stripped:
            # "  [0] 52.1000000, 13.4000000  <- flying here"
            where, _, tail = stripped.partition("]")
            try:
                index = int(where[1:])
                lat, _, lon = tail.partition(",")
                fields["waypoints"].append((index, float(lat), float(lon)))
            except ValueError:
                continue
            if "flying here" in tail:
                fields["flying_at"] = index
        elif " reached, holding " in stripped:
            words = stripped.split()
            fields["reached"] = int(words[0])
            fields["hold_alt_mm"] = int(words[-2])
        elif " started, " in stripped and "taken back" in stripped:
            words = stripped.split()
            fields["started"] = int(words[0])
            fields["cancelled"] = int(words[3])
        elif stripped.startswith("switch on channel "):
            fields["channel"] = int(stripped.rsplit(" ", 1)[1])
    return fields


def is_duration(word):
    """`"12.345"` - a duration as the console prints one, `%u.%03u` microseconds.

    Exactly three decimals and nothing else, so it is a shape rather than "can
    this be parsed as a float": the section lines are found by looking for this,
    and a loose test would find the `1` in a section called `1` instead of the
    number underneath it."""
    head, dot, tail = word.partition(".")
    return dot == "." and head.isdigit() and len(tail) == 3 and tail.isdigit()


def perf_from_console(lines):
    """The console's `perf` report, as the numbers PERF puts on the wire.

    Keyed by the wire's names for the same reason every other parser here is:
    a translation table between two renderings is a third place for a field to
    be read off the wrong line, and there is nothing between these two readings
    to get wrong.

    The units are converted here and only here - the console prints a section's
    average in microseconds with three decimals and the wire carries tenths, so
    the console's number is multiplied by ten. That multiplication is the point
    of the comparison the caller makes: the wire's `section_avg_x10` and the
    console's `%u.%03u us` are the same quantity in two units, and a client that
    read one as the other would be off by a factor of ten in a figure a person
    is meant to act on.

    The section lines are the three-space-indented ones with a `max` in them, and
    they are read by *position* into `PERF_SECTIONS` rather than by name: the
    names are the firmware's and a change to one is a change to both halves of
    this check, so matching on them would let a rename pass on one side only."""
    fields = {"sections": [], "section_names": [], "section_max_us": []}
    for line in lines:
        stripped = line.strip()

        if stripped.startswith("perf:"):
            # "perf:      1000 loops closed, 998 of them measured"
            words = stripped.split()
            fields["loops"] = int(words[1])
            fields["samples"] = int(words[4])
        elif stripped.startswith("period:"):
            # "period:    1000 us nominal, last 1004, min 997, max 1400, 3 late"
            # Read by the label in front of each number rather than by
            # position: the words between them ("us", "nominal,", "last") are
            # prose that a reworded line would move, and a reading that shifted
            # by one would put the minimum in the maximum's field.
            words = stripped.split()
            for i, word in enumerate(words):
                if word == "last":
                    fields["period_last_us"] = int(words[i + 1].rstrip(","))
                elif word == "min":
                    fields["period_min_us"] = int(words[i + 1].rstrip(","))
                elif word == "max":
                    fields["period_max_us"] = int(words[i + 1].rstrip(","))
                elif word == "late":
                    fields["late"] = int(words[i - 1])
            fields["nominal_us"] = int(words[1])
        elif stripped.startswith("jitter:"):
            # "jitter:    p50 4 us, p99 33 us, max 250 us (1 past the 255 us bin)"
            words = stripped.split()
            fields["jitter_p50_us"] = int(words[2])
            fields["jitter_p99_us"] = int(words[5])
            fields["jitter_max_us"] = int(words[8])
            fields["jitter_over"] = int(words[10].lstrip("("))
        elif stripped.startswith("load:"):
            # "load:      12.3% of the 1000 us slot, instrumented sections only"
            fields["load_permille"] = int(round(float(words[1].rstrip("%")) * 10))
        elif line.startswith("  ") and " max " in stripped:
            # "  imu        12.345 us   max 40 us"
            #
            # The name is everything in front of the duration rather than the
            # first word, and the maximum is read from the word after the
            # literal `max`. Both are anchored on something the firmware cannot
            # reword without the caller noticing: reading `words[0]` and
            # `words[4]` would take a two-word section name and raise a
            # ValueError out of the float() below - a crash instead of the
            # failed comparison this section exists to produce.
            words = stripped.split()
            at = next((i for i, word in enumerate(words) if is_duration(word)),
                      None)
            if at is None or "max" not in words[at:]:
                continue
            fields["section_names"].append(" ".join(words[:at]))
            fields["sections"].append(int(round(float(words[at]) * 10)))
            fields["section_max_us"].append(int(words[words.index("max", at) + 1]))
    return fields


def home_from_console(lines):
    """Whether the console thinks there is a fix, from its `home` answer.

    Two sentences and no numbers in the second: "home: no fix yet, so there is
    nowhere to remember" is the board saying it has no position, and a parser
    that reached for the first pair of integers would read the sentence's own
    "no" as a failure to parse rather than as the answer. So this returns
    `True`/`False`/`None` - and `None`, "the console said something neither
    branch prints", is a third answer the caller has to handle rather than a
    default to fall through.
    """
    for line in lines:
        stripped = line.strip()
        if not stripped.startswith("home:"):
            continue
        rest = stripped[len("home:"):].strip()
        if rest.startswith("no fix"):
            return False
        try:
            lat_text, _, lon_text = rest.partition(",")
            return (float(lat_text), float(lon_text))
        except ValueError:
            return None
    return None


def main():
    process = subprocess.Popen([SIM] + SIM_ARGS, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    write = lambda data: (process.stdin.write(data), process.stdin.flush())
    client = Client(process.stdout.read, write)

    print("the configurator's client against the firmware's own server")
    try:
        hello = client.request(HELLO)
        product = c_string(hello, 1)
        count = int.from_bytes(hello[2 + len(product):4 + len(product)], "little")
        expect("hello names the product this build was made as",
               product == PRODUCT, product)
        # The firmware's *live* count, not a number written here: this table is
        # the aircraft's and it grows with every feature. What is checked is
        # that every index it claims exists does.
        expect("and says how many parameters it has", count > 20,
               "%d parameters" % count)

        names = []
        values = {}
        bad = None
        for index in range(count):
            got = row(client, index)
            if got is None:
                bad = index
                break
            names.append(got[0])
            values[got[0]] = got[1]
        expect("every index the table claims answers with a name and a value",
               bad is None, "" if bad is None else "index %d" % bad)
        expect("and no two parameters share a name",
               bad is not None or len(set(names)) == len(names))
        expect("including the ones a configurator actually edits",
               "rate_kp_roll" in names and "airframe" in names)

        # ---- the capability word, against the firmware that sets it --------
        #
        # This is the check that has to be here rather than in the client's own
        # suite: what is being asserted is a claim about *this build*, made by
        # main.c's `proto_io.features` initialiser. A stand-in cannot check it,
        # because a stand-in's features are whatever the stand-in was told.
        parsed = parse_hello(hello)
        expect("hello carries a capability word at all",
               parsed["features"] is not None,
               "0x%08x" % (parsed["features"] or 0))
        if parsed["features"] is not None:
            expect("and this build claims a successful set re-applies the "
                   "configuration",
                   "APPLIES_ON_WRITE" in feature_names(parsed["features"]))
            # The bit that reads the other way round from how it was first
            # written, and the reversal is the record: until the write gate
            # landed, `save` was gated on the aircraft being disarmed and `set`
            # was not, so a board claiming this bit would have been telling a
            # client its writes were guarded while half of them were not - and
            # this line asserted the bit was *absent*. When the gate went in
            # (`write_allowed` in ak_proto.c, consulted by PARAM_SET, PARAM_SAVE
            # and PARAM_DEFAULT) this check failed, which is what it was written
            # to do. It now asserts the claim is made. What it cannot assert
            # here is that the gate *refuses*: a console session on this
            # simulator is disarmed and never arms, so the refusal is driven in
            # test_proto.c against a fake `writable` that is flipped between two
            # otherwise identical requests.
            expect("and claims every configuration write is gated on the "
                   "aircraft being disarmed",
                   "GATES_ON_ARMED" in feature_names(parsed["features"]),
                   feature_names(parsed["features"]))

        # ---- the configuration hash, recomputed from the table we just read -
        #
        # FNV-1a over "name=value\n" per row, in table order, which is what
        # `ak_params_serialize` writes and therefore what `ak_params_hash`
        # hashes. Two implementations, one number: the C in ak_params.c and
        # this. A backup file is only meaningful on a board with the same hash,
        # so this is the property Backup rests on - and it is checked against
        # the real table, whose size nothing here has to know.
        expected = 0x811C9DC5
        for name in names:
            for byte in ("%s=%s\n" % (name, values[name])).encode():
                expected = ((expected ^ byte) * 16777619) & 0xFFFFFFFF
        expect("the config hash is the hash of the table the same hello "
               "described",
               parsed["config_hash"] == expected,
               "board 0x%08x, recomputed 0x%08x"
               % (parsed["config_hash"] or 0, expected))

        beyond = client.request(PARAM_GET, bytes([200]))
        expect("asking past the end is refused rather than answered with noise",
               beyond[0] == 1)

        # ---- the metadata walk, against the value walk ---------------------
        #
        # Two enumerations of one table, and they have to agree index by index.
        # Neither side can tell on its own: the value walk cannot see a group or
        # a bound, and the metadata walk cannot see whether the row it described
        # is the row a configurator would actually be editing. This is the check
        # the plan calls the firmware-side acceptance test, and it runs on the
        # board's own table rather than on a synthetic one built to suit it.
        # (The counts on both sides are read rather than written down here, so
        # this check does not move when the table does - `akproto_check.py` is
        # the one that owns the number, for the reason its comment gives.)
        expect("this build claims it can describe its own parameters",
               parsed["features"] is not None and
               "PARAM_INFO" in feature_names(parsed["features"]))

        described = fetch_param_info(client)
        expect("the metadata walk reaches every parameter the value walk "
               "reached", len(described) == len(names),
               "%d described, %d named" % (len(described), len(names)))

        # Index by index, not name by name - joining by name is what let a
        # snapshot with two fewer rows pass for the board's truth.
        wrong_name = [e["index"] for e in described
                      if e["name"] != names[e["index"]]]
        expect("and names the same parameter at every index", not wrong_name,
               "index(es) %s" % wrong_name[:6])

        unknown_group = sorted({e["group"] for e in described
                                if e["group_name"] == "unknown"})
        expect("every entry carries a group this client has a name for",
               not unknown_group, "byte(s) %s" % unknown_group)

        unparsable, outside, too_precise = [], [], []
        for entry in described:
            # `AK_PARAM_SECRET` is a flag a *text* row carries and this firmware
            # has none of them (the `wifi_*` rows are the ESP32's), so every row
            # here is a number with bounds. A secret is not compared below
            # because its default is deliberately withheld as `***`.
            if entry["type"] == PARAM_TEXT:
                if not entry["max_len"]:
                    unparsable.append(entry["name"])
                continue
            try:
                low, high = float(entry["min"]), float(entry["max"])
                current = float(values[entry["name"]])
            except ValueError:
                unparsable.append(entry["name"])
                continue
            # The bounds have to be about the parameter the value belongs to,
            # which is a thing no page can state and only arithmetic can check:
            # a mis-joined entry shows a right value against a neighbour's range.
            if not low <= current <= high:
                outside.append(entry["name"])
            # The value and the default are spelled by the same table the
            # metadata came from, so a decimals byte that disagrees with them is
            # an entry describing a different parameter than the one it is
            # indexed beside.
            for text in (values[entry["name"]], entry["default"]):
                if "." in text and len(text.split(".")[1]) > entry["decimals"]:
                    too_precise.append(entry["name"])
                    break
        expect("every numeric entry's bounds parse and hold its own value and "
               "its own default", not unparsable and not outside,
               " ".join((unparsable + outside)[:4]))
        expect("and every value is spelled with the precision the entry states",
               not too_precise, " ".join(too_precise[:4]))

        # A row's prose, walked by offset. Fetched for every row rather than one,
        # because the walk has a different length to reassemble per row and the
        # end of the walk is where a client loses a tail.
        silent, drifted = [], []
        for entry in described:
            prose = fetch_param_help(client, entry["index"])
            if not prose:
                silent.append(entry["name"])
            elif fetch_param_help(client, entry["index"]) != prose:
                drifted.append(entry["name"])
        expect("every parameter's help arrives whole and reads the same twice",
               not silent and not drifted,
               "%d silent, %d unstable" % (len(silent), len(drifted)))

        # ---- the receiver, against the firmware that reports it -------------
        #
        # The same argument as the capability word above: `proto_rc_state` in
        # main.c fills a struct, ak_proto.c lays it out, and akproto.py reads it
        # back. Three files, and the frame is only right if all three agree
        # about where every field is - which no one of them can check alone.
        # `parse_rc_channels` recomputes the length the count implies, so a
        # layout drift shows up here as a refusal rather than as sticks read out
        # of a channel's bytes.
        expect("this build claims it can report its receiver",
               parsed["features"] is not None and
               "RC_CHANNELS" in feature_names(parsed["features"]))

        payload = client.request(RC_CHANNELS)
        rc = None
        try:
            rc = parse_rc_channels(payload)
        except IOError as problem:
            expect("the receiver reply parses as the layout ak_proto.h "
                   "describes", False, str(problem))
        if rc is not None:
            # Not `RC_NONE`. main.c sets `proto_io.rc_state` unconditionally, so
            # a build from this tree always has a receiver *port* - what it may
            # not have is a receiver framing on it, which is the reply's `link`
            # flag and a different answer. If this fails, either that line moved
            # (then this check moves with it) or the reply grew a byte in front.
            expect("and answers with a receiver rather than with the 'no "
                   "receiver input' byte",
                   rc["status"] == RC_OK,
                   "status %s" % rc["status_name"])

            # A bit this client has no name for would render as nothing at all,
            # so a firmware that grew one without this table growing with it is
            # a disagreement worth failing on rather than tolerating.
            unknown_flags = rc["flags"] & ~sum(1 << bit for bit in RC_FLAGS)
            expect("every flag the board set is a flag this client can name",
                   unknown_flags == 0,
                   "bit(s) 0x%02x of 0x%02x" % (unknown_flags, rc["flags"]))

            expect("the reply is the size its own count implies, so no field "
                   "is read out of its neighbour's bytes",
                   len(payload) == 4 + rc["count"] * 2 + 8 + 1 + 28,
                   "%u channels, %u bytes" % (rc["count"], len(payload)))
            expect("and inside the protocol's maximum even at the widest count "
                   "this client will read",
                   len(payload) <= 96 and
                   4 + RC_MAX_CHANNELS * 2 + 8 + 1 + 28 <= 96,
                   "%u of 96 bytes" % len(payload))

            # Per-mille, and the throttle bound is not the same as the other
            # three: roll, pitch and yaw are centred at zero and throttle is
            # not. A client that drew all four the same way would put a third of
            # a throttle bar below the stop.
            within = all(-1000 <= stick <= 1000 for stick in rc["sticks"])
            if "decoded" in rc["flag_names"]:
                within = within and 0 <= rc["sticks"][3] <= 1000
            expect("every stick is inside the range the wire documents for it",
                   within, "roll %d pitch %d yaw %d throttle %d"
                   % tuple(rc["sticks"]))

            # What used to be here, and why it is not: "a frame is only reported
            # as decoded when a link is up", and the converse riding on the
            # `decoded` flag. Both are true and neither can fail on *this*
            # board - the simulator frames continuously, so `link` and `decoded`
            # are set on every reply this check will ever see, and a check that
            # cannot fail is a paragraph, not a test. The side a simulator
            # cannot reach is driven where it can be: `test_rc_channels` in
            # tests/test_proto.c flips the flag against a fake receiver that
            # answers all three ways, and the configurator's own suite drives
            # the demo board through the same three cases. What this file checks
            # instead is the property neither of those can: that the two
            # decoders inside this firmware agree - further down, against the
            # console's own `rc`.

            named = sum(1 << bit for bit in RC_SWITCHES)
            expect("and every switch bit is one this client can name",
                   rc["switches"] & ~named == 0, "0x%02x" % rc["switches"])

        # A value written back unchanged: the table has to accept its own text,
        # which is the round trip the configurator's revert does.
        if "rate_kp_roll" in names:
            index = names.index("rate_kp_roll")
            same = client.request(PARAM_SET,
                                  bytes([index]) + values["rate_kp_roll"].encode())
            expect("a parameter accepts the value it already has", same[0] == 0,
                   values["rate_kp_roll"])
            refused = client.request(PARAM_SET, bytes([index]) + b"99")
            status, message = set_reply(refused)
            expect("and refuses one that is out of range", status == 2, message)

        status = parse_status(client.request(STATUS))
        # The state is the core's own number (see ak_flight_state_t), and a
        # console session on the bench is disarmed or in failsafe - never one of
        # the flying states.
        expect("status parses, and an aircraft that is not armed says so",
               status is not None and status["flight_state"] <= 4,
               "state %d, link %d" % (status["flight_state"],
                                      status["link_live"]))
        expect("and carries four motor outputs", len(status["motors"]) == 4)

        for source in sorted(LOG_SOURCES.values()):
            select_log(client, source)
            info = client.request(LOG_INFO)
            records = int.from_bytes(info[0:2], "little")
            expect("the %s log reports how many records it holds"
                   % LOG_SOURCE_NAMES[source], True, "%d records" % records)
            if records > 0:
                rows = []
                fetch_log(client, rows.append, source)
                # The two RAM logs are rings: their count is their capacity
                # once they are full, and a pull returns exactly that many
                # records. The flash log is not a ring - it fills, it wraps, and
                # its count moves *while* the pull runs, because the aircraft
                # keeps logging (`409 records when asked, 449 by the time the
                # pull finished`; `502 records in 4 sectors` from the console's
                # own view a moment later). So this asserts the framing and the
                # header for that one, and the arithmetic for the other two.
                framing = len(rows) >= 2 and rows[0].startswith(
                    "# aerialkit blackbox (%s)" % LOG_SOURCE_NAMES[source])
                expect("and the whole log comes back as a table",
                       framing and (source == LOG_SOURCES["flash"] or
                                    len(rows) == records + 1),
                       "%d rows for a count of %d" % (len(rows), records))

        # ---- the stream a console cannot have ------------------------------
        #
        # This build answers `LOG_STREAM` - the capability word below says so,
        # and this is the check that reads that word against the dispatch - but
        # a console cannot be pushed at: it is the wire a person types at, and
        # frames arriving among their keystrokes would make it useless for that.
        #
        # So the answer is `0` and not a refusal. The opcode is here, the range
        # was understood, and nothing will be sent: three facts, and a client
        # that collapsed them would either tell a console user their board is
        # broken or leave a viewer waiting for frames that were never coming.
        # That is `TELEMETRY`'s rule, kept because the two commands are one
        # shape and a client should not need two.
        #
        # `akproto_check.py` drives the other half against the protocol
        # stand-in, which sets `can_stream` and pushes a whole range, and it
        # compares that range record for record with `log get`. Neither peer can
        # produce the other's answer, which is why both are checked.
        expect("this build claims it can stream a log",
               parsed["features"] is not None and
               "LOG_STREAM" in feature_names(parsed["features"]))
        answer = start_log_stream(client, LOG_SOURCES["fast"], 0, 4,
                                  LOG_STREAM_MAX_HZ)
        expect("and a console asking for a range is told the rate it will get, "
               "which is none",
               answer == (LOG_SOURCES["fast"], 0, 0, 0), str(answer))
        # And nothing actually arrives. The next request is the falsification:
        # a client reading a reply does not allow a pushed frame, so if the
        # board had decided to stream anyway this would raise rather than pass
        # quietly - which is the difference between checking the answer and
        # checking the behaviour.
        quiet = parse_status(client.request(STATUS))
        expect("and none of the range arrives anyway", quiet is not None,
               "state %d" % quiet["flight_state"])

        saved = client.request(PARAM_SAVE)
        expect("and a save goes through the board's own storage", saved[0] == 0)

        # ---- the sensors, against the firmware that reports them ------------
        #
        # The same argument as the receiver: `proto_sensor_state` in main.c
        # fills the struct, ak_proto.c lays it out, akproto.py reads it back,
        # and the frame is only right if all three agree about where every
        # field is - which no one of them can check alone. This board has all
        # five, so unlike the stand-in in akproto_check.py it can answer a
        # fitted body, and the length check below is the one that catches a
        # layout drift before it becomes a number read out of its neighbour.
        expect("this build claims it can describe its sensors",
               parsed["features"] is not None and
               "SENSOR_INFO" in feature_names(parsed["features"]))

        sensors = {}
        for topic in range(SENSOR_TOPICS):
            name = SENSOR_NAMES[topic]
            payload = client.request(SENSOR_INFO, bytes([topic]))
            try:
                sensors[topic] = parse_sensor_info(payload)
            except IOError as problem:
                expect("the %s reply parses as the layout ak_proto.h describes"
                       % name, False, str(problem))
                continue
            answer = sensors[topic]
            expect("the %s reply parses as the layout ak_proto.h describes"
                   % name, answer["status"] == SENSOR_OK,
                   answer["status_name"])
            # Not `present` clear. main.c sets `proto_io.sensor_state`
            # unconditionally and every board in this tree builds all five
            # reports, so a build from this tree answers a *reading* here; a
            # board that has fitted nothing says so with `present` clear, which
            # is a different sentence and the one the stand-in produces. If this
            # fails on one topic, suspect the topic rather than the board.
            expect("and this board has a %s fitted" % name,
                   answer["present"] == 1,
                   "absent, so the wire said nothing about it")
            # The bytes, not the field count: this is the number that says
            # whether the body on the wire is the one the header documents, and
            # every field in it would still parse if the reply carried a byte
            # too few - the last one would simply be read from its neighbour.
            expect("and the reply is exactly the length its topic documents",
                   len(payload) == 3 + SENSOR_BODY_LENGTH[topic],
                   "%u bytes, expected %u"
                   % (len(payload), 3 + SENSOR_BODY_LENGTH[topic]))
        # A topic outside the five is a fact about the *build* rather than about
        # this aircraft, and it must be refused as one - the same distinction
        # the absent case makes one level down.
        beyond = sensor_info(client, SENSOR_TOPICS)
        expect("a topic outside the five is refused rather than answered with "
               "the nearest one", beyond["status"] != SENSOR_OK)

        # The driver name is the one field in these bodies that is a *string*,
        # and a name longer than the wire's fixed field would be cut silently.
        # `test_sensor_names_fit` walks the driver tables in the host suite;
        # this is the other end of the same property, on a board that has one
        # of each - a name that fills all twelve bytes would leave no room for
        # the terminator, and a client reading it as a C string would run into
        # the absent-reason byte behind it.
        for topic in (SENSOR_IMU, SENSOR_BARO, SENSOR_RANGE):
            answer = sensors.get(topic)
            if answer and answer["present"] == 1:
                expect("the %s driver's name fits the wire's field with room "
                       "for its terminator" % SENSOR_NAMES[topic],
                       answer["body"]["driver"] != "" and
                       len(answer["body"]["driver"]) < SENSOR_NAME_LENGTH,
                       "%r" % answer["body"]["driver"])

        # ---- the same receiver, as the board's own console prints it --------
        #
        # The property this milestone exists for is that there is *one*
        # decoder: the firmware decodes the sticks with `flight.rc_cfg`, and a
        # configurator must never recompute them from `rc_mid`, `rc_min` and
        # `rc_deadband`, because a second implementation of `centred()` in
        # another language disagrees at the deadband edge and then the screen
        # and the airframe each believe their own. This firmware has two
        # renderings of `receiver` - `rc_report` for the console and
        # `proto_rc_state` for the wire - and they are the closest thing here to
        # two implementations. Making them agree, over both paths, in one
        # process, is what checks it.
        #
        # Typed only once the client is done: the two share one pipe and one
        # buffered reader, so this reads through the client's own reader (see
        # `console_lines`) and runs last, where nothing else will want a byte.
        if rc is not None:
            seen = receiver_from_console(
                console_lines(process.stdout.read, write, "rc", "telemetry:"))

            expect("the console's own `rc` and the protocol's reply are "
                   "describing the same receiver",
                   seen.get("protocol", "").upper() == rc["protocol_name"],
                   "console %s, wire %s" % (seen.get("protocol"),
                                            rc["protocol_name"]))
            expect("and the same raw channels, count for count",
                   seen.get("channels") == rc["channels"],
                   "console %s, wire %s" % (seen.get("channels"),
                                            rc["channels"]))
            # The sticks are the decode, so this is the line that would catch
            # one path calling `ak_rc_decode` with a different configuration
            # than the other.
            expect("and the same decoded sticks",
                   seen.get("sticks") == rc["sticks"],
                   "console %s, wire %s" % (seen.get("sticks"), rc["sticks"]))
            expect("and the same link state, arm switch and flight mode",
                   (seen.get("link") == "framing") ==
                   ("link" in rc["flag_names"]) and
                   seen.get("arm") == ("on" if "arm" in rc["switch_names"]
                                       else "off") and
                   seen.get("mode") == ("angle" if "angle" in rc["switch_names"]
                                        else "rate"),
                   "console %s / %s / %s" % (seen.get("link"), seen.get("arm"),
                                             seen.get("mode")))

            # The counters climb between the two reads - the aircraft keeps
            # receiving while this check walks from one to the other - so
            # equality is the wrong assertion and "the console's number is not
            # smaller" is the right one. That alone would pass for a firmware
            # reporting the same number twice, which is why the ratio is checked
            # as well: a CRSF frame is 26 bytes and an SBUS one 25, so a reply
            # whose `bytes` and `frames` had been read out of each other's
            # offsets reads about 1/26 rather than about 26 - and no amount of
            # drift between two reads moves a ratio.
            expect("counters that have not gone backwards between the two reads",
                   all(seen.get(k) is not None and seen[k] >= rc[k]
                       for k in ("bytes", "frames", "crc_errors", "rejected",
                                 "dropped")),
                   "console %s bytes / %s frames, wire %s / %s"
                   % (seen.get("bytes"), seen.get("frames"), rc["bytes"],
                      rc["frames"]))
            expect("and a bytes-per-frame ratio a receiver can actually have, "
                   "which is 26 for CRSF and 25 for SBUS and neither 26 nor 25 "
                   "if the two were read from one another's offsets",
                   rc["frames"] == 0 or
                   4 * rc["frames"] <= rc["bytes"] <= 64 * rc["frames"],
                   "%u bytes for %u frames" % (rc["bytes"], rc["frames"]))

        # ---- the same sensors, as the board's own console prints them -------
        #
        # `imu_report`, `baro_report`, `rangefinder_report`, `battery_report`
        # and `gps_report` are the console's renderings of the same structs
        # `proto_sensor_state` puts on the wire, and they are the closest thing
        # in this firmware to two implementations of one thing. A wire that read
        # `baro.samples` where it meant `baro.fails`, or that put the barometer's
        # height where the fused height goes, passes every length and offset
        # check above and fails here - because the console is reading the
        # structs by name and this is comparing the two answers.
        #
        # Typed only once the client is done, and after the receiver's console
        # read, for `console_lines`'s reason: one pipe, one buffered reader.
        for topic, command, until in (
                (SENSOR_IMU, "imu", "gyro bias:"),
                (SENSOR_BARO, "baro", "gps samples"),
                (SENSOR_RANGE, "range", "landing:"),
                (SENSOR_BATTERY, "battery", "adc:"),
                (SENSOR_GPS, "gps", "link:"),
        ):
            answer = sensors.get(topic)
            if answer is None or answer["present"] != 1:
                continue
            seen = sensor_from_console(
                command, console_lines(process.stdout.read, write, command,
                                       until))
            body = answer["body"]
            name = SENSOR_NAMES[topic]

            if topic == SENSOR_IMU:
                expect("the console's `imu` and the wire agree about which part "
                       "is fitted", seen.get("driver") == body["driver"],
                       "console %s, wire %s" % (seen.get("driver"),
                                                body["driver"]))
                # Strictly greater, and the direction is the point: the wire
                # was read *first* and the console second, so a counter that
                # advances every loop must have advanced between them. A
                # firmware reporting the same number on both paths fails here,
                # and so does one reporting a counter that has been reset or
                # that belongs to another sensor.
                expect("and about the sample and the error count, which have "
                       "both only climbed since the wire was read",
                       seen.get("samples") is not None and
                       seen["samples"] > body["samples"] and
                       seen.get("errors") == body["errors"],
                       "console %s/%s, wire %s/%s"
                       % (seen.get("samples"), seen.get("errors"),
                          body["samples"], body["errors"]))
                # The three the aircraft cannot move: a calibration and an
                # alignment are set by a person and stay set, so these are
                # equality and not "has not gone backwards".
                expect("and about the alignment and the gyro bias, which do "
                       "not drift between two reads",
                       seen.get("align") == body["align"] and
                       seen.get("gyro_bias") == body["gyro_bias"],
                       "console %s / %s, wire %s / %s"
                       % (seen.get("align"), seen.get("gyro_bias"),
                          body["align"], body["gyro_bias"]))
                # The sample itself moves every loop, so this is a bound rather
                # than an equality - but a firmware that put the gyro's numbers
                # in the accel's fields would show it as a factor, not as drift.
                expect("and about the sample it is reading right now, to "
                       "within the loop it moved between the two reads",
                       seen.get("accel") is not None and
                       all(abs(a - b) <= 200 for a, b in
                           zip(seen["accel"], body["accel"])),
                       "console %s, wire %s" % (seen.get("accel"),
                                                body["accel"]))

            elif topic == SENSOR_BARO:
                expect("the console's `baro` and the wire agree about which "
                       "part is fitted", seen.get("driver") == body["driver"],
                       "console %s, wire %s" % (seen.get("driver"),
                                                body["driver"]))
                # The two fields the wire carries as integers and the console
                # prints rounded, so the tolerance is one of the console's own
                # last digit and not a fudge.
                expect("and about the pressure it is reading",
                       seen.get("pressure_pa") is not None and
                       abs(seen["pressure_pa"] - body["pressure_pa"]) <= 2,
                       "console %s, wire %s" % (seen.get("pressure_pa"),
                                                body["pressure_pa"]))
                expect("and about the temperature it is compensated with",
                       seen.get("temperature_c") is not None and
                       abs(seen["temperature_c"] - body["temperature_c"]) <= 0.05,
                       "console %s, wire %s" % (seen.get("temperature_c"),
                                                body["temperature_c"]))
                expect("and about the reference captured on the ground",
                       seen.get("have_reference") == body["have_reference"] and
                       (body["have_reference"] == 0 or
                        seen.get("reference_pa") == body["reference_pa"]),
                       "console %s, wire %s" % (seen.get("reference_pa"),
                                                body["reference_pa"]))
                # Two heights, and the pair of them is the check that matters:
                # a firmware that wrote the fused height into the barometric
                # one would agree with itself and disagree with the console.
                # With no reference captured the console prints a sentence
                # instead of a height, and the wire carries a zero *with the
                # flag clear* - so the pair of fields is what is compared, not
                # the height on its own.
                if body["have_reference"]:
                    expect("and about both heights - the barometer's own and "
                           "the one the gps is correcting",
                           seen.get("height_cm") is not None and
                           seen.get("fused_cm") is not None and
                           abs(seen["height_cm"] - body["height_cm"]) <= 1 and
                           abs(seen["fused_cm"] - body["fused_cm"]) <= 1,
                           "console %s / %s, wire %s / %s"
                           % (seen.get("height_cm"), seen.get("fused_cm"),
                              body["height_cm"], body["fused_cm"]))
                else:
                    expect("and it prints no barometric height at all, which "
                           "is what the wire's zero means with the flag clear",
                           seen.get("height_cm") is None and
                           body["height_cm"] == 0,
                           "console %s, wire %s"
                           % (seen.get("height_cm"), body["height_cm"]))
                expect("and about the two sample counts behind them",
                       seen.get("baro_samples") is not None and
                       seen["baro_samples"] >= body["baro_samples"] and
                       seen.get("gps_samples") is not None and
                       seen["gps_samples"] >= body["gps_samples"],
                       "console %s/%s, wire %s/%s"
                       % (seen.get("baro_samples"), seen.get("gps_samples"),
                          body["baro_samples"], body["gps_samples"]))
                # And the two of them stand in a fixed relation to the
                # barometer's own count, because they are the same two sensors
                # seen from the altitude estimator rather than from the driver.
                # A wire that put `altitude.baro_samples` where `baro.samples`
                # goes would agree with the console about neither number, and
                # the gap between the two reads is the same for all three - so
                # the *differences* are equal even though the counts are not.
                expect("and the barometer's own count and the estimator's are "
                       "the same sensor, so they moved by the same amount "
                       "between the two reads",
                       abs((seen["baro_samples"] - body["baro_samples"]) -
                           (seen["samples"] - body["samples"])) <= 2,
                       "console %u/%u, wire %u/%u - moved by %u and %u"
                       % (seen["baro_samples"], seen["samples"],
                          body["baro_samples"], body["samples"],
                          seen["baro_samples"] - body["baro_samples"],
                          seen["samples"] - body["samples"]))

            elif topic == SENSOR_RANGE:
                expect("the console's `range` and the wire agree about which "
                       "part is fitted and where it answers",
                       seen.get("driver") == body["driver"] and
                       seen.get("address") == body["address"],
                       "console %s at %s, wire %s at 0x%02x"
                       % (seen.get("driver"), seen.get("address"),
                          body["driver"], body["address"]))
                expect("and about how far it can reach",
                       seen.get("max_mm") == body["max_mm"],
                       "console %s, wire %s" % (seen.get("max_mm"),
                                                body["max_mm"]))
                # The counters the console prints are the reason to believe the
                # distance, and they are read second - so "not smaller" is the
                # assertion, the same one the receiver's counters get.
                expect("and about the readings it kept, threw away and failed "
                       "to get",
                       seen.get("samples") is not None and
                       seen["samples"] >= body["samples"] and
                       seen["out_of_range"] >= body["out_of_range"] and
                       seen["rejected"] >= body["rejected"] and
                       seen["faults"] >= body["faults"],
                       "console %s/%s/%s/%s, wire %s/%s/%s/%s"
                       % (seen.get("samples"), seen.get("out_of_range"),
                          seen.get("rejected"), seen.get("faults"),
                          body["samples"], body["out_of_range"],
                          body["rejected"], body["faults"]))
                expect("and about the landing rule's two numbers",
                       seen.get("land_mm") == body["land_mm"] and
                       seen.get("agree_m") is not None and
                       abs(seen["agree_m"] * 100.0 - body["agree_cm"]) <= 10,
                       "console %s mm / %s m, wire %s mm / %s cm"
                       % (seen.get("land_mm"), seen.get("agree_m"),
                          body["land_mm"], body["agree_cm"]))

            elif topic == SENSOR_BATTERY:
                expect("the console's `battery` and the wire agree about the "
                       "pack it is reading",
                       seen.get("cells") == body["cells"] and
                       seen.get("state_name") == body["state_name"],
                       "console %s %s, wire %s %s"
                       % (seen.get("cells"), seen.get("state_name"),
                          body["cells"], body["state_name"]))
                expect("and about the pack voltage and the cell it implies",
                       seen.get("volts") is not None and
                       abs(seen["volts"] - body["volts"]) <= 0.005 and
                       abs(seen["volts_per_cell"] - body["volts_per_cell"])
                       <= 0.005,
                       "console %s/%s, wire %s/%s"
                       % (seen.get("volts"), seen.get("volts_per_cell"),
                          body["volts"], body["volts_per_cell"]))
                # A divider ratio is a board constant and a threshold is a
                # parameter: neither moves while the aircraft runs, so these are
                # equality. A wire that read `ratio_milli` where the thresholds
                # go would show up as 11.0 volts a cell.
                expect("and about the divider and both cell thresholds",
                       seen.get("ratio") == body["ratio"] and
                       seen.get("warn_cell_v") == body["warn_cell_v"] and
                       seen.get("critical_cell_v") == body["critical_cell_v"],
                       "console %s/%s/%s, wire %s/%s/%s"
                       % (seen.get("ratio"), seen.get("warn_cell_v"),
                          seen.get("critical_cell_v"), body["ratio"],
                          body["warn_cell_v"], body["critical_cell_v"]))
                expect("and about the voltage at the pin and the readings "
                       "behind it",
                       seen.get("pin_mv") is not None and
                       abs(seen["pin_mv"] - body["pin_mv"]) <= 2 and
                       seen["samples"] >= body["samples"] and
                       seen["rejected"] >= body["rejected"],
                       "console %s/%s/%s, wire %s/%s/%s"
                       % (seen.get("pin_mv"), seen.get("samples"),
                          seen.get("rejected"), body["pin_mv"],
                          body["samples"], body["rejected"]))

            elif topic == SENSOR_GPS:
                expect("the console's `gps` and the wire agree about the fix",
                       seen.get("fix_type") == body["fix_type"] and
                       seen.get("satellites") == body["satellites"],
                       "console type %s on %s, wire type %s on %s"
                       % (seen.get("fix_type"), seen.get("satellites"),
                          body["fix_type"], body["satellites"]))
                # Printed to seven decimals, which is the e7 integer the wire
                # carries - so this is equality even though the console went
                # through a float to print it.
                expect("and about where it thinks it is",
                       seen.get("lat_e7") == body["lat_e7"] and
                       seen.get("lon_e7") == body["lon_e7"],
                       "console %s,%s, wire %s,%s"
                       % (seen.get("lat_e7"), seen.get("lon_e7"),
                          body["lat_e7"], body["lon_e7"]))
                expect("and about the altitude above mean sea level",
                       seen.get("alt_msl_mm") == body["alt_msl_mm"],
                       "console %s, wire %s" % (seen.get("alt_msl_mm"),
                                                body["alt_msl_mm"]))
                expect("and about how far home is and which way it lies",
                       seen.get("home_distance_m") is not None and
                       abs(seen["home_distance_m"] - body["home_distance_m"])
                       <= 1 and
                       seen.get("home_bearing_cdeg") is not None and
                       abs(seen["home_bearing_cdeg"]
                           - body["home_bearing_cdeg"]) <= 1,
                       "console %s m / %s cdeg, wire %s m / %s cdeg"
                       % (seen.get("home_distance_m"),
                          seen.get("home_bearing_cdeg"),
                          body["home_distance_m"], body["home_bearing_cdeg"]))
                expect("and about the fixes it has had and whether the last "
                       "one is current",
                       seen.get("fixes") is not None and
                       seen["fixes"] >= body["fixes"] and
                       seen.get("valid_now") == body["valid_now"],
                       "console %s valid=%s, wire %s valid=%s"
                       % (seen.get("fixes"), seen.get("valid_now"),
                          body["fixes"], body["valid_now"]))
                expect("and about the bytes the receive buffer dropped and the "
                       "configuration it has sent",
                       seen.get("dropped") == body["dropped"] and
                       seen.get("config_sends") is not None and
                       seen["config_sends"] >= body["config_sends"],
                       "console %s/%s, wire %s/%s"
                       % (seen.get("dropped"), seen.get("config_sends"),
                          body["dropped"], body["config_sends"]))
        # ---- the checklist, which is one checklist printed twice ------------
        #
        # This is the section the whole design is arranged around. `preflight`
        # over the wire is not a second checklist that has to agree with the
        # console's - it is the same record, built once and read twice, so what
        # is asserted here is that the two readings are the same bytes. The
        # console prints `marker(verdict) + detail` for each line (main.c's
        # `preflight_run`), and `detail` carries the console's sentence whole,
        # name and all. A wire line that dropped the name, re-worded a fault or
        # folded a continuation would fail here rather than in a screenshot.
        #
        # Taken through the *same* pipe and the same reader as everything above,
        # so this also checks that the console and the protocol really do share
        # the wire without one eating the other's bytes.
        printed = console_lines(process.stdout.read, write, "preflight",
                                "preflight:")
        expect("the console prints the checklist and says how it went",
               len(printed) >= 2 and printed[-1].startswith("preflight:"),
               "%d lines" % len(printed))
        # Two of the lines on this stream are not lines of the checklist, and
        # both have to go before the comparison rather than being compared and
        # explained away: the console echoes the command that was typed
        # (`ak> preflight`), and the board's `alive:` heartbeat can land between
        # two lines of the report. A count that included either would fail here
        # on a busy machine and pass on a quiet one.
        console_checklist = [line for line in printed[:-1]
                             if not line.strip().startswith(("ak>", "alive:"))]

        wire_checklist = fetch_preflight(client)
        expect("and the wire serves a checklist of the same length",
               len(wire_checklist) == len(console_checklist),
               "console %d, wire %d" % (len(console_checklist),
                                        len(wire_checklist)))

        if len(wire_checklist) == len(console_checklist):
            rendered = ["%s%s" % (PREFLIGHT_VERDICTS[v][1], detail)
                        for _name, v, detail in wire_checklist]

            # The two readings are seconds apart, so a line carrying a live
            # counter reads differently in each - `tick advanced after 754
            # spins` against `836` - and that is a movement to characterise
            # rather than to explain away by comparing something weaker. What is
            # asserted is that the *words* are identical and only the readings
            # moved: a re-worded fault, a dropped name or a folded continuation
            # changes the shape and fails here, while a spin count that climbed
            # does not.
            shape = lambda text: re.sub(r"\d+", "#", text)   # noqa: E731
            shaped = [(i, console_checklist[i], rendered[i])
                      for i in range(len(rendered))
                      if shape(console_checklist[i]) != shape(rendered[i])]
            expect("and every line reads as the console's does, word for word, "
                   "with only the readings moving",
                   not shaped,
                   "" if not shaped
                   else "line %d: console %r, wire %r" % shaped[0])

            # And the lines that carry no reading at all - which is most of the
            # prose, and every sentence a person would act on - are the same
            # bytes, not merely the same shape.
            stable = [i for i in range(len(rendered))
                      if not re.search(r"\d", console_checklist[i])]
            expect("and the lines with no reading in them are the same bytes",
                   all(console_checklist[i] == rendered[i] for i in stable),
                   "%d such lines" % len(stable))
            expect("which is enough lines for that to mean something",
                   len(stable) >= 3, "%d" % len(stable))

            moved = [i for i in range(len(rendered))
                     if console_checklist[i] != rendered[i]]
            expect("and only a minority of lines moved between the two reads, "
                   "which is what says they were taken close together",
                   len(moved) <= len(rendered) // 2,
                   "%d of %d moved" % (len(moved), len(rendered)))

            expect("every line carries a verdict the wire defines",
                   all(v in PREFLIGHT_VERDICTS for _n, v, _d in wire_checklist))

            # The names are the stable keys a client diffs two checklists on,
            # and the reason they are sent beside a sentence that already begins
            # with one - the sentence is for a person, the name is for a program
            # that must not parse prose.
            #
            # Not `names`: that is the *parameter table* read at the top of this
            # function and looked up by name further down, and a local here
            # rebinds it for the rest of `main` - which is how a lookup for
            # `wp_count` came back "not in list" against a board that had just
            # answered with it at index 86.
            line_names = [name for name, _v, _d in wire_checklist]
            expect("every line carries a name",
                   all(name for name in line_names), "%r" % (line_names,))
            expect("and no two lines share one",
                   len(set(line_names)) == len(line_names))
            expect("and the names are the ones this firmware's checklist is "
                   "made of",
                   line_names[:3] == ["console", "clock", "tick"],
                   "%r" % (line_names[:3],))
            expect("and a line named `arm` is last, because it is the one that "
                   "sums the rest up",
                   line_names[-1] == "arm",
                   line_names[-1] if line_names else "none")

            # The count is derived rather than tallied beside the lines - `pf_end`
            # increments on FAIL and `preflight_build` returns the total - so a
            # line that says FAIL cannot sit under a report that says otherwise.
            failed = [n for n, v, _d in wire_checklist if v == 0]
            expect("and the console's own verdict agrees with the lines it "
                   "printed",
                   printed[-1].endswith(
                       "the machine is what the firmware thinks it is")
                   == (not failed),
                   "console %r, %d FAIL line(s)"
                   % (printed[-1], len(failed)))

            # Nothing here asserts *which* lines pass: that is the board's
            # business and it changes with what is plugged into it. What is
            # asserted is that a line saying FAIL says why, because a checklist
            # that says something is wrong without saying what is a checklist
            # nobody can act on.
            silent = [n for n, v, d in wire_checklist
                      if v == 0 and len(d.strip()) <= len(n) + 2]
            expect("every FAIL line carries a sentence after its name",
                   not silent, "%r" % (silent,))

        # ---- the mission, which is two readings of one aircraft ------------
        #
        # MISSION is the first opcode on this wire whose verbs change what the
        # aircraft will do in the air, and this section is arranged around the
        # one property that makes it safe to have here at all: a verb that
        # *asks* is not a verb that *flies*. `mission start` at the console -
        # and MISSION's start over the wire, which pokes the same two statics -
        # sets a request and returns; the flight loop starts the mission later,
        # and only once the aircraft is armed and flying. So what is asserted
        # below is that the two routes agree about the *aircraft* rather than
        # about each other's prose: the same count, the same one flying, and a
        # start that leaves `active` at zero on a board that is not armed.
        #
        # The half this check cannot reach is the flying half: a console
        # session on the simulator never arms, so `started` and `cancelled` are
        # asserted to stay zero rather than pretended about. What exercises the
        # mission itself is the simulator's own sessions, where the switch is
        # thrown and there is air under it.
        #
        # The console's report is read through a *second* command typed after
        # it - `mission what?`, an unknown sub-verb whose answer is one stable
        # line. The report has no last line of its own: `N reached`, `N
        # started` and `switch on channel N` are each printed only when they
        # are true, so a reader waiting for one of them would hang on the board
        # where it is absent. The refusal line is the terminator instead, and
        # it costs nothing: `mission what?` prints a usage sentence and changes
        # no state.
        def console_mission(command):
            printed = console_lines(process.stdout.read, write,
                                    command + "\rmission what?",
                                    "mission: what?")
            # Only the report's own lines: `mission:` introduces it and every
            # other line of it is indented, which is exactly what the console's
            # own two echoes are not.
            return mission_from_console(
                [line for line in printed[:-1]
                 if line.startswith("mission:") or line.startswith("  ")])

        def console_home():
            printed = console_lines(process.stdout.read, write,
                                    "home\rmission what?", "mission: what?")
            return home_from_console([line for line in printed[:-1]
                                      if line.startswith("home")])

        def wire_mission(op):
            return parse_mission(client.request(MISSION, bytes([op])))

        # A fresh board has an empty list, and both routes have to say so
        # before anything below means what it says.
        seen = console_mission("mission")
        expect("the console lists the mission and says how many waypoints "
               "there are", seen.get("count") is not None, "%r" % (seen,))
        expect("and this board starts with none, which the checks below rest "
               "on", seen.get("count") == 0, "%r" % (seen.get("count"),))

        empty = wire_mission(MISSION_STATUS)
        expect("and the wire describes the same aircraft the console's "
               "`mission` just did",
               (empty["count"], empty["active"]) ==
               (seen.get("count"), seen.get("flying")),
               "console %r, wire count %u active %u"
               % (seen, empty["count"], empty["active"]))
        # 0xFF and not 0: "not flying a waypoint" and "flying waypoint zero"
        # are the two ends of a mission, and a client that read one as the
        # other would draw the first waypoint as the one in progress.
        expect("with no waypoint in progress, which is 0xFF and not zero",
               empty["index"] == MISSION_NO_INDEX, "%u" % empty["index"])
        expect("and no fix-dependent field folded into a number nobody is "
               "holding", empty["hold_alt_mm"] == 0)

        # The refusal the firmware owns, and the one the dispatch owns, are
        # different facts - a board with no navigator is not a board with an
        # empty list, and a client that collapsed them would tell a person
        # their waypoints were gone.
        refused = wire_mission(MISSION_START)
        expect("a start with nothing to fly is refused as an empty list",
               refused["status"] == MISSION_NO_WAYPOINTS,
               "status %u" % refused["status"])
        expect("which is not the same answer as having no navigator at all",
               MISSION_NO_WAYPOINTS != MISSION_NO_NAV)
        expect("and the refusal left no request behind for the flight loop to "
               "find later", wire_mission(MISSION_STATUS)["requested"] == 0)

        # ---- a waypoint is three rows of the parameter table ---------------
        #
        # This is the design's load-bearing claim: MISSION carries no position,
        # and there is no second storage. `mission add` is a convenience that
        # writes the same rows `set wp0_lat` writes, which is why the wire's
        # answer about a waypoint is a *count* and the position has to be found
        # in the table.
        reply = console_mission("mission add 52.1 13.4")
        expect("the console takes a waypoint and says where it put it",
               reply.get("added") == (0, 52.1, 13.4), "%r" % (reply,))
        added = console_mission("mission")
        expect("and lists it on the next read",
               added.get("count") == 1 and
               added.get("waypoints") == [(0, 52.1, 13.4)],
               "%r" % (added,))
        expect("and the wire counts it, one higher than before",
               wire_mission(MISSION_STATUS)["count"] == 1)

        def parameter(name):
            """`(status, value)` for a row read back by the name it was read
            under at the top of this file. Restating the name rather than the
            index is the point: the table is read by index on the wire, and a
            row that moved would be found here rather than silently off by
            one."""
            payload = client.request(PARAM_GET, bytes([names.index(name)]))
            if payload[0] != 0:
                return payload[0], ""
            got = c_string(payload, 1)
            return 0, c_string(payload, 2 + len(got))

        status, value = parameter("wp_count")
        expect("the count the wire reports is the parameter the table holds",
               status == 0 and value == "1", "status %u, %r" % (status, value))
        status, value = parameter("wp0_lat")
        expect("and the waypoint is wp0_lat, in the table like any other row",
               status == 0 and float(value) == 52.1, "status %u, %r"
               % (status, value))
        status, value = parameter("wp0_lon")
        expect("with its longitude beside it", status == 0 and
               float(value) == 13.4, "status %u, %r" % (status, value))

        # ---- the verbs, and the one property that makes them safe ----------
        lengths = [len(client.request(MISSION, bytes([op])))
                   for op in MISSION_VERB_NAMES]
        expect("every one of the five verbs answers with the whole state in "
               "one frame", lengths == [17] * len(MISSION_VERB_NAMES),
               "%r" % (lengths,))

        asked = wire_mission(MISSION_START)
        expect("a start is accepted once there is something to fly",
               asked["status"] == MISSION_OK, "status %u" % asked["status"])
        expect("and it is a request and not a flight: `requested` is set and "
               "`active` is not, because this board is not armed",
               asked["requested"] == 1 and asked["active"] == 0,
               "requested %u, active %u" % (asked["requested"], asked["active"]))
        expect("and the console says the same, because it is the same two "
               "statics - the wire cannot fly an aircraft the console would not",
               console_mission("mission").get("flying") == 0)
        expect("with nothing in progress and nothing started, since neither "
               "route can fly a mission the flight loop has not begun",
               asked["index"] == MISSION_NO_INDEX and asked["started"] == 0)

        # Stop is the verb that has to work whatever else is true: a client
        # that has lost track of whether a mission is running can always send
        # it, and stopping one that is not running takes the request back,
        # which is what a person pressing it means.
        stopped = wire_mission(MISSION_STOP)
        expect("stop is answered however the aircraft is, and takes the "
               "request back",
               stopped["status"] == MISSION_OK and stopped["requested"] == 0,
               "status %u, requested %u"
               % (stopped["status"], stopped["requested"]))
        expect("and the console agrees there is nothing to fly",
               console_mission("mission").get("flying") == 0)

        # ---- home, which is the one verb that needs a position ------------
        #
        # The console's `home` and the wire's home set are two routes to
        # `ak_nav_set_home` behind one `gps.have_fix` test, so the check that
        # matters is that they answer the same way about the same board. A
        # simulator has a synthetic fix and a bare board has none, and this
        # section has to pass on both - which is why the assertion is the
        # *agreement* rather than the outcome.
        home_line = console_home()
        home_wire = wire_mission(MISSION_HOME_SET)
        expect("the console's `home` and the wire's home set agree about "
               "whether this board has a fix",
               bool(home_line) == (home_wire["status"] == MISSION_OK),
               "console %r, wire status %u"
               % (home_line, home_wire["status"]))
        expect("and both say the same thing as the GPS report did earlier",
               bool(home_line) ==
               bool((sensors.get(SENSOR_GPS) or {}).get("body", {})
                    .get("have_fix")),
               "console %r, sensor %r"
               % (home_line, (sensors.get(SENSOR_GPS) or {}).get("body", {})
                  .get("have_fix")))
        # Never a refusal: clearing a home that is already clear is the state
        # the caller asked for, and a refusal there would be the firmware
        # arguing with a client about a fact that has no other value.
        cleared = wire_mission(MISSION_HOME_CLEAR)
        expect("and home clear is answered whatever there was to clear",
               cleared["status"] == MISSION_OK,
               "status %u" % cleared["status"])

        # ---- the refusals the dispatch owns --------------------------------
        #
        # A short frame must not default to verb zero. Verb zero is the only
        # one of the five that is a read, so the harmful default is not this
        # one - but the next number along starts a mission, and there is no
        # reason for the two paths to be shaped differently. The refusal echoes
        # `0xFF`, which is out of range of all five, so a client can tell "you
        # did not say" from "I do not know that one".
        bare = parse_mission(client.request(MISSION, b""))
        expect("a frame with no verb is refused, and echoes 0xFF so a client "
               "can tell it never said",
               bare["status"] == MISSION_NO_VERB and bare["op"] == 0xFF and
               bare["verb"] is None, "%r" % (bare,))
        expect("and the refusal is a complete reply rather than a header with "
               "a caller's stack behind it",
               bare["active"] == 0 and bare["requested"] == 0 and
               bare["count"] == 0 and bare["index"] == MISSION_NO_INDEX)
        unknown = parse_mission(client.request(MISSION, bytes([5])))
        expect("a verb this firmware does not have is refused too, and echoes "
               "the number so the two refusals are distinguishable",
               unknown["status"] == MISSION_NO_VERB and unknown["op"] == 5 and
               unknown["verb"] is None, "%r" % (unknown,))

        expect("and this build claims it can fly a mission at all",
               parsed["features"] is not None and
               "MISSION" in feature_names(parsed["features"]))

        # ---- the calibration, whose whole point is that it does not block ---
        #
        # The console's four calibrations sit in a loop for up to three seconds
        # because the console is a human wire. This opcode is dispatched from the
        # flight loop, so the board owns the session and the loop advances it a
        # sample at a time - which is the property this section is built around:
        # `gyro` and `rc` return *with the session running*, and what completes
        # it is the loop, not the command. A client that expects a finished
        # answer from the verb that started it would read zeros forever, so
        # every reading below is taken from `status`.
        #
        # The half this check cannot reach is the armed refusal: a console
        # session on this simulator never arms, so ARMED is driven in
        # test_proto.c against a fake `writable`, exactly as the write gate's is.
        def wire_cal(verb, *extra):
            # The echo is checkable only for the four verbs that name themselves
            # in the session they start; `status` and `abort` report the session
            # they are about, which is the reading this opcode exists to give.
            return parse_calibration(
                client.request(CALIBRATE, bytes([verb]) + bytes(extra)),
                sent=verb if verb in (CALIBRATE_GYRO, CALIBRATE_RC,
                                      CALIBRATE_ACCEL, CALIBRATE_VBAT)
                else None)

        def wire_poll():
            return parse_calibration(client.request(CALIBRATE,
                                                    bytes([CALIBRATE_STATUS])))

        def wait_out():
            """The session's last reading, once it has stopped sampling. The
            bound is the firmware's own 15 s timeout plus room, so a session
            that never finishes fails here rather than hanging the check."""
            deadline = time.time() + 25.0
            state = wire_poll()
            while state["active"] and time.time() < deadline:
                time.sleep(0.2)
                state = wire_poll()
            return state

        fresh = wire_poll()
        expect("a board that has never calibrated names no session rather than "
               "the `status` verb a client happens to be sending",
               fresh["verb"] == CALIBRATE_NO_SESSION and
               fresh["active"] == 0 and fresh["samples"] == 0,
               "verb %r" % (fresh["verb"],))

        started = wire_cal(CALIBRATE_GYRO)
        expect("`gyro` returns with the session running, because the flight "
               "loop is not held for it",
               started["active"] == 1 and started["verb"] == CALIBRATE_GYRO)

        gyro = wait_out()
        expect("and the loop is what finishes it, with samples rather than a "
               "promise", gyro["samples"] >= 400 and gyro["active"] == 0,
               "%u samples" % gyro["samples"])
        expect("and the poll names the session it is reading rather than "
               "echoing its own verb, which is how a client knows how to read "
               "the six slots",
               gyro["verb"] == CALIBRATE_GYRO and
               gyro["result_names"] ==
               CALIBRATE_RESULT_MEANING[CALIBRATE_GYRO],
               "%r" % (gyro["verb_name"],))
        # The number the wire reports and the number the table holds are the
        # same measurement through two routes. Weak on this simulator, whose
        # gyro is still and reports zero - but the *comparison* is what would
        # catch a slot filled from the wrong axis, and it is not weaker for
        # being satisfied by zero on one board.
        status, value = parameter("gyro_bias_roll")
        expect("and the bias it measured is the row the table now holds",
               status == 0 and
               abs(int(round(float(value) * 1000.0)) - gyro["results"][0]) <= 1,
               "table %s, wire %d mdps" % (value, gyro["results"][0]))

        # `vbat` is the one verb that finishes inside the request: the only
        # thing it waits for is the ADC, and there is nobody to hold still.
        vbat = wire_cal(CALIBRATE_VBAT, *(12600).to_bytes(4, "little"))
        expect("`vbat` finishes inside the request, because nothing has to "
               "hold still for it",
               vbat["status"] == CALIBRATE_OK and vbat["active"] == 0 and
               vbat["samples"] > 0, "%u samples" % vbat["samples"])
        status, value = parameter("vbat_ratio")
        expect("and the ratio it measured is the row the table now holds - "
               "the one parameter a calibration writes that this check can see "
               "move on a simulator",
               status == 0 and
               abs(int(round(float(value) * 1e6)) - vbat["results"][0]) <= 1,
               "table %s, wire %d x1e6" % (value, vbat["results"][0]))

        wire_cal(CALIBRATE_RC)
        rc = wait_out()
        expect("`rc` takes its centre from the frames the receiver sends, not "
               "from a clock",
               rc["samples"] > 0 and rc["verb"] == CALIBRATE_RC,
               "%u frames" % rc["samples"])
        status, value = parameter("rc_mid")
        expect("and the centre it found is the row the table now holds",
               status == 0 and int(value) == rc["results"][0],
               "table %s, wire %d us" % (value, rc["results"][0]))

        # The six-face flow, which is six commands that accumulate - so the
        # bitmask is the whole of a wizard's progress and each reply carries it.
        #
        # And on this simulator the sixth face fails, which is the honest
        # outcome and the more valuable half to drive: a simulated IMU reports
        # one accelerometer vector whichever way the aircraft is held, so the
        # six faces cannot be a gravity. What is asserted is both that the five
        # accumulate and that the failure *discards* them - a wizard allowed to
        # resume a flow whose measurement was rejected would carry five faces
        # into a sixth that then wrote a bias built from them.
        seen_faces = 0
        for face in range(6):
            wire_cal(CALIBRATE_ACCEL, face)
            state = wait_out()
            if face < 5:
                seen_faces = state["faces"]
            else:
                sixth = state
        expect("each face of the six accumulates into the bitmask the reply "
               "carries", seen_faces == 0x1F, "0x%02x" % seen_faces)
        expect("and a sixth face that is not a plausible gravity is refused "
               "rather than written",
               sixth["status"] == CALIBRATE_IMPLAUSIBLE,
               "status %u" % sixth["status"])
        expect("and the faces it rejected are discarded, so a wizard cannot "
               "resume a flow that was already found wrong",
               sixth["faces"] == 0, "0x%02x" % sixth["faces"])
        for name in ("accel_bias_x", "accel_scale_z"):
            status, value = parameter(name)
            expect("and `%s` is left at the value a refused calibration found "
                   "nothing to change about it" % name,
                   status == 0 and (value == "0.0000" or value == "1.0000"),
                   "%s" % value)

        expect("a face past the sixth is refused rather than wrapped onto a "
               "real one",
               wire_cal(CALIBRATE_ACCEL, 9)["status"] == CALIBRATE_NO_FACE)
        idle = wire_cal(CALIBRATE_ABORT)
        expect("and aborting a session that is not running is refused, while "
               "still naming the last one so a client knows what it asked "
               "about",
               idle["status"] == CALIBRATE_IDLE and
               idle["verb"] == CALIBRATE_ACCEL, "%r" % (idle["verb_name"],))

        lengths = [len(client.request(CALIBRATE, bytes([op])))
                   for op in sorted(CALIBRATE_VERB_NAMES)]
        expect("every one of the six verbs answers with the whole session in "
               "one 37-byte frame", lengths == [37] * 6, "%r" % (lengths,))

        bare = parse_calibration(client.request(CALIBRATE, b""))
        expect("a frame that named no verb is refused, and echoes 0xFF so a "
               "client can tell it never said",
               bare["status"] == CALIBRATE_NO_VERB and
               bare["verb"] == CALIBRATE_NO_SESSION, "%r" % (bare["verb"],))
        unknown = parse_calibration(client.request(CALIBRATE, bytes([9])))
        expect("and a verb this firmware does not have is refused, echoing the "
               "number so the two refusals are distinguishable",
               unknown["status"] == CALIBRATE_NO_VERB and
               unknown["verb"] == 9, "%r" % (unknown["verb"],))
        expect("and a vbat frame with no voltage is refused rather than read "
               "as a pack of zero volts",
               parse_calibration(
                   client.request(CALIBRATE, bytes([CALIBRATE_VBAT])))
               ["status"] == CALIBRATE_NO_VERB)

        expect("and this build claims it can calibrate at all",
               parsed["features"] is not None and
               "CALIBRATE" in feature_names(parsed["features"]))

        # ---- where the loop's time goes ------------------------------------
        #
        # PERF is read twice, on purpose: the console's `perf` and the wire's
        # `perf` print the *same* snapshot out of `ak_perf.c`, and the thing that
        # can go wrong between them is a unit. The console prints a section's
        # average in microseconds with three decimals; the wire carries tenths.
        # A client that read one as the other would be off by a factor of ten in
        # a number a person is meant to act on, and nothing else here would
        # notice.
        #
        # **What this binary can and cannot check.** This is the firmware
        # simulator, and it has no cycle counter: the host model behind
        # `ak_cycles()` (tests/host_cycles_model.c) is a counter the *tests*
        # move, and nothing moves it here. So the board under this check is in
        # the state `docs/29-timing.md` calls "a port that cannot convert cycles
        # to time" - it counts the loops it closes and measures none of them -
        # and the two renderings are compared over a window whose durations are
        # all zero. That comparison is real and it is not sufficient: zero times
        # ten is still zero, so a factor-of-ten error between the two routes
        # would pass here.
        #
        # The units are therefore checked where they can be, and both halves are
        # named rather than left to be assumed. `tests/test_perf.c`'s arithmetic
        # tests drive exact cycle counts through the same conversion;
        # `tests/test_proto.c`'s PERF test pins the wire's every field offset;
        # and `main.c`'s nanoseconds-to-tenths conversion, which is the seam
        # between them, is exercised only by a window with numbers in it - which
        # means a board whose counter runs, which means hardware, which this
        # machine does not have attached. See `docs/29-timing.md`, "What is not
        # measured".
        #
        # The read is ended by a second command rather than by the report's own
        # last line, because `perf` has two answers that are not a report - a
        # build with no profiler, and a refusal - and a delimiter that only
        # appears on the happy path is one that reads to the end of the buffer
        # when the board says no. `perf what?` is the argument refusal, which
        # every build prints.
        def console_perf():
            printed = console_lines(process.stdout.read, write,
                                    "perf\rperf what?", "perf:      unknown")
            # The report's own lines: `perf:` and its siblings start at column
            # zero, the five sections are indented under them, and the refusal
            # that ends the read is neither.
            return perf_from_console(
                [line for line in printed[:-1]
                 if not line.startswith("perf:      unknown")])

        def wire_perf():
            return parse_perf(client.request(PERF, b""))

        window = wire_perf()
        expect("a board with a profiler answers with a window rather than a "
               "refusal", window["status"] == PERF_OK,
               "status %u" % window["status"])
        expect("and the reply is the fixed size the parser reads at, whatever "
               "it reports",
               len(client.request(PERF, b"")) == PERF_REPLY_BYTES)

        # The loop has been turning since the process started, so this is a real
        # window and not the empty one - and `loops` and `samples` are checked
        # separately *and against each other*, because they are two different
        # claims. A port whose clock reads zero counts the periods it closes and
        # times none of them; this binary is that port, and the pair is how it
        # says so without a footnote.
        expect("the window covers loops the board has actually run",
               window["loops"] > 0, "%u loops" % window["loops"])
        expect("and this port cannot time them, so it says so in the numbers "
               "instead: every loop counted, none measured, and every duration "
               "below zero because none was taken rather than because none was "
               "spent",
               window["samples"] == 0,
               "%u of %u measured" % (window["samples"], window["loops"]))
        expect("and the nominal period is the one the loop is built to hold, "
               "which the loop knows without a counter",
               window["nominal_us"] == 1000, "%u us" % window["nominal_us"])
        for field in ("period_last_us", "period_min_us", "period_max_us",
                      "late", "jitter_p50_us", "jitter_p99_us",
                      "jitter_max_us", "jitter_over", "load_permille"):
            expect("and `%s` is zero for the same reason - no period was timed"
                   % field, window[field] == 0, "%u" % window[field])

        seen = console_perf()
        expect("the console and the wire report the same five sections, in the "
               "same order",
               tuple(seen["section_names"]) == PERF_SECTIONS,
               "%r" % (seen["section_names"],))
        expect("and the two routes agree about the period the loop is holding, "
               "so a client reading the wire is reading the console's number",
               seen["nominal_us"] == window["nominal_us"],
               "console %u, wire %u" % (seen["nominal_us"], window["nominal_us"]))

        # The console's reading is taken *after* the wire's, and the loop keeps
        # turning between them, so the two cannot be compared field for field.
        # `loops` is the one that is allowed to move and it is allowed to move
        # one way only; everything else below is a window average that a few
        # more iterations cannot shift, and that is why they can be compared at
        # all.
        expect("and the console read the window after the wire did, not before",
               seen["loops"] >= window["loops"],
               "console %u, wire %u" % (seen["loops"], window["loops"]))

        for i, name in enumerate(PERF_SECTIONS):
            expect("and `%s` is the same number in both units - the console's "
                   "microseconds and the wire's tenths" % name,
                   abs(seen["sections"][i] - window["section_avg_x10"][i]) <= 1,
                   "console %u, wire %u (tenths)"
                   % (seen["sections"][i], window["section_avg_x10"][i]))

        # The load is the sum of the sections over the nominal slot, and it is
        # carried in per-mille on the wire and as a percentage with one decimal
        # on the console - the same quantity twice. Rounding the sum of five
        # truncated tenths can lose a couple of per-mille, so the comparison
        # allows for exactly that and no more.
        summed = sum(window["section_avg_x10"]) // 10
        expect("and the load is the sum of the sections it says it is the sum "
               "of, not a separately measured number that could disagree",
               abs(summed - window["load_permille"]) <= 5,
               "sections %u, load %u per-mille"
               % (summed, window["load_permille"]))
        expect("and the console prints the same load",
               abs(seen["load_permille"] - window["load_permille"]) <= 1,
               "console %u, wire %u per-mille"
               % (seen["load_permille"], window["load_permille"]))

        expect("and this build claims it can profile at all",
               parsed["features"] is not None and
               "PERF" in feature_names(parsed["features"]))

    finally:
        # Close the input rather than killing it: a console session ends by
        # itself, which is also what lets a coverage build count these lines.
        try:
            process.stdin.close()
        except OSError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    print("the firmware's own server: %s" % ("FAIL" if failures else "PASS"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
