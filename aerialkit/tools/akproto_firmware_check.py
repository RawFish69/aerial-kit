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
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from akproto import (Client, HELLO, LOG_INFO, LOG_SOURCES, LOG_SOURCE_NAMES,
                     PARAM_GET, PARAM_SAVE, PARAM_SET, PARAM_TEXT,
                     RC_CHANNELS, RC_FLAGS, RC_MAX_CHANNELS, RC_OK,
                     RC_SWITCHES, SENSOR_BARO, SENSOR_BATTERY,
                     SENSOR_BODY_LENGTH, SENSOR_GPS,
                     SENSOR_IMU, SENSOR_INFO, SENSOR_NAME_LENGTH, SENSOR_NAMES,
                     SENSOR_OK, SENSOR_RANGE, SENSOR_TOPICS, STATUS, c_string,
                     feature_names,
                     fetch_log, fetch_param_help, fetch_param_info,
                     parse_hello, parse_rc_channels, parse_sensor_info,
                     parse_status, select_log, sensor_info,
                     set_reply)  # noqa: E402 - after sys.path

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
        # real 92-row table rather than on a synthetic one built to suit it.
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
