#!/usr/bin/env python3
"""End-to-end check of the config protocol: the real client against the real
firmware code.

    make proto-test

The C tests check the firmware's half against frames built from the wire
description. This checks the *other* half - the client that a tool would use -
against that same firmware code, over a pipe. Two halves that pass their own
tests and disagree in the middle is the classic way a protocol fails, and it is
not something either half can catch alone.
"""

import os
import struct
import subprocess
import sys

from akproto import (Client, HELLO, LOG_FIELDS, LOG_INFO, LOG_SOURCES, PARAM_GET,
                     PARAM_SET, PARAM_SAVE, RC_CHANNELS, RC_MAX_CHANNELS, RC_NONE,
                     SENSOR_BARO, SENSOR_BATTERY, SENSOR_BODY_LENGTH, SENSOR_GPS,
                     SENSOR_IMU, SENSOR_INFO, SENSOR_NO_SUCH, SENSOR_OK,
                     SENSOR_RANGE, SENSOR_TOPICS,
                     STATUS, c_string,
                     fetch_log, parse_hello, parse_rc_channels, parse_sensor_info,
                     parse_status, parse_telemetry, select_log, sensor_info,
                     set_reply)

# A telemetry frame the ESP32 produced under QEMU on 2026-09-14, the first one
# after a client subscribed at 20 Hz. It is the `frame` line of
# docs/evidence/esp32-network.txt, byte for byte. The simulated part of `make
# proto-test` has no network and cannot push one, so without this the client's
# telemetry parser would only ever be checked against frames the client itself
# built - which is the circularity that hid the GPS offsets for a milestone.
#
# aa 55 01 08 1a  = sync, version 1, telemetry, 26 bytes
# cc 10 00 00     = uptime 4300 ms
# 16 zero bytes   = disarmed, no link, no fix, no attitude, no position
# 4 zero bytes    = motors stopped
# e9 d6           = crc16 of everything after the sync pair
CAPTURED_TELEMETRY = (
    "aa5501081acc10000000000000000000000000000000000000000000000000e9d6")


def check_captured_telemetry():
    """The client against a frame from the target, not from itself."""
    frame = bytes.fromhex(CAPTURED_TELEMETRY)

    # Fed one byte at a time, the way a socket delivers it.
    stream = iter(frame)
    client = Client(lambda n: bytes([next(stream)]), lambda data: None)
    command, payload = client.read_frame()

    expect("a telemetry frame from the ESP32 frames correctly",
           command == 0x08)
    telemetry = parse_telemetry(payload)
    expect("its uptime is where the client thinks it is",
           telemetry["uptime_ms"] == 4300)
    expect("and the rest of it is the status body",
           telemetry["flight_state"] == 0 and telemetry["motors"] == [0, 0, 0, 0])


def check_sensor_replies():
    """SENSOR_INFO's parser against frames no peer here can produce.

    The two peers this check drives are both boards that report no sensors, so
    neither can produce a fitted body or a body of the wrong length - and those
    are exactly the cases where a client's mistake is invisible. A body read
    from the wrong offset is a plausible number; a body read past its end is the
    next reply's bytes. Frames written by hand are the only way to reach either,
    and a payload produced by the code under test would agree with it about a
    wrong offset anyway.
    """
    def body(topic, **fields):
        """A body of the documented length for `topic`, with `fields` filled in
        by byte offset. The rest is a pattern, so a field written twice or
        skipped moves everything after it."""
        length = SENSOR_BODY_LENGTH[topic]
        raw = bytearray((0x40 + i) & 0xFF for i in range(length))
        for at, value in fields.items():
            raw[int(at.lstrip("_")):int(at.lstrip("_")) + len(value)] = value
        return bytes(raw)

    def reply(payload):
        return parse_sensor_info(payload)

    def refusal(name, payload, wanted):
        """Refused, and refused in words that name the problem.

        The substring matters as much as the refusal. A check that only asserted
        "it raised" would pass for a client that reported every malformed reply
        the same way - and the one it must not be confused with is the first:
        0x7F is a board saying it does not have the command, which is a fact
        about the firmware, and "status 127" is a fact about nothing."""
        try:
            parse_sensor_info(payload)
        except IOError as problem:
            expect("%s (%s)" % (name, wanted), wanted in str(problem))
        except (struct.error, IndexError):
            expect("%s (as a struct error rather than a sentence)" % name, False)
        else:
            expect(name, False)

    refusal("a board answering 0x7F is refused rather than read as 'no such "
            "topic'", bytes([0x7F, 0, 0]), "predates `sensor info`")
    refusal("a reply too short to carry its own status is refused",
            bytes([SENSOR_OK, 0]), "on every path")
    refusal("a refused topic with a body behind it is refused",
            bytes([SENSOR_NO_SUCH, SENSOR_BARO, 0]) + bytes(52), "three bytes")
    refusal("an absent sensor with a body behind it is refused - that is the "
            "distinction the present byte exists for",
            bytes([SENSOR_OK, SENSOR_BARO, 0]) + bytes(52), "reading zero")
    refusal("a body longer than its topic's is refused",
            bytes([SENSOR_OK, SENSOR_IMU, 1]) + bytes(47), "would take the next "
            "field's bytes")
    refusal("a body shorter than its topic's is refused",
            bytes([SENSOR_OK, SENSOR_GPS, 1]) + bytes(55), "would take the next "
            "field's bytes")
    refusal("and a status outside the pair is refused",
            bytes([9, SENSOR_IMU, 0]), "status 9")

    # The three answers that look alike on a screen and are not. All three end
    # up as "nothing to show" in a naive client, and only the first is a fact
    # about the firmware.
    no_such = reply(bytes([SENSOR_NO_SUCH, SENSOR_RANGE, 0]))
    absent = reply(bytes([SENSOR_OK, SENSOR_RANGE, 0]))
    expect("a topic this build does not have and a sensor this board has not "
           "fitted are different answers",
           no_such["status"] != absent["status"] and
           no_such["topic"] == absent["topic"] and
           no_such["body"] is None and absent["body"] is None)

    # Present, and every field read at its documented offset. Two topics, with
    # distinct values throughout: a field read from its neighbour's bytes fails
    # here rather than coinciding.
    imu = reply(bytes([SENSOR_OK, SENSOR_IMU, 1]) + body(
        SENSOR_IMU,
        _0=b"bmi270\x00\x00\x00\x00\x00\x00", _12=bytes([1, 0xAB]),
        _14=struct.pack("<3h", 1, 2, 3), _20=struct.pack("<3h", 4, 5, 6),
        _26=struct.pack("<3h", 7, 8, 9), _32=struct.pack("<3h", 10, 11, 12),
        _38=struct.pack("<I", 4000), _42=struct.pack("<I", 7)))
    expect("an imu body reads every field at the offset the wire puts it at",
           imu["present"] == 1 and imu["body"]["driver"] == "bmi270" and
           imu["body"]["absent"] == 1 and imu["body"]["absent_reason"].startswith(
               "nothing answered") and
           imu["body"]["whoami"] == 0xAB and
           imu["body"]["accel"] == [1, 2, 3] and
           imu["body"]["gyro"] == [4, 5, 6] and
           imu["body"]["align"] == [7, 8, 9] and
           imu["body"]["gyro_bias"] == [10, 11, 12] and
           imu["body"]["samples"] == 4000 and imu["body"]["errors"] == 7)

    # The barometer carries the one field whose absence is a *state* rather than
    # a missing sensor: the reference pressure is captured on the ground, and a
    # height above it means nothing until it is. Both the flag and the number
    # are here so a client can say "not captured yet" rather than "0 m".
    baro = reply(bytes([SENSOR_OK, SENSOR_BARO, 1]) + body(
        SENSOR_BARO, _0=b"bmp388\x00\x00\x00\x00\x00\x00",
        _12=struct.pack("<i", 101325), _16=struct.pack("<h", 2150),
        _18=bytes([1]), _19=struct.pack("<i", 101000),
        _23=struct.pack("<i", 2750), _27=bytes([0]),
        _28=struct.pack("<i", 2680),
        _32=struct.pack("<I", 900), _36=struct.pack("<I", 3),
        _40=struct.pack("<I", 0), _44=struct.pack("<I", 880),
        _48=struct.pack("<I", 120)))
    expect("a baro body reads its pressure, its temperature and its heights",
           baro["body"]["driver"] == "bmp388" and
           baro["body"]["pressure_pa"] == 101325 and
           abs(baro["body"]["temperature_c"] - 21.5) < 1e-9 and
           baro["body"]["have_reference"] == 1 and
           baro["body"]["reference_pa"] == 101000 and
           baro["body"]["height_cm"] == 2750 and
           baro["body"]["have_gps_reference"] == 0 and
           baro["body"]["fused_cm"] == 2680 and
           baro["body"]["baro_samples"] == 880 and
           baro["body"]["gps_samples"] == 120)

    # The rangefinder's distance is signed because "nothing in range" is a
    # negative number, not a zero - zero is a wall against the lens.
    rng = reply(bytes([SENSOR_OK, SENSOR_RANGE, 1]) + body(
        SENSOR_RANGE, _0=b"tof10120\x00\x00\x00\x00", _12=bytes([0x52]),
        _13=struct.pack("<H", 2000), _15=struct.pack("<i", -1),
        _19=struct.pack("<I", 42), _23=struct.pack("<I", 100),
        _27=struct.pack("<I", 4), _31=struct.pack("<I", 2),
        _35=struct.pack("<I", 1), _39=struct.pack("<I", 0),
        _43=struct.pack("<I", 400), _47=struct.pack("<H", 300)))
    expect("a range body keeps 'nothing in range' negative rather than "
           "clamping it to a wall at the lens",
           rng["body"]["distance_mm"] == -1 and rng["body"]["address"] == 0x52 and
           rng["body"]["max_mm"] == 2000 and rng["body"]["age_ms"] == 42 and
           (rng["body"]["samples"], rng["body"]["out_of_range"],
            rng["body"]["rejected"], rng["body"]["faults"]) == (100, 4, 2, 1) and
           rng["body"]["land_mm"] == 400 and rng["body"]["agree_cm"] == 300)

    # The pin voltage is signed too: -1 is "the board had no reading", which is
    # a different thing from 0 mV at the pin.
    bat = reply(bytes([SENSOR_OK, SENSOR_BATTERY, 1]) + body(
        SENSOR_BATTERY, _0=bytes([1, 1, 2, 3]),
        _4=struct.pack("<H", 1180), _6=struct.pack("<H", 393),
        _8=struct.pack("<h", -1), _10=struct.pack("<H", 11000),
        _12=bytes([0]), _13=struct.pack("<H", 3500),
        _15=struct.pack("<H", 3300), _17=struct.pack("<I", 500),
        _21=struct.pack("<I", 3), _25=struct.pack("<I", 1)))
    expect("a battery body reads its pack in volts and its thresholds in "
           "millivolts, and keeps 'no reading at the pin' negative",
           bat["body"]["ready"] == 1 and bat["body"]["have_reading"] == 1 and
           bat["body"]["state_name"] == "warn" and bat["body"]["cells"] == 3 and
           abs(bat["body"]["volts"] - 11.8) < 1e-9 and
           abs(bat["body"]["volts_per_cell"] - 3.93) < 1e-9 and
           bat["body"]["pin_mv"] == -1 and
           abs(bat["body"]["ratio"] - 11.0) < 1e-9 and
           abs(bat["body"]["warn_cell_v"] - 3.5) < 1e-9 and
           abs(bat["body"]["critical_cell_v"] - 3.3) < 1e-9 and
           (bat["body"]["samples"], bat["body"]["rejected"],
            bat["body"]["returns"]) == (500, 3, 1))

    gps = reply(bytes([SENSOR_OK, SENSOR_GPS, 1]) + body(
        SENSOR_GPS, _0=bytes([1, 3, 1, 11, 1]),
        _5=struct.pack("<i", 521234567), _9=struct.pack("<i", -1224194300),
        _13=struct.pack("<i", 12345),
        _17=struct.pack("<i", 4200), _21=struct.pack("<i", 9000000),
        _25=bytes([1]), _26=struct.pack("<i", 521000000),
        _30=struct.pack("<i", -1224000000), _34=struct.pack("<i", 512),
        _38=struct.pack("<i", 9000), _42=bytes([0, 1]),
        _44=struct.pack("<I", 88), _48=struct.pack("<I", 2),
        _52=struct.pack("<I", 3)))
    expect("a gps body reads its fix, its position and its way home",
           gps["body"]["fix_type"] == 3 and gps["body"]["satellites"] == 11 and
           gps["body"]["valid_now"] == 1 and
           gps["body"]["lat_e7"] == 521234567 and
           gps["body"]["lon_e7"] == -1224194300 and
           gps["body"]["alt_msl_mm"] == 12345 and
           gps["body"]["speed_mm_s"] == 4200 and
           gps["body"]["have_home"] == 1 and
           gps["body"]["home_distance_m"] == 512 and
           gps["body"]["home_bearing_cdeg"] == 9000 and
           gps["body"]["returning"] == 0 and gps["body"]["rth_enabled"] == 1 and
           (gps["body"]["fixes"], gps["body"]["dropped"],
            gps["body"]["config_sends"]) == (88, 2, 3))

    # The driver name is a fixed field, so a name that fills it and one that
    # does not produce the same length. That is what lets the body's numbers be
    # read without measuring the string first.
    short = reply(bytes([SENSOR_OK, SENSOR_IMU, 1]) +
                  body(SENSOR_IMU, _0=b"ab\x00" + bytes(9)))
    full = reply(bytes([SENSOR_OK, SENSOR_IMU, 1]) +
                 body(SENSOR_IMU, _0=b"lsm6dsotwo\x00"))
    expect("a short driver name and a full-length one are the same reply "
           "length, so the numbers after the name do not move",
           short["body"]["driver"] == "ab" and
           full["body"]["driver"] == "lsm6dsotwo")


def check_reply_shapes():
    """The receiver parser against frames no peer here can produce.

    Both boards this check drives answer `rc channels` correctly - the stand-in
    with the one-byte "no receiver" status, the whole firmware with a live
    frame - so neither can exercise what a *broken* reply must do. Those are the
    cases where a client's mistake is invisible: a reply that says eight
    channels and carries seven, or a status byte that is not a status at all.
    A frame written by hand is the only way to reach them, and it is the right
    way for a second reason - a payload produced by the code under test would
    agree with it about a wrong offset.
    """
    def frame(status, flags=0, protocol=0, channels=(), sticks=(0, 0, 0, 0),
              switches=0, counters=(0,) * 7):
        body = bytes([status, flags, protocol, len(channels)])
        for value in channels:
            body += struct.pack("<H", value)
        for value in sticks:
            body += struct.pack("<h", value)
        body += bytes([switches])
        for value in counters:
            body += struct.pack("<I", value)
        return body

    def refusal(name, payload, wanted):
        """Refused, and refused *in words that name the problem*.

        The substring matters as much as the refusal. Every malformed reply here
        raises something, so a check that only asserted "it raised" would pass
        for a client that reported all four as the same generic status - and the
        one it must not be confused with is the first: `0x7F` is a board saying
        it does not have the command, which is a fact about the firmware, and
        "status 127" is a fact about nothing a person can act on.
        """
        try:
            parse_rc_channels(payload)
        except IOError as problem:
            expect("%s (%s)" % (name, wanted), wanted in str(problem))
        except (struct.error, IndexError):
            # The other way a short frame shows up. Still a refusal of the frame
            # rather than of the aircraft, which is the property being checked -
            # but a worse message, so it is worth knowing which one happened.
            expect("%s (as a struct error rather than a sentence)" % name, False)
        else:
            expect(name, False)

    # The fourth answer, which is not a receiver fact at all: 0x7F is what this
    # firmware replies to a command it does not implement. It is not a member of
    # the status enum, and reporting it as `RC_NONE` would turn "I do not have
    # that command" into a statement about a board that may have a receiver
    # plugged into it at this moment.
    refusal("a board answering 0x7F is refused rather than read as 'no receiver'",
            bytes([0x7F]), "predates `rc channels`")

    # "One byte and no more" is the firmware's rule; a client that read past it
    # would be parsing a stream of bytes it had decided means nothing.
    refusal("a 'no receiver' status with a body behind it is refused",
            bytes([RC_NONE, 0, 0, 8]), "answers one byte")

    # Reading on would take the sticks for channels and the four stick values
    # for counters - a frame of the right shape and the wrong meaning.
    refusal("a channel count past what the protocol allows is refused",
            frame(0, channels=[992] * (RC_MAX_CHANNELS + 1)), "at most 16")

    # The case the firmware's clamp exists to make impossible, and the one a
    # client must still refuse: eight channels promised, one delivered.
    refusal("a reply shorter than its own count implies is refused",
            frame(0, channels=[992])[:-1], "one is")

    # And the offsets, on a frame with seven distinct counters: a field read
    # from its neighbour's bytes is a failing comparison rather than a
    # coincidence.
    rc = parse_rc_channels(frame(
        0, flags=(1 << 0) | (1 << 2) | (1 << 4), protocol=1,
        channels=[172, 992, 1811], sticks=(0, -1000, 250, 1000),
        switches=(1 << 1), counters=(1, 2, 3, 4, 5, 6, 7)))
    expect("a hand-built frame reads every field at the offset the wire puts it "
           "at",
           rc["channels"] == [172, 992, 1811] and
           rc["sticks"] == [0, -1000, 250, 1000] and
           rc["protocol_name"] == "SBUS" and rc["switch_names"] == ["angle"] and
           (rc["bytes"], rc["frames"], rc["crc_errors"], rc["rejected"]) ==
           (1, 2, 3, 4) and
           (rc["lost"], rc["failsafe_frames"], rc["dropped"]) == (5, 6, 7))
    # Signed, and -1000 is the value that says so: written as a magnitude it
    # would survive as +1000, which is full stick the other way.
    expect("and the sticks are signed rather than magnitudes",
           rc["sticks"][1] == -1000)
    # A protocol number this client has never heard of keeps its number and gets
    # no name - "the board says 9" is a fact, and a neighbour's name is not.
    unknown = parse_rc_channels(frame(0, protocol=9, channels=[1]))
    expect("and a protocol this client cannot name is reported by its number",
           unknown["protocol_name"] is None and unknown["protocol"] == 9)


# The simulator half of this check, built by `make host`. Its location moves
# when HOST_OUT moves - which the build harness that drives this repository
# does, so that a target's objects stay in the target's build directory. It is
# therefore passed in rather than assumed; the default is where a plain
# `make host` puts it, so the script still runs by hand.
#
# This was a real hole rather than tidiness. With the path hard-coded, a
# harness build that set HOST_OUT elsewhere ran this check against whatever
# binary happened to be in build-host - built by some earlier, unrelated
# command. The check passed, and it was not checking the build under test.
SIM = os.environ.get("AK_SIM", "build-host/aerialkit-sim")
# And how to start it: the protocol simulator takes no arguments (it *is* the
# protocol's other end), where the whole firmware in a console session needs
# `0 console`. Empty by default, so nothing about the simulator changes.
SIM_ARGS = os.environ.get("AK_SIM_ARGS", "").split()

# The product the firmware under test was built as, which the Makefile passes
# along with the simulator's path. It is an environment variable rather than a
# constant here because a check that names one target's product is a check that
# fails the moment a second target is built - which is what happened when the
# third one (an AT32F435) arrived and these tests ran against an image that
# correctly called itself something else.
PRODUCT_UNDER_TEST = os.environ.get("AK_PRODUCT", "")

failures = 0


def expect(name, condition):
    global failures
    if condition:
        print("  ok       %s" % name)
    else:
        print("  FAILED   %s" % name)
        failures += 1


def main():
    process = subprocess.Popen([SIM] + SIM_ARGS, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL)

    def write(data):
        process.stdin.write(data)
        process.stdin.flush()

    client = Client(process.stdout.read, write)

    print("client against the firmware's own protocol code")

    hello = client.request(HELLO)
    product = c_string(hello, 1)
    count = int.from_bytes(hello[2 + len(product):4 + len(product)], "little")
    expect("hello names the product the build was made as",
           product == PRODUCT_UNDER_TEST)
    # The count is written down rather than read from the firmware, because a
    # table that grew or shrank without anybody noticing is exactly what this is
    # here to catch. It moved from 27 to 28 with `arm_max_tilt_deg`, from 28 to
    # 31 with the fixed wing's lost-link descent, to 32 with the gyro's low
    # pass, and to 33 with the arming gate's accelerometer low pass - and each
    # time the number is *here* rather than read from `AK_FLIGHT_PARAM_COUNT` on
    # purpose: a check that took the count from the firmware could not notice the
    # firmware changing it.
    #
    # The 33rd is the one that shows why the number is worth writing down. The
    # gate-filter patch added `arm_accel_lpf_hz` to the table and did not move
    # this line, and `make test` could not have said so: the parameter table is
    # reached over the protocol, and `proto-test` is a different stage. It was
    # green everywhere the patch was measured and red the first time the stage
    # that owns this number ran.
    expect("and reports the parameter table", count == 33)

    # ---- the capability word, on the stand-in -----------------------------
    #
    # `akproto_firmware_check.py` makes the same reading against the whole
    # firmware, where it is the interesting one — it is the check that main.c's
    # `proto_io.features` initialiser tells the truth. This one is about
    # `akproto_sim.c`, whose initialiser is a *different* claim and has to be a
    # different claim: this binary answers `param info` and `param help` because
    # ak_proto.c has the cases, and it has no `on_change` and nothing to
    # re-apply, so it must not claim APPLIES_ON_WRITE. The failure this guards
    # against is under-reporting, which is the same class of lie as
    # over-reporting and is the one a hand-started initialiser makes: every
    # field left out of a designated initialiser is zero, so a sim that grew a
    # `case` and did not grow a bit would look to every client like a board from
    # before the command existed.
    #
    # The two lines are not redundant, and the falsification says so: with the
    # initialiser's `.features` line deleted, the first still passes — the reply
    # carries eight bytes of word either way, and zero is a word — and only the
    # second fails. That is the protocol's own distinction (present-and-zero is
    # not absent) showing up in the check, and it is why the value line is the
    # one that matters.
    parsed = parse_hello(hello)
    expect("the stand-in carries a capability word at all",
           parsed["features"] is not None)
    expect("and it is exactly the one this binary can keep "
           "(PARAM_INFO, and not APPLIES_ON_WRITE)",
           parsed["features"] == 1 << 0)

    first = client.request(PARAM_GET, bytes([0]))
    name = c_string(first, 1)
    value = c_string(first, 2 + len(name))
    expect("a parameter comes back with a name and a value",
           len(name) > 0 and len(value) > 0)

    beyond = client.request(PARAM_GET, bytes([200]))
    expect("asking past the end is refused, not answered with noise", beyond[0] == 1)

    accepted = client.request(PARAM_SET, bytes([0]) + b"0.5")
    expect("a value in range is accepted", accepted[0] == 0)
    read_back = client.request(PARAM_GET, bytes([0]))
    read_name = c_string(read_back, 1)
    expect("and reads back as what was set",
           c_string(read_back, 2 + len(read_name)) == "0.500")

    refused = client.request(PARAM_SET, bytes([0]) + b"99")
    refused_status, refused_message = set_reply(refused)
    expect("a value out of range is refused", refused_status == 2)
    expect("and the refusal carries the board's own reason for it",
           "out of" in refused_message)

    saved = client.request(PARAM_SAVE)
    expect("save reports success", saved[0] == 0)

    status = parse_status(client.request(STATUS))
    expect("status carries the flight state and the link",
           status["flight_state"] == 1 and status["link_live"] == 1)
    expect("the attitude comes back signed",
           abs(status["roll_deg"] - 12.5) < 0.01 and abs(status["pitch_deg"] + 4.0) < 0.01)
    expect("the position round trips",
           abs(status["lat"] - 52.1234567) < 1e-6 and abs(status["lon"] - 4.9876543) < 1e-6)
    expect("and the motor outputs are there", status["motors"] == [40, 40, 60, 60])

    info = client.request(LOG_INFO)
    count = int.from_bytes(info[0:2], "little")
    expect("the blackbox reports how many records it holds", count == 6)

    rows = []
    fetch_log(client, rows.append)
    # One write for the header and one per record, which is what a client
    # streaming to a file does.
    expect("and the whole log comes back", len(rows) == 7)
    expect("with the console's own header",
           rows[0].startswith("# aerialkit blackbox (fast)") and
           "time_ms,gyro_x" in rows[0])
    expect("and the first record on it", rows[1].startswith("1000,100,"))
    expect("and the last one", rows[6].startswith("1020,105,"))

    # And the whole row, not just its first two columns: the client decodes the
    # record's fields itself, so an offset that is wrong in the *client* is a
    # CSV that looks right and is not - which is a bug this file can catch and
    # nothing else can. The record's yaw and height are the last two fields the
    # firmware added, and they are the ones a bad landing is read from.
    first = rows[1].split(",")
    last = rows[6].split(",")
    expect("the row has a column per field the record has",
           len(first) == len(LOG_FIELDS))
    expect("and the yaw and the height decode where the layout says",
           first[9] == "-900" and first[10] == "23000" and
           last[9] == "-905" and last[10] == "23050")
    # ... and the position, which is the field *after* those: the columns a
    # tool needs to say where the aircraft was, and the ones where a client
    # that forgot the layout grew after the state and the flags writes a
    # column of zeroes that look exactly like a flight with no fix.
    expect("and the position decodes where the layout says",
           first[len(LOG_FIELDS) - 2] == "521234500" and
           first[len(LOG_FIELDS) - 1].strip() == "-49876500" and
           last[len(LOG_FIELDS) - 2] == "521234505" and
           last[len(LOG_FIELDS) - 1].strip() == "-49876495")

    # The log in flash, over the same two commands: this is the whole path a
    # tool uses to pull the log that survived the crash - source, count, and
    # records by index - against the same code the board runs.
    flash_rows = []
    expect("the log in flash comes back over the protocol",
           fetch_log(client, flash_rows.append, LOG_SOURCES["flash"]) == 3)
    expect("with its own header and the records in order",
           flash_rows[0].startswith("# aerialkit blackbox (flash)") and
           flash_rows[1].startswith("2000,0,200,") and
           flash_rows[3].startswith("2010,0,202,"))

    # And a log this device does not have is refused rather than reported
    # empty, which is what the status byte in the reply is for.
    expect("a log this device does not have is refused, not reported empty",
           select_log(client, LOG_SOURCES["long"]) is None)
    expect("and the source it had is still selected afterwards",
           fetch_log(client, rows.append, LOG_SOURCES["fast"]) == 6)

    check_captured_telemetry()
    check_reply_shapes()

    # The other half of the receiver, and the reason it is checked here rather
    # than against the whole firmware: this stand-in has no `rc_state` callback,
    # so it is a board with no receiver port - exactly the board the one-byte
    # `RC_NONE` answer is for. The firmware's own check drives the opposite
    # case, a board with a receiver framing on it, because that is what
    # `aerialkit-fw-sim` has and this does not. Neither half can produce the
    # other's answer, and a client that conflated them would pass one and fail
    # the other.
    rc = parse_rc_channels(client.request(RC_CHANNELS))
    expect("a board with no receiver port answers the one-byte 'none' status",
           rc["status"] == RC_NONE)
    # Not zeros. Every field is empty, and `frames` is 0 because nothing was
    # counted rather than because the count is zero - the difference a client
    # that drew a bar chart would show as eight bars at rest.
    expect("and says nothing else about a receiver it does not have",
           rc["channels"] == [] and rc["sticks"] == [] and
           rc["protocol"] is None and rc["count"] == 0)
    # Worth noting rather than asserting, because the stand-in may legitimately
    # grow the bit one day: this stand-in does *not* claim RC_CHANNELS in its
    # capability word, and it answers the command anyway. A bit and a `case` are
    # separate things - the bit is advisory and the reply is the answer - which
    # is why the client reads the status byte rather than the feature word to
    # decide what it is looking at.

    check_sensor_replies()

    # The same split as the receiver, one level up, and the reason it is worth
    # driving both halves: this stand-in has no `sensor_state` callback, so it is
    # a board with no sensor reporting at all - the case the null callback is
    # reserved for - and every topic must come back as a fact about the *build*.
    # `akproto_firmware_check.py` drives the opposite case against a board that
    # has its sensors, where the answer is a fact about the aircraft. A client
    # that showed NO_SUCH and "not fitted" the same way would pass one and fail
    # the other, which is exactly the mistake that sends somebody looking for a
    # driver when the part is not soldered in.
    for topic in range(SENSOR_TOPICS):
        answer = sensor_info(client, topic)
        expect("a board with no sensor reporting refuses topic %u as a fact "
               "about the build, not about a missing part" % topic,
               answer["status"] == SENSOR_NO_SUCH and answer["present"] == 0 and
               answer["body"] is None and answer["topic"] == topic)
    expect("and refuses a topic outside the five rather than answering about "
           "the nearest one",
           sensor_info(client, SENSOR_TOPICS)["status"] == SENSOR_NO_SUCH and
           sensor_info(client, 200)["status"] == SENSOR_NO_SUCH)
    # A request that named no topic at all. The reply is the same refusal, and
    # it must be: a frame cut short by a UART is not a question about the IMU.
    expect("and refuses a request that named no topic",
           parse_sensor_info(client.request(SENSOR_INFO, b""))["status"] ==
           SENSOR_NO_SUCH)

    process.stdin.close()
    process.wait(timeout=5)

    print("proto end to end: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
