#!/usr/bin/env python3
"""The cross-repository contract, checked by an implementation that shares no
code with either side that produced it.

    make contract-check

`contract/aerialkit-contract-v1.json` holds values from two producers:

  * this firmware's own encoders, run by tools/contract_vectors.c, for the
    attitude conversions, the telemetry frame and the blackbox record;
  * the other repository's own `frame_transforms.py`, for the NED->ENU
    rotation, which this firmware cannot state because it has no ENU in it.

A file produced by both sides and read by neither is not evidence of anything.
So this is the third implementation: every number below is recomputed here -
the CRC, the little-endian packing, the field order, the float32 rounding of
the angle conversion, the quaternion algebra - from the layout written down in
docs/16-protocol.md and docs/31-contract.md, and compared against what the two
producers said. A disagreement is a real cross-repository bug, not a typo in a
test.

**It checks the contract, not the code.** Where the two differ, this file is
right and something else is wrong, and that is the point of writing it down in
two languages before writing it down in one.

**Float32 is modelled deliberately.** `ak_attitude_ddeg()` is C `float`
arithmetic and its answers near +/-180 degrees depend on that - see the vector
`{"what": "just past pi"}`, which wraps one ULP after the angle the firmware
calls pi. A checker written in float64 would disagree with the firmware about
which side of the wrap a heading is on, and would be wrong to.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VECTORS = os.path.join(ROOT, "contract", "aerialkit-contract-v1.json")

checks = 0
failures = 0
# Set while the self-test runs its mutations: the failures it provokes are the
# ones it is asking for, and printing them beside the real output reads as a
# broken contract rather than as a working check.
quiet = False

# ---------------------------------------------------------------------------
# The layout, from docs/16-protocol.md and docs/31-contract.md.
# ---------------------------------------------------------------------------

SYNC1 = 0xAA
SYNC2 = 0x55
PROTOCOL_VERSION = 1
CMD_TELEMETRY = 0x08
HEADER_BYTES = 5
CRC_BYTES = 2

# u8 state, u8 link, u8 fix, u8 sats, i16 roll, i16 pitch, i16 yaw,
# i32 lat, i32 lon, u8 motor[4]
STATUS_BODY_BYTES = 22
# the streamed frame carries the uptime before the body
TELEMETRY_PREFIX_BYTES = 4

# u32 time_ms, i16 gyro[3], i16 accel[3], i16 attitude[2], i16 yaw, i32 alt_mm,
# i16 stick[4], i8 torque[3], u8 motor[4], u8 state, u8 flags,
# i32 lat_e7, i32 lon_e7                                        = 51 bytes
# i16 gyro_filtered[3], u16 notch_hz[3], u8 notch_engaged[3]     = 15 more
# u32 time_us, i16 rate_setpoint[3], i8 pid_p[3], i8 pid_i[3], i8 pid_d[3],
# u16 vbat_mv                                                    = 21 more
#
# The first 51 are version 2's and the next 15 version 3's, unmoved, and the
# checks below assert both: a client built before roadmap 2.4, or before 4.1,
# decodes a correct prefix rather than a record with every field after the
# shift misread.
LOG_WIRE_BYTES = 87
LOG_V2_WIRE_BYTES = 51
LOG_V3_WIRE_BYTES = 66
# The struct's internal padding - the two bytes before alt_mm and the three
# before lat_e7 - which the wire does not carry. Named because several checks
# below are about the difference between the two layouts.
LOG_STRUCT_PADDING = 5


def expect(name, condition, detail=""):
    global checks, failures
    checks += 1
    if not condition:
        failures += 1
        if not quiet:
            print("  FAIL  %s%s" % (name, ("  " + detail) if detail else ""))
    return bool(condition)


# ---------------------------------------------------------------------------
# The arithmetic, written from the definitions rather than imported.
# ---------------------------------------------------------------------------

def f32(value):
    """The float32 nearest to `value`, as C's `float` would hold it."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


PI = f32(3.14159265358979)      # AK_PI, spelled as the firmware spells it
TURN = f32(6.28318531)
WRAP_LIMIT = f32(1.0e5)
DEG_PER_RAD = f32(f32(180.0) / PI)


def crc16(data):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def wrap_pi(radians):
    """Reduce to (-pi, pi], in float32, with the same guard the firmware has."""
    r = f32(radians)
    # Both comparisons are false for a NaN, which is what catches it.
    if not (r <= WRAP_LIMIT and r >= -WRAP_LIMIT):
        return f32(0.0)
    whole = int(f32(r / TURN))               # truncation toward zero
    r = f32(r - f32(f32(float(whole)) * TURN))
    if r > PI:
        r = f32(r - TURN)
    elif r < -PI:
        r = f32(r + TURN)
    return r


def ddeg(radians):
    """Tenths of a degree, in an int16 - the protocol's and the log's unit."""
    value = f32(f32(wrap_pi(radians)) * DEG_PER_RAD)
    return int(f32(value * f32(10.0)))


def mrad(radians):
    """The console's unit. No guard: undefined for a NaN or an infinity."""
    return int(f32(f32(radians) * f32(1000.0)))


def bits_of(value):
    return "0x%08x" % struct.unpack("<I", struct.pack("<f", value))[0]


def u8(value):
    return struct.pack("<B", value & 0xFF)


def i16(value):
    return struct.pack("<h", value)


def u16(value):
    return struct.pack("<H", value & 0xFFFF)


def i32(value):
    return struct.pack("<i", value)


def i8(value):
    return struct.pack("<b", value)


# ---------------------------------------------------------------------------
# Quaternions, w-first, as both repositories carry them.
# ---------------------------------------------------------------------------

def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_from_euler_zyx(roll, pitch, yaw):
    """Body-FRD -> NED, w-first, ZYX - the order the firmware's estimator uses."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr)


def close(a, b, tol=1e-12, allow_sign=True):
    """Quaternions are equal up to a sign: q and -q are the same rotation."""
    if allow_sign and _all_negated(a, b, tol):
        return True
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _all_negated(a, b, tol):
    return all(abs(x + y) <= tol for x, y in zip(a, b))


def unit(q):
    return abs(sum(v * v for v in q) - 1.0) <= 1e-12


# ---------------------------------------------------------------------------
# Group 1: the angles.
# ---------------------------------------------------------------------------

def check_attitudes(document):
    cases = document["attitude"]["cases"]
    expect("the attitude group has cases", len(cases) >= 15, "(%d)" % len(cases))

    undefined_mrad = 0
    for case in cases:
        what = case["what"]
        raw = int(case["bits"], 16)
        value = struct.unpack("<f", struct.pack("<I", raw))[0]

        # The stored decimal must be the stored bits, or a reader trusting the
        # decimal would be reading a different number than the one checked.
        if "radians" in case:
            expect("%s: the decimal is the bits" % what,
                   abs(f32(case["radians"]) - value) == 0.0
                   or (math.isnan(value) and math.isnan(case["radians"])))

        expect("%s: ddeg" % what, ddeg(value) == case["ddeg"],
               "(contract says %d, firmware said %d)"
               % (ddeg(value), case["ddeg"]))

        if math.isfinite(value):
            expect("%s: mrad" % what, mrad(value) == case["mrad"],
                   "(contract says %d, firmware said %d)"
                   % (mrad(value), case["mrad"]))
        else:
            # The console multiplies by 1000 and casts to int with no guard,
            # which the C standard leaves undefined for a NaN. The firmware
            # printed INT_MIN; that is this compiler's answer to an undefined
            # question, not a value the contract can promise. Counted rather
            # than silently passed, so the gap stays visible.
            undefined_mrad += 1

    expect("three undefined console-mrad cases were expected", undefined_mrad == 3,
           "(found %d)" % undefined_mrad)

    by_name = {case["what"]: case for case in cases}
    # The two claims that no reader would guess, and that the group exists for.
    expect("pi encodes as 1800, the far edge of the field",
           by_name["pi"]["ddeg"] == 1800, "(%d)" % by_name["pi"]["ddeg"])
    expect("minus pi encodes as -1800",
           by_name["minus pi"]["ddeg"] == -1800)
    expect("one float32 past pi wraps to -1799",
           by_name["just past pi"]["ddeg"] == -1799,
           "(%d)" % by_name["just past pi"]["ddeg"])
    expect("the wrap boundary is exactly one ULP wide",
           by_name["just inside pi"]["ddeg"] == 1799
           and by_name["just past pi"]["ddeg"] == -1799)
    expect("past the wrap limit the firmware answers zero",
           by_name["past the limit"]["ddeg"] == 0)
    expect("a whole number of turns is zero",
           by_name["five turns"]["ddeg"] == 0)
    # Truncation toward zero, not rounding: a heading a hair below 360 degrees
    # encodes as 0 rather than -1, which is a whole degree of lost resolution
    # and the reason the field is not a compass bearing.
    expect("359.9 degrees truncates to 0, not -1",
           by_name["359.9 degrees"]["ddeg"] == 0,
           "(%d)" % by_name["359.9 degrees"]["ddeg"])


# ---------------------------------------------------------------------------
# Group 2: the telemetry frame.
# ---------------------------------------------------------------------------

def encode_status_body(status):
    return b"".join([
        u8(status["flight_state"]),
        u8(status["link_live"]),
        u8(status["gps_fix_type"]),
        u8(status["gps_satellites"]),
        i16(status["roll_ddeg"]),
        i16(status["pitch_ddeg"]),
        i16(status["yaw_ddeg"]),
        i32(status["lat_e7"]),
        i32(status["lon_e7"]),
        b"".join(u8(m) for m in status["motor"]),
    ])


def encode_header(command, payload):
    frame = bytes([SYNC1, SYNC2, PROTOCOL_VERSION, command, len(payload)]) + payload
    crc = crc16(frame[2:])
    return frame + bytes([crc & 0xFF, crc >> 8])


def check_telemetry(document):
    group = document["telemetry"]
    status = group["status"]

    body = encode_status_body(status)
    expect("the status body is %d bytes" % STATUS_BODY_BYTES,
           len(body) == STATUS_BODY_BYTES, "(%d)" % len(body))

    payload = i32(group["now_ms"]) + body
    expect("the payload is the uptime and the body",
           len(payload) == group["payload_length"], "(%d)" % len(payload))

    frame = encode_header(CMD_TELEMETRY, payload)
    expect("the frame length is header, payload and crc",
           len(frame) == group["frame_length"], "(%d)" % len(frame))
    expect("the frame matches the firmware byte for byte",
           frame.hex() == group["frame_hex"],
           "\n        contract: %s\n        firmware: %s"
           % (frame.hex(), group["frame_hex"]))

    # The parts of that frame, named, so a mismatch says which field moved
    # rather than only that a hex string differs.
    raw = bytes.fromhex(group["frame_hex"])
    expect("the sync pair is AA 55", raw[0] == SYNC1 and raw[1] == SYNC2)
    expect("the version is the protocol's", raw[2] == PROTOCOL_VERSION)
    expect("a streamed frame carries no response bit",
           raw[3] == CMD_TELEMETRY and not raw[3] & 0x80, "(0x%02x)" % raw[3])
    expect("the length field is the payload", raw[4] == len(payload))
    expect("the crc is the last two bytes, little-endian",
           raw[-2] == (crc16(raw[2:-2]) & 0xFF)
           and raw[-1] == crc16(raw[2:-2]) >> 8)
    expect("255 means no output and stays 255",
           raw[HEADER_BYTES + TELEMETRY_PREFIX_BYTES + 18 + 2] == 255)


# ---------------------------------------------------------------------------
# Group 3: the blackbox record.
# ---------------------------------------------------------------------------

def encode_log_record_v2(record):
    """Version 2's record, which is version 3's first 51 bytes."""
    return b"".join([
        struct.pack("<I", record["time_ms"]),
        b"".join(i16(v) for v in record["gyro"]),
        b"".join(i16(v) for v in record["accel"]),
        b"".join(i16(v) for v in record["attitude"]),
        i16(record["yaw"]),
        i32(record["alt_mm"]),
        b"".join(i16(v) for v in record["stick"]),
        b"".join(i8(v) for v in record["torque"]),
        b"".join(u8(v) for v in record["motor"]),
        u8(record["state"]),
        u8(record["flags"]),
        i32(record["lat_e7"]),
        i32(record["lon_e7"]),
    ])


def encode_log_record_v3(record):
    return encode_log_record_v2(record) + b"".join([
        b"".join(i16(v) for v in record["gyro_filtered"]),
        b"".join(u16(v) for v in record["notch_hz"]),
        b"".join(u8(v) for v in record["notch_engaged"]),
    ])


def encode_log_record(record):
    """Version 4: version 3 plus the controller's columns (roadmap 4.1)."""
    return encode_log_record_v3(record) + b"".join([
        struct.pack("<I", record["time_us"]),
        b"".join(i16(v) for v in record["rate_setpoint"]),
        b"".join(i8(v) for v in record["pid_p"]),
        b"".join(i8(v) for v in record["pid_i"]),
        b"".join(i8(v) for v in record["pid_d"]),
        u16(record["vbat_mv"]),
    ])


def check_log_record(document):
    group = document["log_record"]
    record = group["record"]

    wire = encode_log_record(record)
    expect("the wire record is %d bytes" % LOG_WIRE_BYTES,
           len(wire) == LOG_WIRE_BYTES, "(%d)" % len(wire))
    expect("the encoder wrote %d bytes" % LOG_WIRE_BYTES,
           group["encoded_length"] == LOG_WIRE_BYTES)
    expect("the record matches the firmware byte for byte",
           wire.hex() == group["wire_hex"],
           "\n        contract: %s\n        firmware: %s"
           % (wire.hex(), group["wire_hex"]))

    # And the compatibility story, which is a claim about the bytes and so is
    # checked against the bytes: roadmap 2.4 appended the filtering fields, so
    # version 2's fifty-one bytes are still there, unmoved, at the front.
    v2 = encode_log_record_v2(record)
    expect("version 2's record is a prefix of version 3's",
           wire[:LOG_V2_WIRE_BYTES] == v2)
    expect("and the last of version 2's fields still ends where it did",
           v2[-4:] == struct.pack("<i", record["lon_e7"]))
    # And version 3's sixty-six, for the same reason: 4.1 appended too.
    v3 = encode_log_record_v3(record)
    expect("version 3's record is a prefix of version 4's",
           wire[:LOG_V3_WIRE_BYTES] == v3 and len(v3) == LOG_V3_WIRE_BYTES)

    # The struct is *not* the wire format, and a client that assumes it is
    # reads every field after the first misaligned one as noise. The firmware
    # states both numbers; this is the check that they stay different and that
    # the difference is the padding: the two gaps internal to version 2's
    # fields (five bytes), the byte that aligns version 4's `time_us` after
    # version 3's byte-sized tail, the byte that aligns `vbat_mv` after the
    # nine P/I/D bytes, and the two the struct's end is rounded up by to its
    # four-byte alignment. 5 + 1 + 1 + 2 = 9.
    expect("the C struct is larger than the wire record",
           group["sizeof_record"] > LOG_WIRE_BYTES,
           "(%d vs %d)" % (group["sizeof_record"], LOG_WIRE_BYTES))
    expect("the difference is padding, %d bytes"
           % (group["sizeof_record"] - LOG_WIRE_BYTES),
           group["sizeof_record"] - LOG_WIRE_BYTES == LOG_STRUCT_PADDING + 4)

    # The offsets are the struct's, and the wire has none of the gaps: every
    # offset after the first padded field is smaller on the wire. Checked by
    # asserting the two orders agree, which is what a decoder needs.
    offsets = group["offsets"]
    order = sorted(offsets, key=lambda name: offsets[name])
    expect("the struct's field order is the wire's field order",
           order == ["time_ms", "gyro", "accel", "attitude", "yaw", "alt_mm",
                     "stick", "torque", "motor", "state", "flags", "lat_e7",
                     "lon_e7", "gyro_filtered", "notch_hz", "notch_engaged",
                     "time_us", "rate_setpoint", "pid_p", "pid_i", "pid_d",
                     "vbat_mv"],
           "(%s)" % ", ".join(order))
    expect("alt_mm is padded in the struct but not on the wire",
           offsets["alt_mm"] == 24 and offsets["yaw"] + 2 == 22)
    expect("version 3's last field ends where version 3's wire did",
           offsets["notch_engaged"] + 3 == LOG_V3_WIRE_BYTES + LOG_STRUCT_PADDING)
    # The tail is appended, so the version-2 fields' struct offsets are
    # exactly what they were and the new trio starts after them. A field
    # inserted anywhere earlier would pass every check above and fail this one.
    expect("the appended fields start where version 2's struct ended",
           offsets["gyro_filtered"] == offsets["lon_e7"] + 4)
    expect("and the tail is packed with no padding of its own",
           offsets["notch_hz"] == offsets["gyro_filtered"] + 6
           and offsets["notch_engaged"] == offsets["notch_hz"] + 6)
    # Version 4's tail: appended after version 3's, its only gaps the two
    # alignment bytes counted above, so a field inserted inside it shows here.
    expect("version 4's fields start after version 3's, aligned",
           offsets["time_us"] == offsets["notch_engaged"] + 3 + 1)
    expect("and run in wire order with only vbat_mv's alignment byte between",
           offsets["rate_setpoint"] == offsets["time_us"] + 4
           and offsets["pid_p"] == offsets["rate_setpoint"] + 6
           and offsets["pid_i"] == offsets["pid_p"] + 3
           and offsets["pid_d"] == offsets["pid_i"] + 3
           and offsets["vbat_mv"] == offsets["pid_d"] + 3 + 1)


# ---------------------------------------------------------------------------
# Group 4: the NED -> ENU rotation, which is the other repository's to state.
# ---------------------------------------------------------------------------

def check_ned_enu(document):
    group = document["ned_enu"]
    ned_enu_q = tuple(group["ned_enu_q_wxyz"])
    baselink_q = tuple(group["baselink_q_wxyz"])

    expect("NED_ENU_Q is a unit quaternion", unit(ned_enu_q))
    expect("AIRCRAFT_BASELINK_Q is a unit quaternion", unit(baselink_q))
    expect("NED_ENU_Q is the documented 180 about (1,1,0)/sqrt2",
           close(ned_enu_q, (0.0, math.sqrt(2) / 2, math.sqrt(2) / 2, 0.0)))
    expect("AIRCRAFT_BASELINK_Q is the documented 180 about body x",
           close(baselink_q, (0.0, 1.0, 0.0, 0.0)))

    for case in group["attitudes"]:
        q_ned = tuple(case["q_ned_wxyz"])
        expect("attitude %.0f/%.0f/%.0f: the input is a rotation"
               % (case["roll_deg"], case["pitch_deg"], case["yaw_deg"]), unit(q_ned))

        # The composition, from the definition in C2.
        q_enu = qmul(qmul(ned_enu_q, q_ned), baselink_q)
        expect("attitude %.0f/%.0f/%.0f: q_enu"
               % (case["roll_deg"], case["pitch_deg"], case["yaw_deg"]),
               close(q_enu, tuple(case["q_enu_wxyz"])),
               "\n        contract: %s\n        other repo: %s"
               % (_fmt(q_enu), _fmt(case["q_enu_wxyz"])))
        expect("attitude %.0f/%.0f/%.0f: the ROS order is the same rotation"
               % (case["roll_deg"], case["pitch_deg"], case["yaw_deg"]),
               close(tuple(case["q_ros_xyzw"]),
                     (q_enu[1], q_enu[2], q_enu[3], q_enu[0])))

    # The frames differ only by a fixed rotation: converting a body-FRD->NED
    # attitude without the baselink term must give a *different* quaternion
    # wherever the attitude is not symmetric, or the term is doing nothing and
    # the two frames have been silently conflated.
    disagreed = 0
    for case in group["attitudes"]:
        q_ned = tuple(case["q_ned_wxyz"])
        without = qmul(ned_enu_q, q_ned)
        if not close(without, tuple(case["q_enu_wxyz"])):
            disagreed += 1
    expect("dropping the baselink term changes the answer",
           disagreed == len(group["attitudes"]),
           "(%d of %d)" % (disagreed, len(group["attitudes"])))

    check_the_yaw_origin(group)
    check_positions(group)


def check_the_yaw_origin(group):
    """The one disagreement between the two repositories, pinned.

    This firmware's yaw is measured from north, clockwise. The simulator's
    `heading_rad` is measured from +world-x, which in ENU is east, and
    counter-clockwise. They differ by a quarter turn, and a wiring that passes
    one into the other without converting flies ninety degrees off - so the
    identity that relates them is a contract term, not a remark.
    """
    worst = 0.0
    for case in group["level_attitudes"]:
        yaw = math.radians(case["yaw_deg"])
        q_ned = q_from_euler_zyx(0.0, 0.0, yaw)
        expect("yaw %.0f: the stated q_ned is that yaw"
               % case["yaw_deg"], close(q_ned, tuple(case["q_ned_wxyz"])))

        q_enu_frd = qmul(tuple(group["ned_enu_q_wxyz"]), q_ned)
        heading = math.pi / 2.0 - yaw
        expect("yaw %.0f: the simulator's heading is a quarter turn off"
               % case["yaw_deg"],
               abs(case["sim_heading_rad"] - heading) <= 1e-12)

        # The simulator's closed form for a level attitude, written here from
        # its own docstring: (0, cos(h/2), sin(h/2), 0).
        sim = (0.0, math.cos(heading / 2), math.sin(heading / 2), 0.0)
        expect("yaw %.0f: the two repositories agree on the attitude"
               % case["yaw_deg"], close(q_enu_frd, sim),
               "\n        firmware side: %s\n        simulator:     %s"
               % (_fmt(q_enu_frd), _fmt(sim)))
        expect("yaw %.0f: the stored simulator value is the same"
               % case["yaw_deg"],
               close(sim, tuple(case["sim_level_quat_wxyz"])))
        worst = max(worst, max(abs(a - b) for a, b in zip(q_enu_frd, sim)))

    expect("every level heading agrees to 1e-12", worst <= 1e-12,
           "(worst %.3g)" % worst)


def check_positions(group):
    for case in group["positions"]:
        north, east, down = case["ned"]
        expect("position %s maps to ENU" % (case["ned"],),
               list(case["enu"]) == [east, north, -down],
               "(%s)" % (case["enu"],))


def _fmt(q):
    return "(" + ", ".join("%.9f" % v for v in q) + ")"


# ---------------------------------------------------------------------------
# The check's own check.
# ---------------------------------------------------------------------------

def run_all(document):
    """Every real check, on a document the caller may have mutated. Returns how
    many failed, and leaves the counters and the output as it found them."""
    global checks, failures, quiet
    saved = (checks, failures, quiet)
    checks, failures, quiet = 0, 0, True
    check_attitudes(document)
    check_telemetry(document)
    check_log_record(document)
    check_ned_enu(document)
    result = failures
    checks, failures, quiet = saved
    return result


def self_test(document):
    """Mutate the document and require the check to notice.

    Without this, a checker whose comparisons never run - a group renamed, a
    loop over an empty list, an exception swallowed - reports zero failures and
    looks exactly like a contract that holds.
    """
    import copy

    def grouped(name, description, mutate):
        mutated = copy.deepcopy(document)
        mutate(mutated)
        caught = run_all(mutated)
        expect("a mutation is caught: %s" % description, caught > 0,
               "(%s changed nothing that is checked)" % name)

    def set_ddeg(doc):
        doc["attitude"]["cases"][5]["ddeg"] += 1

    def set_frame(doc):
        # Flip a nibble in the middle of the payload. Not the first byte: the
        # frame already starts with "aa", so replacing it with "aa" - which is
        # what this mutation used to do - changed nothing and left the
        # self-test reporting a mutation it had not made.
        hexed = list(doc["telemetry"]["frame_hex"])
        at = len(hexed) // 2
        hexed[at] = "0" if hexed[at] != "0" else "1"
        doc["telemetry"]["frame_hex"] = "".join(hexed)

    def set_wire(doc):
        # Flip a nibble in the middle of the record, at a byte the vector
        # actually pins. Not the last byte: roadmap 2.4 gave the record a tail
        # whose final byte is `notch_engaged[2]`, which this vector sets to
        # zero, so "replace the last byte with 00" - which is what this
        # mutation used to do - changed nothing and left the self-test
        # reporting a mutation it had not made. The middle of the version-2
        # fields is pinned by definition and cannot be zeroed by accident.
        hexed = list(doc["log_record"]["wire_hex"])
        at = (LOG_V2_WIRE_BYTES // 2) * 2
        hexed[at] = "0" if hexed[at] != "0" else "1"
        doc["log_record"]["wire_hex"] = "".join(hexed)

    def set_sizeof(doc):
        doc["log_record"]["sizeof_record"] = LOG_WIRE_BYTES

    def set_offset(doc):
        doc["log_record"]["offsets"]["alt_mm"] = 22

    def set_tail_offset(doc):
        # The new fields moved one byte earlier in the struct - which would be
        # a field inserted ahead of the append, the one edit the compatibility
        # story does not allow.
        doc["log_record"]["offsets"]["gyro_filtered"] = 52

    def set_notch(doc):
        doc["log_record"]["record"]["notch_hz"][1] += 1

    def set_quat(doc):
        doc["ned_enu"]["attitudes"][3]["q_enu_wxyz"][1] += 0.01

    def set_heading(doc):
        doc["ned_enu"]["level_attitudes"][2]["sim_heading_rad"] += 0.01

    def set_position(doc):
        doc["ned_enu"]["positions"][1]["enu"] = [0.0, 0.0, 0.0]

    grouped("attitude", "a tenth of a degree in the angle table", set_ddeg)
    grouped("telemetry", "a byte in the telemetry frame", set_frame)
    grouped("log_record", "a byte in the blackbox wire record", set_wire)
    grouped("log_record", "the struct and the wire conflated", set_sizeof)
    grouped("log_record", "a field offset moved", set_offset)
    grouped("log_record", "an appended field placed earlier than the append",
            set_tail_offset)
    grouped("log_record", "a notch centre off by one hertz", set_notch)
    grouped("ned_enu", "a rotated attitude", set_quat)
    grouped("ned_enu", "the yaw origin off by a quarter turn", set_heading)
    grouped("ned_enu", "a position mapped the wrong way", set_position)


def main():
    print("contract_check: the contract, against an implementation that shares "
          "no code with it")
    if not os.path.isfile(VECTORS):
        print("  FAIL  %s is missing; run `make contract-vectors`"
              % os.path.relpath(VECTORS, ROOT))
        print("contract_check: 1 checks, 1 failed")
        return 1
    with open(VECTORS) as handle:
        document = json.load(handle)

    print("  the vectors are from: %s" % document["provenance"]["firmware"])
    print("  the NED->ENU group:   %s" % document["provenance"]["ned_enu"])

    check_attitudes(document)
    check_telemetry(document)
    check_log_record(document)
    check_ned_enu(document)
    self_test(document)

    print("contract_check: %d checks, %d failed" % (checks, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
