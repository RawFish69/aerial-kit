#!/usr/bin/env python3
"""Talk to AerialKit's config protocol.

    akproto.py [--port /dev/ttyACM0] hello|status|list|get N|set N VALUE|save
               |info|help N|log [FILE]|telemetry [HZ]|rc|preflight|mission [VERB]
               |calibrate [VERB [FACE|MV]]

Without --port it speaks the protocol on stdin and stdout, which is how the
simulator and the tests drive it:

    build-host/aerialkit-sim < requests | akproto.py ...

Frame: AA 55 version command length payload... crc16(lo) crc16(hi), the crc
covering the version, command, length and payload. A reply carries the same
command with the top bit set, which is why every function here checks one byte
instead of keeping a table.
"""

import argparse
import struct
import sys
import time

SYNC1 = 0xAA
SYNC2 = 0x55
VERSION = 1
RESPONSE_BIT = 0x80

HELLO = 0x01
PARAM_GET = 0x02
PARAM_SET = 0x03
PARAM_SAVE = 0x04
STATUS = 0x05
LOG_INFO = 0x06
LOG_GET = 0x07
LOG_SOURCE = 0x09
PARAM_INFO = 0x0A
PARAM_HELP = 0x0B
TELEMETRY = 0x08
RC_CHANNELS = 0x0D
SENSOR_INFO = 0x0E
OUTPUT_INFO = 0x0F
OUTPUT_TEST = 0x10
LOG_STREAM = 0x11
PREFLIGHT = 0x12
CALIBRATE = 0x13
MISSION = 0x14
PERF = 0x15
MOTOR_TELEMETRY = 0x16

# The status this firmware gives a command it does not implement, defined in
# ak_proto.c. It arrives correlated to the request, so an old board costs one
# round trip and produces a sentence rather than a timeout - which is only true
# if a client says *which* it was. Every parser here that reads a status byte
# as an enum has to check for this one first: 127 is not a member of any of
# those enums, and reporting it as the nearest one turns "I do not have that
# command" into a fact about the aircraft.
UNKNOWN_COMMAND = 0x7F

# The most the firmware will stream, from ak_proto.h. Asking for more is not an
# error: the reply says what the rate will actually be.
TELEMETRY_MAX_HZ = 50
LOG_STREAM_MAX_HZ = 50

SET_STATUS = {
    0: "ok",
    1: "no such parameter",
    2: "value refused",
    3: "nowhere to save",
    4: "the board refused the write",
}


def set_reply(payload):
    """`(status, message)` from a PARAM_SET reply.

    The message is the parameter table's own words - "out of 0.000..1.000" -
    which is what a configurator shows beside a refused value. It is appended
    after the status byte so a client that only reads the status is unaffected;
    an older board that sends nothing after the status reads as an empty
    message, which is the honest answer rather than an error."""
    message = c_string(payload, 1) if len(payload) > 1 else ""
    return payload[0], message


# The capability bits, from AK_PROTO_FEATURE_* in ak_proto.h. A bit means this
# build answers that command and behaves as docs/16-protocol.md says - it is
# not a version, because the version byte only moves when an existing byte
# changes meaning and a new command is a new command.
FEATURES = {
    0: "PARAM_INFO",
    1: "PARAM_DEFAULT",
    2: "APPLIES_ON_WRITE",
    3: "GATES_ON_ARMED",
    4: "RC_CHANNELS",
    5: "SENSOR_INFO",
    6: "OUTPUT_INFO",
    7: "OUTPUT_TEST",
    8: "LOG_STREAM",
    9: "PREFLIGHT",
    10: "CALIBRATE",
    11: "MISSION",
    12: "PERF",
    13: "MOTOR_TELEMETRY",
}


def parse_hello(payload):
    """`hello` as a dict, including the two fields that may be absent.

    `features` and `config_hash` are appended after the four fields HELLO used
    to end with, which is what lets them be added without moving the protocol
    version. A board written before them answers a payload that ends earlier,
    and **both keys come back as None** - not as 0 and not as an empty set.

    That distinction is the whole reason this function exists rather than four
    lines at the call site. A board that cannot say what it supports and a
    board that supports nothing are different claims, and only one of them is
    true of a build that predates the field."""
    version = payload[0]
    product = c_string(payload, 1)
    count = struct.unpack_from("<H", payload, 2 + len(product))[0]
    changed = c_string(payload, 4 + len(product))
    # Where the four old fields end: the version, the product and its
    # terminator, the u16 count, and the changed string and its terminator.
    at = 5 + len(product) + len(changed)
    trailing = len(payload) - at
    if trailing == 0:
        return {"version": version, "product": product, "count": count,
                "changed": changed, "features": None, "config_hash": None}
    if trailing < 8:
        # Not a default and not a short word: the two fields are written
        # together or not at all, so a reply carrying four of the eight is a
        # truncated frame and reading half of it would invent a capability.
        raise IOError(
            "hello: the capability word and hash are 8 bytes together, and the "
            "reply carried %u after them" % trailing)
    features, config_hash = struct.unpack_from("<II", payload, at)
    return {"version": version, "product": product, "count": count,
            "changed": changed, "features": features, "config_hash": config_hash}


def feature_names(word):
    """The set bits, by name. `None` - an absent word - is an empty list here
    and the caller is expected to say 'cannot tell' rather than 'none'."""
    if word is None:
        return []
    return [name for bit, name in sorted(FEATURES.items()) if word & (1 << bit)]


def crc16(data):
    """CRC-16/CCITT-FALSE, the same one the firmware computes."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(command, payload=b""):
    body = bytes([VERSION, command, len(payload)]) + payload
    crc = crc16(body)
    return bytes([SYNC1, SYNC2]) + body + struct.pack("<H", crc)


class Client:
    """Framing on top of a byte stream, so it works on a pipe or a serial port."""

    def __init__(self, read, write):
        self._read = read
        self._write = write
        # The bytes of the last frame read, header, crc and all. Kept so a test
        # can compare what actually arrived rather than what this class would
        # have built from what it made of it.
        self.last_frame = b""

    def request(self, command, payload=b""):
        self._write(build(command, payload))
        return self._read_reply(command | RESPONSE_BIT)

    def request_while_streaming(self, command, payload=b""):
        """The same, for a command sent while a telemetry stream is running.

        The stream's frames carry the command byte *without* the response bit,
        so they are distinguishable - but one of them can arrive between the
        request and its reply, and a client that treated that as a protocol
        error would fail whenever the timing went against it. That is what
        happened to `subscribe(hz=0)`: the reply to stopping the stream raced
        the last frame of it."""
        self._write(build(command, payload))
        return self._read_reply(command | RESPONSE_BIT, allow=(command,))

    def _read_reply(self, wanted, allow=()):
        """The reply to `wanted`, skipping frames in `allow`.

        A client sharing a connection with a telemetry stream has to expect
        frames it did not ask for. Skipping them is not laxity: everything else
        - a reply to a different command, a frame with no response bit - is
        still a protocol error and still raises.
        """
        # Bounded: each read has the caller's timeout, but a stream of allowed
        # frames resets none of it, so a lost reply under a running telemetry
        # stream used to wait forever. Five hundred skipped frames is ten
        # seconds at the stream's 50 Hz ceiling.
        for _ in range(500):
            command, payload = self.read_frame()
            if command == wanted:
                return payload
            if command not in allow:
                raise IOError("reply to %02x, wanted %02x" % (command, wanted))
        raise IOError("no reply to %02x among 500 streamed frames" % wanted)

    def read_frame(self):
        """The next frame on the stream, whoever it is for."""
        state = 0
        body = b""
        raw = bytearray()
        expected = 3
        while True:
            byte = self._read(1)
            if not byte:
                raise IOError("the other end stopped talking")
            value = byte[0]
            raw += byte
            if state == 0:
                state = 1 if value == SYNC1 else 0
            elif state == 1:
                state = 2 if value == SYNC2 else 0
                body = b""
                expected = 3
            elif state == 2:
                body += bytes([value])
                if len(body) == 3:
                    expected = 3 + body[2]
                if len(body) >= expected:
                    state = 3
            elif state == 3:
                body += bytes([value])
                state = 4
            else:
                body += bytes([value])
                crc = crc16(body[:expected])
                received = struct.unpack("<H", body[expected:expected + 2])[0]
                if crc != received:
                    raise IOError("bad checksum in the reply")
                self.last_frame = bytes(raw)
                return body[1], body[3:expected]


def parse_status(payload):
    state, link, fix, sats = payload[0:4]
    roll, pitch, yaw = struct.unpack_from("<hhh", payload, 4)
    lat, lon = struct.unpack_from("<ii", payload, 10)
    return {
        "flight_state": state,
        "link_live": link,
        "gps_fix_type": fix,
        "gps_satellites": sats,
        "roll_deg": roll / 10.0,
        "pitch_deg": pitch / 10.0,
        "yaw_deg": yaw / 10.0,
        "lat": lat / 1e7,
        "lon": lon / 1e7,
        "motors": list(payload[18:22]),
    }


def parse_telemetry(payload):
    """A pushed telemetry frame: the status body with an uptime in front.

    It arrives on its own - nothing asked for this frame - and the command byte
    has no response bit set, which is how it is told apart from the reply to the
    subscribe that started the stream.
    """
    uptime_ms = struct.unpack_from("<i", payload, 0)[0]
    status = parse_status(payload[4:])
    status["uptime_ms"] = uptime_ms
    return status


def subscribe(client, hz):
    """Ask for a telemetry stream. Returns the rate the firmware agreed to."""
    reply = client.request_while_streaming(TELEMETRY, bytes([hz]))
    return reply[0]


def next_telemetry(client):
    """Blocks until a telemetry frame arrives, and returns it parsed."""
    for _ in range(500):
        command, payload = client.read_frame()
        if command == TELEMETRY:
            return parse_telemetry(payload)
    raise IOError("no telemetry frame among 500 others")


# The status byte on RC_CHANNELS, and the number of channels a reply can carry.
# Both from ak_proto.h; the count is a bound on the wire's side of the receiver
# rather than a copy of the flight core's AK_RC_CHANNELS, so a firmware that
# grew its receiver keeps this client working.
RC_OK = 0
RC_NONE = 1
RC_MAX_CHANNELS = 16
RC_STATUS = {RC_OK: "ok", RC_NONE: "no receiver input"}

# Facts about the receiver as bits, in AK_PROTO_RC_* order. Facts rather than
# warnings: which of them is worth colouring red is the screen's business.
RC_FLAGS = {
    0: "link",
    1: "failsafe",
    2: "decoded",
    3: "no inverter",
    4: "telemetry",
}
RC_SWITCHES = {0: "arm", 1: "angle"}
# The protocols the firmware can decode, which are ak_rc_protocol_t's numbers.
# A protocol this client has never heard of keeps its number and gets no name -
# the same rule the parameter groups follow, and for the same reason: "the
# board says 9" is a fact, and guessing a neighbour's name is not.
RC_PROTOCOLS = {0: "CRSF", 1: "SBUS"}


def _named_bits(mask, table):
    """The set bits of `mask`, by name, from a table keyed by bit."""
    return [name for bit, name in sorted(table.items()) if mask & (1 << bit)]


def parse_rc_channels(payload):
    """`rc channels` as a dict, with three replies that look alike kept apart.

    They are what this function exists for, because on a screen all three can be
    drawn as eight zeroed bars:

      - `status = RC_NONE`, one byte and nothing else - this board has no
        receiver port. A fact about the hardware, and every field below comes
        back empty rather than zero, because a channel list of zeroes here would
        be a reading of a receiver that does not exist.
      - status ok with `link` clear - a receiver port with nothing framing on
        it. A wiring or binding fault, and a different afternoon's work.
      - status ok with `link` and `decoded` set and the sticks at zero - a
        handset sitting centred. Neither fault.

    The third is why `decoded` is a bit of its own: zeroed sticks with it set
    mean "centred" and without it mean "no idea", from the same bytes.

    A build without the command does not reach that branch at all: `0x7F` is not
    a member of the status enum, and it is refused here rather than folded in,
    because reporting it as `RC_NONE` would turn "I do not have that command"
    into a statement about a board that may have a receiver plugged into it
    right now."""
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "rc channels: the board answered 0x%02x, the status this firmware "
            "gives a command it does not implement - this build predates "
            "`rc channels`" % UNKNOWN_COMMAND)

    status = raw
    if status == RC_NONE:
        if len(payload) != 1:
            # "One byte and no more" is the firmware's rule (ak_proto.c), and it
            # is checked here rather than trusted: everything after this byte -
            # flags, channels, counters - would be a claim about a receiver the
            # board does not have, and a client that ignored the tail would be
            # reading a stream of bytes it has decided to believe nothing about.
            raise IOError("rc channels: a board with no receiver answers one "
                          "byte, and this reply carried %u" % len(payload))
        return {"status": status, "status_name": RC_STATUS[status],
                "flags": 0, "flag_names": [], "protocol": None,
                "protocol_name": None, "count": 0, "channels": [],
                "sticks": [], "switches": 0, "switch_names": [],
                "bytes": 0, "frames": 0, "crc_errors": 0, "rejected": 0,
                "lost": 0, "failsafe_frames": 0, "dropped": 0}
    if status != RC_OK:
        raise IOError("rc channels: status %u" % status)

    flags, protocol, count = payload[1:4]
    if count > RC_MAX_CHANNELS:
        # Reading on would take the sticks for channels and the four stick
        # values for counters. A count the firmware cannot have written is a
        # broken frame, not a frame with more channels in it.
        raise IOError("rc channels: the reply claims %u channels, and the "
                      "protocol carries at most %u" % (count, RC_MAX_CHANNELS))

    # The length the count implies, which the frame has to be. The firmware
    # clamps its count to what it will actually write, so a reply of the wrong
    # size is a firmware bug and a short one would be misread rather than
    # refused if this were not checked.
    implied = 4 + count * 2 + 8 + 1 + 28
    if len(payload) != implied:
        raise IOError("rc channels: %u channels means a %u-byte reply and this "
                      "one is %u bytes" % (count, implied, len(payload)))

    channels = list(struct.unpack_from("<%dH" % count, payload, 4))
    # Signed, and that is not a detail: -1000 written as a magnitude is +1000,
    # which is full stick the other way.
    sticks = list(struct.unpack_from("<4h", payload, 4 + count * 2))
    switches = payload[4 + count * 2 + 8]
    counters = struct.unpack_from("<7I", payload, 4 + count * 2 + 8 + 1)
    return {"status": status, "status_name": RC_STATUS[status],
            "flags": flags, "flag_names": _named_bits(flags, RC_FLAGS),
            "protocol": protocol,
            "protocol_name": RC_PROTOCOLS.get(protocol),
            "count": count, "channels": channels, "sticks": sticks,
            "switches": switches,
            "switch_names": _named_bits(switches, RC_SWITCHES),
            "bytes": counters[0], "frames": counters[1],
            "crc_errors": counters[2], "rejected": counters[3],
            "lost": counters[4], "failsafe_frames": counters[5],
            "dropped": counters[6]}


# The sensors SENSOR_INFO answers about, in ak_proto.h's order, and the two
# statues. NO_SUCH is a claim about the build and the absent body is a claim
# about the aircraft - the same split the receiver's RC_NONE makes, one level
# up, and a caller that showed them the same way would send somebody looking
# for a driver when the part is simply not soldered in.
SENSOR_IMU = 0
SENSOR_BARO = 1
SENSOR_RANGE = 2
SENSOR_BATTERY = 3
SENSOR_GPS = 4
SENSOR_TOPICS = 5
SENSOR_NAMES = {SENSOR_IMU: "imu", SENSOR_BARO: "baro", SENSOR_RANGE: "range",
                SENSOR_BATTERY: "battery", SENSOR_GPS: "gps"}

SENSOR_OK = 0
SENSOR_NO_SUCH = 1
SENSOR_STATUS = {SENSOR_OK: "ok", SENSOR_NO_SUCH: "no such topic"}

# The fixed width of the driver-name field. A body's length does not depend on
# which part answered, which is what lets the checks below compare a reply
# against a number rather than against the length of the name in it.
SENSOR_NAME_LENGTH = 12

# The body each topic carries, counted from the wire description in ak_proto.h
# and not from struct sizes - the C side counts them the same way in
# tests/test_proto.c, and the two agreeing is a check rather than a hope.
SENSOR_BODY_LENGTH = {
    SENSOR_IMU: 12 + 1 + 1 + 6 + 6 + 6 + 6 + 4 + 4,
    SENSOR_BARO: 12 + 4 + 2 + 1 + 4 + 4 + 1 + 4 + 4 + 4 + 4 + 4 + 4,
    SENSOR_RANGE: 12 + 1 + 2 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 2,
    SENSOR_BATTERY: 1 + 1 + 1 + 1 + 2 + 2 + 2 + 2 + 1 + 2 + 2 + 4 + 4 + 4,
    SENSOR_GPS: 1 + 1 + 1 + 1 + 1 + 4 + 4 + 4 + 4 + 4 + 1 + 4 + 4 + 4 + 4 +
        1 + 1 + 4 + 4 + 4,
}

# Why a board has no IMU, as bits. Not ak_imu_result_t's numbers: that enum is
# the driver layer's and this is the wire's, and pinning one to the other would
# make renumbering either a protocol change.
IMU_ABSENT = {
    1: "nothing answered on the bus",
    2: "something answered, and it is not a known part",
    3: "the right part answered and would not configure",
}

BATTERY_STATES = {0: "absent", 1: "ok", 2: "warn", 3: "critical"}

def _sensor_body(topic, body):
    """One topic's body, by the offsets the header documents."""
    def u16(at):
        return struct.unpack_from("<H", body, at)[0]

    def i16(at):
        return struct.unpack_from("<h", body, at)[0]

    def u32(at):
        return struct.unpack_from("<I", body, at)[0]

    def i32(at):
        return struct.unpack_from("<i", body, at)[0]

    if topic == SENSOR_IMU:
        absent = body[12]
        return {
            "driver": c_string(body, 0),
            "absent": absent,
            "absent_reason": IMU_ABSENT.get(absent),
            "whoami": body[13],
            "accel": [i16(14 + 2 * i) for i in range(3)],
            "gyro": [i16(20 + 2 * i) for i in range(3)],
            "align": [i16(26 + 2 * i) for i in range(3)],
            "gyro_bias": [i16(32 + 2 * i) for i in range(3)],
            "samples": u32(38),
            "errors": u32(42),
        }
    if topic == SENSOR_BARO:
        return {
            "driver": c_string(body, 0),
            "pressure_pa": i32(12),
            "temperature_c": i16(16) / 100.0,
            "have_reference": body[18],
            "reference_pa": i32(19),
            "height_cm": i32(23),
            "have_gps_reference": body[27],
            "fused_cm": i32(28),
            "samples": u32(32),
            "errors": u32(36),
            "fails": u32(40),
            "baro_samples": u32(44),
            "gps_samples": u32(48),
        }
    if topic == SENSOR_RANGE:
        return {
            "driver": c_string(body, 0),
            "address": body[12],
            "max_mm": u16(13),
            # Negative is "nothing in range", sent as it is rather than clamped
            # to zero: zero is a wall against the lens.
            "distance_mm": i32(15),
            "age_ms": u32(19),
            "samples": u32(23),
            "out_of_range": u32(27),
            "rejected": u32(31),
            "faults": u32(35),
            "fails": u32(39),
            "land_mm": u32(43),
            "agree_cm": u16(47),
        }
    if topic == SENSOR_BATTERY:
        return {
            "ready": body[0],
            "have_reading": body[1],
            "state": body[2],
            "state_name": BATTERY_STATES.get(body[2]),
            "cells": body[3],
            "volts": u16(4) / 100.0,
            "volts_per_cell": u16(6) / 100.0,
            "pin_mv": i16(8),
            "ratio": u16(10) / 1000.0,
            "rth": body[12],
            "warn_cell_v": u16(13) / 1000.0,
            "critical_cell_v": u16(15) / 1000.0,
            "samples": u32(17),
            "rejected": u32(21),
            "returns": u32(25),
        }
    if topic == SENSOR_GPS:
        return {
            "have_fix": body[0],
            "fix_type": body[1],
            "fix_ok": body[2],
            "satellites": body[3],
            "valid_now": body[4],
            "lat_e7": i32(5),
            "lon_e7": i32(9),
            "alt_msl_mm": i32(13),
            "speed_mm_s": i32(17),
            "course_e5": i32(21),
            "have_home": body[25],
            "home_lat_e7": i32(26),
            "home_lon_e7": i32(30),
            "home_distance_m": i32(34),
            "home_bearing_cdeg": i32(38),
            "returning": body[42],
            "rth_enabled": body[43],
            "fixes": u32(44),
            "dropped": u32(48),
            "config_sends": u32(52),
        }
    raise IOError("sensor info: no body is defined for topic %u" % topic)


def parse_sensor_info(payload):
    """`sensor info` as a dict, with "not fitted" and "reading zero" kept apart.

    The reply is `status, topic, present` and then a body that is *there or not
    there* - not a body of zeroes - and this function is the client's half of
    that promise. A caller that skipped `present` and read the body anyway would
    see all zeros for a board with no barometer, which is the same bytes a
    barometer reading zero pressure produces; that is the confusion the
    opcode's shape exists to prevent, so the absent case comes back with
    `body: None` rather than a dict of zeros.

    Three answers, and they are three different sentences:

      - status NO_SUCH - this build does not answer for that topic at all.
      - status ok with `present` clear - the build knows the question and this
        board has nothing fitted. Something to go and look at.
      - status ok with `present` set - a reading, and a zero in it is a zero.

    A build without the command does not reach any of them: 0x7F is refused
    here, because reporting it as NO_SUCH would turn "I do not have that
    command" into a statement about a board that may have the part."""
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "sensor info: the board answered 0x%02x, the status this firmware "
            "gives a command it does not implement - this build predates "
            "`sensor info`" % UNKNOWN_COMMAND)

    status = raw
    if status not in (SENSOR_OK, SENSOR_NO_SUCH):
        raise IOError("sensor info: status %u" % status)

    if len(payload) < 3:
        raise IOError("sensor info: the reply carries the status, the topic "
                      "and the present byte on every path, and this one is %u "
                      "byte(s)" % len(payload))

    topic, present = payload[1], payload[2]
    if status == SENSOR_NO_SUCH:
        # Every refusal is three bytes and no more. A tail here is a frame this
        # client does not understand rather than an answer with extra in it.
        if len(payload) != 3:
            raise IOError("sensor info: a refused topic answers three bytes, "
                          "and this reply carried %u" % len(payload))
        return {"status": status, "status_name": SENSOR_STATUS[status],
                "topic": topic, "topic_name": SENSOR_NAMES.get(topic),
                "present": 0, "body": None}

    if present == 0:
        if len(payload) != 3:
            raise IOError(
                "sensor info: an absent sensor carries no body - that is what "
                "tells it from a sensor reading zero - and this reply has %u "
                "bytes after the topic and present" % (len(payload) - 3))
        return {"status": status, "status_name": SENSOR_STATUS[status],
                "topic": topic, "topic_name": SENSOR_NAMES.get(topic),
                "present": 0, "body": None}

    body = payload[3:]
    if len(body) != SENSOR_BODY_LENGTH.get(topic):
        raise IOError(
            "sensor info: topic %u carries a %s-byte body and this one is %u "
            "- a client that read on would take the next field's bytes for "
            "this one's" % (topic, SENSOR_BODY_LENGTH.get(topic),
                            len(body)))
    return {"status": status, "status_name": SENSOR_STATUS[status],
            "topic": topic, "topic_name": SENSOR_NAMES.get(topic),
            "present": 1, "body": _sensor_body(topic, body)}


def sensor_info(client, topic):
    """One SENSOR_INFO round trip."""
    return parse_sensor_info(client.request(SENSOR_INFO, bytes([topic])))


# The two kinds of output, in ak_proto.h's order. A build with an output whose
# kind is neither of these keeps its number and gets no name - the rule the
# parameter groups and the RC protocols follow - because "the board says 3" is
# a fact and calling it a servo is not.
OUTPUT_MOTOR = 0
OUTPUT_SERVO = 1
OUTPUT_KINDS = {OUTPUT_MOTOR: "motor", OUTPUT_SERVO: "servo"}

OUTPUT_INFO_OK = 0
OUTPUT_INFO_NONE = 1
OUTPUT_INFO_TOO_MANY = 2
OUTPUT_INFO_STATUS = {OUTPUT_INFO_OK: "ok",
                      OUTPUT_INFO_NONE: "no outputs",
                      OUTPUT_INFO_TOO_MANY: "more outputs than one frame carries"}

# Seven bytes a descriptor, counted from the wire description in ak_proto.h and
# not from a struct - the C side counts them the same way in tests/test_proto.c.
OUTPUT_ENTRY_BYTES = 7


def parse_output_info(payload):
    """`output info` as a dict, with "no list" and "an empty list" kept apart.

    The header is `status, count, motors, servos, cap` and then `count`
    descriptors of seven bytes. The status is the part worth reading carefully,
    because two of its three values would draw the same blank panel:

      - NONE - there is no list. Either this board has nothing to drive or it is
        not ready to say; both are "no outputs", and neither is an aircraft with
        zero motors. A zeroed `OUTPUT_INFO` reply is not a thing the firmware
        writes (ak_proto.c answers NONE for it), and a client that read on past
        this status would be inventing one.
      - TOO_MANY - the board has more outputs than one frame carries, so it
        refused rather than paged. A short list drawn as the whole aircraft is a
        client missing a servo it will then go looking for in the wiring.
      - OK - here is the list, and `count` is how many entries follow.

    The count and the body are checked against each other rather than trusted.
    The firmware clamps its count to what it writes, so a reply whose length
    disagrees with its own count is a firmware bug, and one that read on would
    take the next entry's kind byte for this one's index.

    A build without the command raises rather than answering NONE: 0x7F is not a
    member of the status enum, and folding it in would turn "I do not have that
    command" into a statement about a board that has four ESCs bolted to it."""
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "output info: the board answered 0x%02x, the status this firmware "
            "gives a command it does not implement - this build predates "
            "`output info`" % UNKNOWN_COMMAND)

    status = raw
    if status not in OUTPUT_INFO_STATUS:
        raise IOError("output info: status %u" % status)

    if len(payload) < 5:
        # The header is on every path - including the two that carry no
        # descriptors - so the cap comes back even from a board with nothing to
        # drive. A client that only learned the ceiling from a board with
        # outputs would have no ceiling on a board without.
        raise IOError("output info: the status, count, split and cap are five "
                      "bytes on every path and this reply is %u"
                      % len(payload))

    status, count, motors, servos, cap = payload[:5]
    if status != OUTPUT_INFO_OK:
        if len(payload) != 5:
            raise IOError("output info: status %u carries no list, and this "
                          "reply has %u bytes behind the header"
                          % (status, len(payload) - 5))
        if count or motors or servos:
            raise IOError("output info: status %u carries no list, and this "
                          "reply claims %u of them" % (status, count))
        return {"status": status, "status_name": OUTPUT_INFO_STATUS[status],
                "count": 0, "motors": 0, "servos": 0, "cap_pct": cap,
                "outputs": []}

    implied = 5 + count * OUTPUT_ENTRY_BYTES
    if len(payload) != implied:
        raise IOError("output info: %u outputs means a %u-byte reply and this "
                      "one is %u bytes" % (count, implied, len(payload)))
    if motors + servos != count:
        # The split is what lets a screen say "4 motors, 2 servos" without
        # counting descriptors, and a split that does not add up means one of
        # the two numbers was written from something other than the list.
        raise IOError("output info: the header says %u motors and %u servos "
                      "and %u descriptors follow" % (motors, servos, count))

    outputs = []
    for i in range(count):
        at = 5 + i * OUTPUT_ENTRY_BYTES
        kind, index, reversed_, trim_us, travel_us = struct.unpack_from(
            "<BBBhH", payload, at)
        outputs.append({
            "kind": kind, "kind_name": OUTPUT_KINDS.get(kind),
            "index": index, "reversed": bool(reversed_),
            # The wire always carries all three. Whether they mean anything is
            # `kind`'s answer, and saying so here is what keeps a screen from
            # drawing a motor with a neutral and a travel.
            "trim_us": trim_us if kind == OUTPUT_SERVO else None,
            "travel_us": travel_us if kind == OUTPUT_SERVO else None,
        })
    return {"status": status, "status_name": OUTPUT_INFO_STATUS[status],
            "count": count, "motors": motors, "servos": servos, "cap_pct": cap,
            "outputs": outputs}


def output_info(client):
    """One OUTPUT_INFO round trip. The request is the empty frame."""
    return parse_output_info(client.request(OUTPUT_INFO))


OUTPUT_TEST_HOLD = 0
OUTPUT_TEST_STOP = 1

OUTPUT_TEST_OK = 0
OUTPUT_TEST_STOPPED = 1
OUTPUT_TEST_NO_OUTPUT = 2
OUTPUT_TEST_ARMED = 3
OUTPUT_TEST_NO_BOARD = 4
OUTPUT_TEST_NO_OP = 5
OUTPUT_TEST_STATUS = {
    OUTPUT_TEST_OK: "ok",
    OUTPUT_TEST_STOPPED: "stopped",
    OUTPUT_TEST_NO_OUTPUT: "no such output",
    OUTPUT_TEST_ARMED: "refused: the aircraft is armed",
    OUTPUT_TEST_NO_BOARD: "this board drives no outputs",
    OUTPUT_TEST_NO_OP: "the request named no verb",
}


def parse_output_test(payload, sent=None):
    """`output test` as a dict, with the echo checked against what was sent.

    The reply echoes the op, kind, index and the percentage actually driven, on
    every path, and every field is checked against `sent` when the caller has
    it. That check is the point of the echo: a client holding a screen of four
    outputs has to know which answer it is holding, and one that had lost track
    of the state can send a STOP without asking first.

    Two properties of the status are worth stating, because both are things a
    client acts on:

      - `level_pct` is what *will* be driven, not what was asked for. Asking for
        200 is not an error and is not obeyed; it comes back as the firmware's
        cap, and a UI that counted down from its own request would be counting
        something else.
      - STOPPED is not a refusal. A stop is answered before every other check,
        so it arrives even while the aircraft is armed or the hold it is
        stopping was refused - which is the whole reason `op` is a value on the
        wire rather than a second command.

    A client that gets anything other than OK must not leave a control drawn as
    though something is running: NO_OUTPUT, ARMED, NO_BOARD and NO_OP all mean
    nothing was driven."""
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "output test: the board answered 0x%02x, the status this firmware "
            "gives a command it does not implement - this build predates "
            "`output test`, and a configurator that read this as a refusal to "
            "drive an output would be reporting a policy decision that was "
            "never made" % UNKNOWN_COMMAND)

    status = raw
    if status not in OUTPUT_TEST_STATUS:
        raise IOError("output test: status %u" % status)
    if len(payload) != 7:
        raise IOError("output test: the reply is seven bytes on every path and "
                      "this one is %u" % len(payload))

    _, op, kind, index, level_pct, remaining_ms = struct.unpack("<BBBBBH", payload)
    result = {"status": status, "status_name": OUTPUT_TEST_STATUS[status],
              "op": op, "op_name": "stop" if op == OUTPUT_TEST_STOP else
                                    ("hold" if op == OUTPUT_TEST_HOLD else None),
              "kind": kind, "kind_name": OUTPUT_KINDS.get(kind),
              "index": index, "level_pct": level_pct,
              "remaining_ms": remaining_ms}
    if sent is not None:
        wanted_op, wanted_kind, wanted_index = sent[0], sent[1], sent[2]
        if (op, kind, index) != (wanted_op, wanted_kind, wanted_index):
            raise IOError(
                "output test: asked about kind %u output %u with op %u and the "
                "reply names kind %u output %u with op %u - the answer belongs "
                "to another request" % (wanted_kind, wanted_index, wanted_op,
                                        kind, index, op))
    return result


def output_test(client, op, kind=OUTPUT_MOTOR, index=0, level_pct=0):
    """One OUTPUT_TEST round trip, echoing the request back for the check."""
    payload = bytes([op, kind, index, level_pct])
    return parse_output_test(
        client.request(OUTPUT_TEST, payload),
        sent=(op, kind, index))


def stop_outputs(client):
    """The stop, as its own verb - what a client sends when it is not sure.

    A stop names no output, so kind and index are whatever a hold would have
    used; the firmware answers the stop before it looks at either."""
    return output_test(client, OUTPUT_TEST_STOP)


LOG_FIELDS = ["time_ms", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y",
              "accel_z", "roll", "pitch", "yaw", "alt_mm", "stick_roll",
              "stick_pitch", "stick_yaw", "stick_throttle", "torque_roll",
              "torque_pitch", "torque_yaw", "motor1", "motor2", "motor3",
              "motor4", "state", "flags", "lat_e7", "lon_e7",
              # Version 3's tail (roadmap 2.4). The unfiltered gyro above is the
              # column these are read against: the pair is what makes a
              # filter's effect visible in a log, which is the whole point of
              # carrying both. The names are the firmware's own, spelled the way
              # `log dump` spells them, because a CSV from the console and one
              # from here have to be the same file and this list is what decides
              # whether a reader recognises the header it is given.
              "gyro_fx", "gyro_fy", "gyro_fz",
              "notch_hz_x", "notch_hz_y", "notch_hz_z",
              "notch_engaged_x", "notch_engaged_y", "notch_engaged_z",
              # Version 4's tail (roadmap 4.1): what the rate loop was asked
              # for and what each of its terms did about it, the loop's own
              # microsecond clock, and the pack voltage. Spelled as `log dump`
              # spells them, for the same reason as above.
              "time_us", "setpoint_roll", "setpoint_pitch", "setpoint_yaw",
              "p_roll", "p_pitch", "p_yaw", "i_roll", "i_pitch", "i_yaw",
              "d_roll", "d_pitch", "d_yaw", "vbat_mv"]

# The record's length on the wire, and the lengths versions 2 and 3 were.
#
# **A record carries no version of its own: its length is the version
# statement.** 87 is a record with the controller's columns, exactly 66 is one
# from a board that predates them, exactly 51 one that predates the filtering
# fields too, and anything else is a record that was cut short - a fault, not a
# version, and reported as one.
LOG_RECORD_BYTES = 87
LOG_RECORD_V3_BYTES = 66
LOG_RECORD_V2_BYTES = 51


def parse_log_record(payload, offset=1):
    """The record layout from ak_log.h, decoded the way a client would.

    `offset` is where the record starts. LOG_GET puts one status byte in front
    of it; a streamed frame puts status, source and index there. Naming the
    offset at the call site is what keeps those two from being confused, since
    both would otherwise decode - the streamed one would simply be two bytes
    out and read as a record with the wrong timestamp.

    An older record's missing columns - version 2's filtering and controller
    columns, version 3's controller columns - come back as empty strings, not
    as zeros. "This board did not send a notch centre" and "the notch centre is
    0 Hz" are different claims, and 0 is inside the notch's own range, so a
    number there would look like a measurement rather than a mistake. The
    empty field is what the CSV has for absent, and it keeps every row the same
    width as LOG_FIELDS, which the checks downstream assert."""
    available = len(payload) - offset
    if available >= LOG_RECORD_BYTES:
        length = LOG_RECORD_BYTES
    elif available in (LOG_RECORD_V3_BYTES, LOG_RECORD_V2_BYTES):
        length = available
    else:
        raise IOError(
            "a log record is %d bytes, or %d from a firmware that predates the "
            "controller fields, or %d from one that predates the filtering "
            "fields; this frame left %d"
            % (LOG_RECORD_BYTES, LOG_RECORD_V3_BYTES, LOG_RECORD_V2_BYTES,
               available))
    body = payload[offset:offset + length]

    def u16(at):
        return int.from_bytes(body[at:at + 2], "little")

    def i16(at):
        value = u16(at)
        return value - 0x10000 if value & 0x8000 else value

    values = [int.from_bytes(body[0:4], "little")]
    values += [i16(4), i16(6), i16(8)]              # gyro
    values += [i16(10), i16(12), i16(14)]           # accel
    values += [i16(16), i16(18)]                    # attitude
    values += [i16(20)]                             # yaw
    alt = int.from_bytes(body[22:26], "little")
    values += [alt - 0x100000000 if alt & 0x80000000 else alt]  # alt_mm
    values += [i16(26), i16(28), i16(30), i16(32)]  # sticks
    values += [b - 0x100 if b & 0x80 else b for b in body[34:37]]  # torque
    values += list(body[37:41])                     # motors
    values += [body[41], body[42]]                  # state, flags
    lat = int.from_bytes(body[43:47], "little")
    lon = int.from_bytes(body[47:51], "little")
    values += [lat - 0x100000000 if lat & 0x80000000 else lat,   # lat_e7
               lon - 0x100000000 if lon & 0x80000000 else lon]   # lon_e7
    if length >= LOG_RECORD_V3_BYTES:
        values += [i16(51), i16(53), i16(55)]       # gyro_filtered
        values += [u16(57), u16(59), u16(61)]       # the notch centres, in Hz
        values += list(body[63:66])                 # which axes each engaged
    if length >= LOG_RECORD_BYTES:
        values += [int.from_bytes(body[66:70], "little")]  # time_us
        values += [i16(70), i16(72), i16(74)]       # rate setpoints, 0.1 dps
        values += [b - 0x100 if b & 0x80 else b for b in body[76:85]]  # P, I, D
        values += [u16(85)]                         # vbat_mv
    if len(values) < len(LOG_FIELDS):
        values += [""] * (len(LOG_FIELDS) - len(values))
    return values


# The comment block a pulled log carries, word for word what `log dump` writes -
# a log is a log whichever way it left the aircraft, so a file from the console
# and a file from here have to be the same file, and the column line below is
# what makes that checkable. The last line is the one the console never writes,
# and it is here because a pull, unlike a dump, can come from firmware that
# predates the columns: the client would rather name the empty fields than write
# a row whose zeroes look like a reading.
LOG_UNITS = (
    "# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, "
    "alt mm above the take-off reference, sticks per-mille, torque "
    "percent, motor 0..254\n"
    "# gyro_* is the driver's reading; gyro_f* is what the notch bank and the "
    "two low-passes made of it, and is the number the controller flew on\n"
    "# notch_hz_* is 0 when no notch is engaged on that axis; notch_engaged_* "
    "is how many a measurement has put in place there\n"
    "# setpoint_* is the rate loop's target in 0.1 dps; p_*, i_*, d_* are its "
    "terms in torque percent (P + I - D is the torque before its clamp, "
    "+/-127 is saturated); vbat_mv is 0 unless flags has 0x10; flags 0x08 is "
    "angle mode\n"
    "# the last fourteen columns are empty on a record from firmware that "
    "predates the controller fields, and the nine before them too on one that "
    "predates the filtering fields\n")

LOG_HEADER = ("# aerialkit blackbox, pulled over the config protocol\n"
              + LOG_UNITS + ",".join(LOG_FIELDS) + "\n")

# The three logs a device can have, and the name for each. Which ones exist is
# the device's answer, not the client's assumption: an ESP32 has no log in
# flash yet, and a board with no retained RAM still has a long ring.
LOG_SOURCES = {"fast": 0, "long": 1, "flash": 2}
LOG_SOURCE_NAMES = {v: k for k, v in LOG_SOURCES.items()}


def select_log(client, source):
    """Selects a log and returns how many records it holds, or None when the
    device does not have that log at all - which is a different answer from an
    empty one."""
    payload = client.request(LOG_SOURCE, bytes([source]))
    if payload[0] != 0:
        return None
    return int.from_bytes(payload[2:4], "little")


def fetch_log(client, write, source=0):
    """Pulls every record of one log and writes CSV, with the same header the
    console's own dump uses - so a log is a log whichever way it left the
    aircraft.

    A record the device refuses is a slot whose checksum did not survive - the
    one the power cut in half - and it is counted and skipped rather than
    treated as the end of the log: everything after it is still good, and a
    reader that stopped there would throw away the part that matters."""
    count = select_log(client, source)
    if count is None:
        raise IOError("this device has no %s log"
                      % LOG_SOURCE_NAMES.get(source, "source %u" % source))
    name = LOG_SOURCE_NAMES.get(source, "source %u" % source)
    write("# aerialkit blackbox (%s), pulled over the config protocol\n"
          % name
          + LOG_UNITS + ",".join(LOG_FIELDS) + "\n")
    skipped = 0
    for index in range(count):
        payload = client.request(LOG_GET, index.to_bytes(2, "little"))
        if payload[0] != 0:
            skipped += 1
            continue
        write(",".join(str(v) for v in parse_log_record(payload)) + "\n")
    return count - skipped


# A pushed log frame's status byte, from AK_PROTO_LOG_STREAM_* in ak_proto.h.
# RECORD carries a record, HOLE names an index the device would not produce,
# and DONE says the range is over - which is a frame and not a silence, so a
# reader can tell a finished log from a link that died.
LOG_STREAM_RECORD = 0
LOG_STREAM_HOLE = 1
LOG_STREAM_DONE = 2


def start_log_stream(client, source, start, count, hz):
    """Asks for a range of a log to be pushed, and returns what will be sent as
    (source, first, count, hz) - or None when the device has no such log.

    The four numbers are the firmware's answer about what it *will* send and
    not an echo of the request: a count running past the end of the ring comes
    back smaller, and a rate past AK_PROTO_LOG_STREAM_MAX_HZ comes back lower.
    A rate of 0 in the answer means nothing is coming, and that one answer
    covers three different reasons - an empty range, a rate of zero, and a link
    that cannot push frames at all. It is TELEMETRY's rule and it is deliberate
    in both places: the client's job is to notice no stream is coming, not to
    be told which of three sentences to print.

    `request_while_streaming` rather than `request`, because a stream already
    running will have frames in flight and one of them can arrive between this
    request and its reply - which is exactly the race that made stopping a
    telemetry stream fail once."""
    payload = client.request_while_streaming(
        LOG_STREAM,
        bytes([source]) + start.to_bytes(2, "little") +
        count.to_bytes(2, "little") + bytes([hz]))
    if payload[0] != 0:
        return None
    return (payload[1],
            int.from_bytes(payload[2:4], "little"),
            int.from_bytes(payload[4:6], "little"),
            payload[6])


def next_log_frame(client):
    """Blocks until a pushed log frame arrives, and returns (status, source,
    index, record) with the record None for a hole or for the end.

    The index comes off the frame rather than from a counter here. A client
    that counted frames would report the records after a hole against the wrong
    timestamps, and would do it silently - which is worse than reporting the
    hole."""
    while True:
        command, payload = client.read_frame()
        if command != LOG_STREAM:
            continue
        status = payload[0]
        record = (parse_log_record(payload, 4)
                  if status == LOG_STREAM_RECORD else None)
        return (status, payload[1],
                int.from_bytes(payload[2:4], "little"), record)


def fetch_log_streamed(client, write, source=0, chunk=256, hz=LOG_STREAM_MAX_HZ):
    """Pulls a whole log by asking the device to push it, rather than asking for
    one record at a time.

    It writes the same CSV the round-trip version does, so a log is a log
    whichever way it left the aircraft. What it does not do is hide a hole: a
    record the device refused is written as a line naming the index and nothing
    else, because a reader that dropped it would silently shift every record
    after it - and the records after the gap are the ones somebody is reading
    the log for."""
    total = select_log(client, source)
    if total is None:
        raise IOError("this device has no %s log"
                      % LOG_SOURCE_NAMES.get(source, "source %u" % source))
    name = LOG_SOURCE_NAMES.get(source, "source %u" % source)
    write("# aerialkit blackbox (%s), pushed over the config protocol\n"
          % name
          + LOG_UNITS + ",".join(LOG_FIELDS) + "\n")

    got = 0
    holes = 0
    for first in range(0, total, chunk):
        count = min(chunk, total - first)
        answer = start_log_stream(client, source, first, count, hz)
        if answer is None:
            raise IOError("the device refused to stream its %s log" % name)
        _, _, sent, rate = answer
        if rate == 0 or sent == 0:
            raise IOError("the device accepted no stream for the %s log "
                          "(asked for %u records at %u Hz)"
                          % (name, count, hz))
        while True:
            status, _, index, record = next_log_frame(client)
            if status == LOG_STREAM_DONE:
                break
            if status == LOG_STREAM_HOLE:
                holes += 1
                write("# hole at %u\n" % index)
                continue
            write(",".join(str(v) for v in record) + "\n")
            got += 1
    return got, holes


def c_string(data, offset):
    end = data.index(b"\0", offset)
    return data[offset:end].decode()


# The parameter types, from ak_param_type_t. The numbers are the wire's, and
# they are named here so that a caller reading an entry says "text" rather than
# 2 - the same reason the group names are spelled out below.
PARAM_FLOAT = 0
PARAM_U32 = 1
PARAM_TEXT = 2

# ak_param_group_t, from ak_params.h. A client carries a copy of this the way it
# carries a copy of the command numbers: a number it does not know renders as
# "unknown", never as a neighbour's name.
PARAM_GROUPS = {
    0: "none",
    1: "rates",
    2: "angle",
    3: "arming",
    4: "receiver",
    5: "airframe",
    6: "outputs",
    7: "power",
    8: "failsafe",
    9: "navigation",
    10: "sensors",
    11: "network",
    12: "timing",
}

# What a `param info` reply can say. Status 2 is the one that needs explaining:
# it is not "the table ended" and not a malformed request - it is the row at
# `first` not fitting in one frame at any size. A client that read it as the end
# would stop early and never learn the row exists.
INFO_OK = 0
INFO_NO_INDEX = 1
INFO_TOO_BIG = 2
INFO_STATUS = {
    INFO_OK: "ok",
    INFO_NO_INDEX: "the request named no index",
    INFO_TOO_BIG: "the entry does not fit one frame",
}

# How an entry is laid out, and how many bytes of header the page itself has.
# Both are the wire's, from docs/16-protocol.md.
INFO_HEADER = 3


def parse_param_entry(payload, at):
    """One entry, and where the next one starts.

    The name first, then the four one-byte fields, and then - told apart by the
    type the entry itself stated - either two spelled bounds or a length. The
    order matters: it is what lets a client read a text parameter without
    guessing from the name whether it has a range."""
    name = c_string(payload, at)
    at += len(name) + 1
    kind, group, decimals, flags = payload[at:at + 4]
    at += 4

    entry = {"name": name, "type": kind, "group": group, "group_name":
             PARAM_GROUPS.get(group, "unknown"), "decimals": decimals,
             "flags": flags, "secret": bool(flags & 0x01)}

    if kind == PARAM_TEXT:
        entry["max_len"] = payload[at]
        at += 1
        entry["min"] = None
        entry["max"] = None
    else:
        entry["min"] = c_string(payload, at)
        at += len(entry["min"]) + 1
        entry["max"] = c_string(payload, at)
        at += len(entry["max"]) + 1
        entry["max_len"] = None

    entry["default"] = c_string(payload, at)
    at += len(entry["default"]) + 1
    return entry, at


def fetch_param_info(client, first=0, limit=None):
    """The whole table's description of itself, by index.

    Paged, so this walks: it asks from an index, takes what came back, and asks
    from where that left off. The loop ends when the board says the page is
    empty - `carried == 0` with status 0.

    Status 2 is the other answer, and folding it into "the end" is the bug this
    function exists to not have: it means the row at this index exists but does
    not fit one frame at any size. The row is recorded with no name and a reason
    - a hole a caller can see - and the walk steps over it, because a client
    that stopped there would show ninety-one parameters and call that the
    table."""
    entries = []
    index = first
    while limit is None or len(entries) < limit:
        payload = client.request(PARAM_INFO, bytes([index]))
        status = payload[0]
        if payload[1] != (index & 0xFF):
            # The page has to say which index it is about, and the client has to
            # read that rather than assume it. A page built from the wrong offset
            # would otherwise be joined onto the previous one and produce a table
            # that is right about every parameter and wrong about every index -
            # which is the shape of mistake this walk exists to not make.
            raise IOError("param info: asked from %u and the page says %u"
                          % (index, payload[1]))
        if status == INFO_TOO_BIG:
            entries.append({"index": index, "name": None,
                            "unavailable": INFO_STATUS[status]})
            index += 1
            continue
        if status != INFO_OK:
            raise IOError("param info at %u: %s"
                          % (index, INFO_STATUS.get(status, status)))
        carried = payload[2]
        if carried == 0:
            break
        at = INFO_HEADER
        for _ in range(carried):
            entry, at = parse_param_entry(payload, at)
            entry["index"] = index
            entries.append(entry)
            index += 1
    return entries


def fetch_param_help(client, index):
    """One row's prose, walked by offset until the whole thing has arrived.

    Walked rather than paged, and that is what makes a help string longer than a
    frame an ordinary case: the client is done when it has `total` bytes and
    there is no path here that loses the tail quietly."""
    parts = []
    offset = 0
    total = None
    while total is None or offset < total:
        payload = client.request(PARAM_HELP,
                                 bytes([index]) + offset.to_bytes(2, "little"))
        if payload[0] != 0:
            raise IOError("param help %u at %u: refused" % (index, offset))
        answered = int.from_bytes(payload[2:4], "little")
        if answered != offset:
            raise IOError("param help %u: asked from %u and got %u"
                          % (index, offset, answered))
        total = int.from_bytes(payload[4:6], "little")
        length = payload[6]
        parts.append(payload[7:7 + length])
        if length == 0 and offset < total:
            raise IOError("param help %u: stopped at %u of %u with nothing to "
                          "send" % (index, offset, total))
        offset += length
    return b"".join(parts).decode()


# The status byte on PREFLIGHT, which is ak_proto.h's enum.
PREFLIGHT_OK = 0
PREFLIGHT_NO_INDEX = 1
PREFLIGHT_NONE = 2
# The verdict on a line, which is what the console's left-hand column means.
# The markers are the console's own five-and-a-bit columns, so a `preflight`
# here and a `preflight` typed at the console are the same eight hundred bytes.
PREFLIGHT_VERDICTS = {0: ("FAIL", "FAIL  "), 1: ("pass", "ok    "),
                      2: ("fact", "--    ")}


def parse_preflight_page(payload):
    """One page of one line: `status, index, count, verdict, name, total, part`.

    Three whole-payload replies and one real page. The two non-OK ones are kept
    apart because from a client's side they are different facts: `NONE` means
    this board has no checklist to serve - a build without one, or a board that
    is not an aircraft - and `NO_INDEX` means the walk went off the end of a
    checklist that exists. One is "there is nothing here" and the other is "you
    asked for line nine and there are six", and a client that showed the second
    as the first would hide a bug in its own loop.

    `0x7F` is checked first, before either: it is not a member of this status
    enum, and a build predating the command answers it to every request."""
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "preflight: the board answered 0x%02x, the status this firmware "
            "gives a command it does not implement - this build predates "
            "`preflight`" % UNKNOWN_COMMAND)
    if raw == PREFLIGHT_NONE:
        if len(payload) != 1:
            raise IOError("preflight: status 2 says this board has no checklist "
                          "and arrived with %u bytes after it" % (len(payload) - 1))
        return {"status": raw, "index": None, "count": 0, "verdict": None,
                "name": None, "total": 0, "part": ""}
    if raw == PREFLIGHT_NO_INDEX:
        # The index and the count are both carried, and the index is the one
        # that was asked for rather than a failure to report it.
        return {"status": raw, "index": payload[1], "count": payload[2],
                "verdict": None, "name": None, "total": 0, "part": ""}
    if raw != PREFLIGHT_OK:
        raise IOError("preflight: the board answered with status %u" % raw)

    name_len = payload[4]
    if len(payload) < 5 + name_len + 3:
        raise IOError("preflight: a name of %u bytes arrived in a reply too "
                      "short to hold it" % name_len)
    name = payload[5:5 + name_len].decode()
    body = 5 + name_len
    total = int.from_bytes(payload[body:body + 2], "little")
    length = payload[body + 2]
    if len(payload) < body + 3 + length:
        raise IOError("preflight: line %u said %u bytes of sentence and %u "
                      "arrived" % (payload[1], length, len(payload) - body - 3))
    return {"status": raw, "index": payload[1], "count": payload[2],
            "verdict": payload[3], "name": name, "total": total,
            "part": payload[body + 3:body + 3 + length].decode()}


def fetch_preflight(client):
    """The board's own checklist, every line and every byte of every sentence.

    Two loops, and both of them raise rather than return what they have. The
    outer one walks the lines until the board says the index is past the end;
    the inner one walks one line's sentence by offset until `total` bytes have
    arrived. `fetch_param_help` is the same inner loop, one level down, and it
    exists for the same reason: the console prints a fault and its cause on one
    line, and a cause longer than a frame is cut here or walked here.

    The rebuild rule is the firmware's - index zero rebuilds, every other index
    reads what index zero built - and this walk depends on it. A checklist that
    changed shape between two pages would be read here as a line appearing or
    vanishing, which is why the count is asserted on every page rather than
    taken from the first."""
    lines = []
    count = None
    index = 0
    while True:
        page = parse_preflight_page(
            client.request(PREFLIGHT, bytes([index]) + b"\x00\x00"))
        if page["status"] == PREFLIGHT_NONE:
            if index != 0:
                raise IOError("preflight: line %u of a checklist the board had "
                              "already started serving" % index)
            return []
        if page["status"] == PREFLIGHT_NO_INDEX:
            if page["index"] != index:
                raise IOError("preflight: asked for line %u and the refusal "
                              "named %u" % (index, page["index"]))
            if count is not None and page["count"] != count:
                raise IOError("preflight: the board had %u lines and now says "
                              "%u" % (count, page["count"]))
            return lines

        if count is None:
            count = page["count"]
        elif page["count"] != count:
            # The checklist changed shape under the walk. Every line read so far
            # is still true and the list as a whole is not, so this is raised
            # rather than returned - a caller that printed eight of twelve lines
            # has shown a checklist that never existed.
            raise IOError("preflight: the board had %u lines at the first page "
                          "and %u at line %u" % (count, page["count"], index))

        parts = [page["part"]]
        got = len(page["part"])
        while got < page["total"]:
            more = parse_preflight_page(
                client.request(PREFLIGHT,
                               bytes([index]) + got.to_bytes(2, "little")))
            if more["status"] != PREFLIGHT_OK:
                raise IOError("preflight: line %u stopped part way with status "
                              "%u" % (index, more["status"]))
            if more["total"] != page["total"]:
                raise IOError("preflight: line %u's sentence was %u bytes and "
                              "then %u" % (index, page["total"], more["total"]))
            if not more["part"]:
                raise IOError("preflight: line %u stopped at %u of %u with "
                              "nothing to send" % (index, got, page["total"]))
            parts.append(more["part"])
            got += len(more["part"])
        lines.append((page["name"], page["verdict"], "".join(parts)))
        index += 1


# The five verbs on MISSION, which are ak_proto.h's defines, and the names a
# person reads. The wire value is what goes on the wire and the name is what a
# client shows; keeping both here means a verb added in the firmware and not
# here is a KeyError rather than a number in a menu.
MISSION_STATUS = 0
MISSION_START = 1
MISSION_STOP = 2
MISSION_HOME_SET = 3
MISSION_HOME_CLEAR = 4
MISSION_VERB_NAMES = {
    MISSION_STATUS: "status",
    MISSION_START: "start",
    MISSION_STOP: "stop",
    MISSION_HOME_SET: "home set",
    MISSION_HOME_CLEAR: "home clear",
}

# The status byte on MISSION, which is ak_proto.h's enum. Two of the four are
# refusals the *firmware* owns - an empty list and a home asked for with no
# usable fix - and they are different facts from each other and from the two
# the dispatch owns (a frame with no verb, a verb it does not know).
MISSION_OK = 0
MISSION_NO_VERB = 1
MISSION_NO_NAV = 2
MISSION_NO_WAYPOINTS = 3
MISSION_NO_FIX = 4

# Ak_proto.h's out-of-range index for "not flying a waypoint". A byte rather than
# 0xFF-in-a-u8 cast at each site, and deliberately not a valid index: "flying
# waypoint zero" and "not flying one" must not read the same.
MISSION_NO_INDEX = 0xFF


def parse_mission(payload):
    """The mission's whole state, from any of the five verbs.

    Seventeen bytes, one shape for all five, which is the design: a client that
    has read one reply has read them all, and the verb it sent only decides
    whether anything changed. `status` and `op` are checked here because they
    are the two bytes whose *absence* would turn a refusal into a reading: an
    OK with a stale op is a client that asked one thing and read another.

    `0x7F` is checked first, before the enum: this firmware gives it to a
    command it does not implement, and a build predating `mission` answers it
    to every request. Reporting it as the nearest of the five statuses would
    turn "this board cannot do missions" into "this board's list is empty"."""
    if len(payload) != 17:
        raise IOError("mission: the reply is %u bytes and every mission reply "
                      "is 17" % len(payload))
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "mission: the board answered 0x%02x, the status this firmware gives "
            "a command it does not implement - this build predates `mission`"
            % UNKNOWN_COMMAND)
    if raw not in (MISSION_OK, MISSION_NO_VERB, MISSION_NO_NAV,
                   MISSION_NO_WAYPOINTS, MISSION_NO_FIX):
        raise IOError("mission: the board answered with status %u, which is not "
                      "one of the five this firmware defines" % raw)

    # The op is echoed rather than corrected, and *both* out-of-range cases are
    # deliberate - `0xFF` is "the frame did not say which verb" and any other
    # number above four is "you asked for a verb this firmware does not have".
    # So a name is looked up rather than required: neither echo is an error
    # here, and a client that showed `0xFF` as "status" would be reporting a
    # question the caller never asked. `None` is that answer.
    op = payload[1]

    return {
        "status": raw,
        "op": op,
        "verb": MISSION_VERB_NAMES.get(op),
        "active": payload[2],
        "requested": payload[3],
        "count": payload[4],
        "index": payload[5],
        "channel": payload[6],
        "reached": int.from_bytes(payload[7:9], "little"),
        "started": int.from_bytes(payload[9:11], "little"),
        "cancelled": int.from_bytes(payload[11:13], "little"),
        "hold_alt_mm": int.from_bytes(payload[13:17], "little", signed=True),
    }


def fetch_mission(client, op):
    """Send one verb and read the whole state back. Raises on a refusal.

    `fetch_mission(MISSION_STATUS)` is the read; the other four are the
    commands, and each replies with the state *after* it ran, so a caller
    never has to send a second frame to find out what its own verb did."""
    state = parse_mission(client.request(MISSION, bytes([op])))
    if state["status"] == MISSION_NO_VERB:
        # The echo is in the message because it is the whole point of the echo:
        # `0xFF` is a caller that sent no verb and any other number is a caller
        # that sent one this board does not have, and those are different bugs.
        raise IOError("mission: the board refused verb 0x%02x (status 1)%s"
                      % (state["op"],
                         "" if state["op"] == 0xFF
                         else " - it does not have that one"))
    if state["status"] == MISSION_NO_NAV:
        raise IOError("mission: this board has no navigator")
    if state["status"] == MISSION_NO_WAYPOINTS:
        raise IOError("mission: there is nothing to fly - the list is empty")
    if state["status"] == MISSION_NO_FIX:
        raise IOError("mission: no usable position fix, so there is no home "
                      "to set")
    return state


# The status on PERF, from ak_proto.h. `OK` and `NONE` are the whole enum, and
# the second is not an error: it is a *build* with no profiler in it, which is
# a fact about the firmware rather than about the aircraft. The distinction is
# the reason the byte exists - a client that read a window of zeros as "the
# loop costs nothing" would be reporting the opposite of what the board said.
PERF_OK = 0
PERF_NONE = 1

# The five sections a PERF reply carries, in the order the firmware sends them,
# which is ak_perf.h's enum without AK_PERF_NONE - the state between sections,
# which is work that has not been divided up and so has no name on the wire.
PERF_SECTIONS = ("imu", "estimator", "pid", "mixer", "output")

# The fixed size of a PERF reply, from ak_proto.h's ak_proto_perf_t: one status
# byte, then 4+4+2+4+4+4+4+2+2+2+4 = 36 bytes of scalars, then two arrays of
# five u16 (10 + 10) and a final u16 - 59 in total. Asserted rather than
# assumed by `parse_perf`, because a reply of the wrong length is a frame read
# at the wrong offsets and every number below would be plausible nonsense.
PERF_REPLY_BYTES = 59


def parse_perf(payload):
    """The profiler's window, as PERF sends it.

    The units are the field names' and they are not the console's, which is the
    one place this parser has to be read rather than skimmed: `section_avg_x10`
    is in tenths of a microsecond because a nanosecond figure does not fit a u16
    for any section that matters, and a whole microsecond would hide the
    difference between a loop that is comfortable and one that is nearly out of
    slot. Everything else is whole microseconds except `load_permille`.

    `loops` and `samples` both come back, and neither is derived here. A port
    whose clock reads zero can still count a period closing, so `loops` keeps
    climbing while `samples` - the periods it could actually *time* - stands
    still, and a caller that divided one stream of numbers by the other count
    would fabricate a plausible answer. Reporting both is what makes the
    difference visible instead."""
    if len(payload) != PERF_REPLY_BYTES:
        raise IOError("perf: the reply is %u bytes and every perf reply is %u"
                      % (len(payload), PERF_REPLY_BYTES))
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "perf: the board answered 0x%02x, the status this firmware gives a "
            "command it does not implement - this build predates `perf`"
            % UNKNOWN_COMMAND)
    if raw not in (PERF_OK, PERF_NONE):
        raise IOError("perf: the board answered with status %u, which is not "
                      "one of the two this firmware defines" % raw)

    at = 1

    def u16():
        nonlocal at
        value = int.from_bytes(payload[at:at + 2], "little")
        at += 2
        return value

    def u32():
        nonlocal at
        value = int.from_bytes(payload[at:at + 4], "little")
        at += 4
        return value

    window = {
        "status": raw,
        "loops": u32(),
        "samples": u32(),
        "nominal_us": u16(),
        "period_last_us": u32(),
        "period_min_us": u32(),
        "period_max_us": u32(),
        "late": u32(),
        "jitter_p50_us": u16(),
        "jitter_p99_us": u16(),
        "jitter_max_us": u16(),
        "jitter_over": u32(),
    }
    window["section_avg_x10"] = [u16() for _ in PERF_SECTIONS]
    window["section_max_us"] = [u16() for _ in PERF_SECTIONS]
    window["load_permille"] = u16()
    if at != PERF_REPLY_BYTES:
        # Unreachable while the offsets above and PERF_REPLY_BYTES agree, which
        # is exactly why it is stated: the constant is the check that catches a
        # field added to the firmware's struct and not to this walk.
        raise IOError("perf: read %u of %u bytes" % (at, PERF_REPLY_BYTES))
    return window


# The status on MOTOR_TELEMETRY, and the two bounds ak_proto.h puts on a reply.
# `NONE` is the same refusal RC_CHANNELS makes about a missing receiver port,
# about a missing telemetry path: one byte and stop. It is *not* an error and it
# is *not* four stopped motors - a frame of zeros here would say the board heard
# four ESCs report zero, which is a different claim about different hardware.
MOTOR_OK = 0
MOTOR_NONE = 1

# The wire's bound on how many motors a reply can carry, from ak_proto.h. It is
# a bound on the wire's side rather than a copy of AK_MAX_MOTORS, so a board
# that grew a motor keeps this client working - and a client that needs to know
# whether it was shown all of them asks OUTPUT_INFO, whose count is the board's
# own rather than this constant.
MOTOR_MAX = 4

# One entry's size, from ak_proto.h's ak_proto_motor_t: flags (1), eRPM and rpm
# (4 each), temperature and its session maximum (1 each), millivolts and
# milliamps (2 each), and the quality window's packets and invalid (2 each) -
# 19 bytes. Asserted by `parse_motor_telemetry` rather than assumed, because a
# reply read at the wrong stride is a list of plausible numbers from the wrong
# motors.
MOTOR_ENTRY_BYTES = 19

# Facts about one motor as bits, in AK_PROTO_MOTOR_FLAG_* order. Every one says
# a measurement happened; none is a verdict, and the names are the firmware's so
# that no translation can go wrong between the two.
MOTOR_FLAGS = {
    0: "measured",
    1: "rpm",
    2: "temperature",
    3: "voltage",
    4: "current",
}


def parse_motor_telemetry(payload):
    """`motor telemetry` as a dict, with the same three look-alikes RC_CHANNELS
    keeps apart, because they collapse the same way on a screen:

      - `status = MOTOR_NONE`, one byte and nothing else: this build has no way
        to hear an ESC. Every field below comes back empty rather than zero.
      - status ok, count motors, every `measured` flag clear: a board with the
        path that has heard nothing yet. Four motors, none of them reporting,
        which is a different sentence from "four motors stopped" and from "no
        telemetry path".
      - status ok with `measured` set: the ESC answered. A motor that is
        genuinely stopped reports eRPM 0 with `measured` set, which is the only
        one of the three that is a reading of a motor at rest.

    `rpm` is present only when its own flag is set, which is only when the board
    was told the motor's pole count - carried in `poles`, and zero on every
    board in this tree today. An eRPM with no pole count is a true reading of
    something that is not a speed, and this parser does not divide it by a
    guessed fourteen to make it look like one."""
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "motor telemetry: the board answered 0x%02x, the status this "
            "firmware gives a command it does not implement - this build "
            "predates `motor telemetry`" % UNKNOWN_COMMAND)

    status = raw
    if status == MOTOR_NONE:
        if len(payload) != 1:
            raise IOError("motor telemetry: a board with no telemetry path "
                          "answers one byte, and this reply carried %u"
                          % len(payload))
        return {"status": status, "status_name": "no motor telemetry",
                "count": 0, "poles": 0, "motors": []}
    if status != MOTOR_OK:
        raise IOError("motor telemetry: the board answered with status %u, "
                      "which is not one of the two this firmware defines"
                      % status)

    count = payload[1]
    if count > MOTOR_MAX:
        raise IOError("motor telemetry: the reply says %u motors, and the "
                      "protocol allows at most %u" % (count, MOTOR_MAX))
    expected = 3 + count * MOTOR_ENTRY_BYTES
    if len(payload) != expected:
        raise IOError("motor telemetry: %u motors is %u bytes and this reply "
                      "carried %u" % (count, expected, len(payload)))

    poles = payload[2]
    at = 3
    motors = []
    for _ in range(count):
        entry = payload[at:at + MOTOR_ENTRY_BYTES]
        flags = entry[0]
        motor = {
            "flags": flags,
            "flag_names": _named_bits(flags, MOTOR_FLAGS),
            "measured": bool(flags & (1 << 0)),
            "erpm": int.from_bytes(entry[1:5], "little"),
            "rpm": int.from_bytes(entry[5:9], "little")
                   if flags & (1 << 1) else None,
            "temperature": entry[9] if flags & (1 << 2) else None,
            "max_temperature": entry[10] if flags & (1 << 2) else None,
            "millivolts": int.from_bytes(entry[11:13], "little")
                          if flags & (1 << 3) else None,
            "milliamps": int.from_bytes(entry[13:15], "little")
                         if flags & (1 << 4) else None,
            "packets": int.from_bytes(entry[15:17], "little"),
            "invalid": int.from_bytes(entry[17:19], "little"),
        }
        motors.append(motor)
        at += MOTOR_ENTRY_BYTES

    return {"status": status, "status_name": "ok", "count": count,
            "poles": poles, "motors": motors}


# The verbs and the statuses on CALIBRATE, from ak_proto.h. Six verbs and ten
# statuses, and the tenth is the one worth having: `NOTHING` is a fact about the
# build and `NO_SAMPLES` is a fact about the aircraft, and a client that drew
# them the same way would send somebody to look at a connector that was never
# fitted.
CALIBRATE_STATUS = 0
CALIBRATE_GYRO = 1
CALIBRATE_RC = 2
CALIBRATE_ACCEL = 3
CALIBRATE_VBAT = 4
CALIBRATE_ABORT = 5

CALIBRATE_VERB_NAMES = {
    CALIBRATE_STATUS: "status",
    CALIBRATE_GYRO: "gyro",
    CALIBRATE_RC: "rc",
    CALIBRATE_ACCEL: "accel",
    CALIBRATE_VBAT: "vbat",
    CALIBRATE_ABORT: "abort",
}

CALIBRATE_OK = 0
CALIBRATE_NO_VERB = 1
CALIBRATE_ARMED = 2
CALIBRATE_BUSY = 3
CALIBRATE_NOTHING = 4
CALIBRATE_NO_SAMPLES = 5
CALIBRATE_IMPLAUSIBLE = 6
CALIBRATE_IDLE = 7
CALIBRATE_NO_FACE = 8
CALIBRATE_BAD_VALUE = 9

# The sentence each status renders as. **This table and the one in
# `docs/16-protocol.md` are the two records of it, and they are the same
# sentences character for character** - a fourth copy lives in the web
# configurator's `CALIBRATE_STATUS_TEXT`, and `apps/configurator`'s
# `tests/calibration.test.ts` reads both of these and fails if any of the three
# disagrees. That test exists because they had already drifted: `ok` was `done`
# in the console and the document, and `no such verb` was missing the `refused:`
# prefix every other refusal in this table carries.
CALIBRATE_STATUS_NAMES = {
    CALIBRATE_OK: "done",
    CALIBRATE_NO_VERB: "refused: no such verb",
    CALIBRATE_ARMED: "refused: the aircraft is armed",
    CALIBRATE_BUSY: "refused: a session is already running",
    CALIBRATE_NOTHING: "refused: nothing on this board to calibrate",
    CALIBRATE_NO_SAMPLES: "ran, and did not get enough still samples",
    CALIBRATE_IMPLAUSIBLE: "ran, and the measurement is not one this aircraft "
                           "will accept",
    CALIBRATE_IDLE: "refused: nothing to abort",
    CALIBRATE_NO_FACE: "refused: no such accelerometer face",
    CALIBRATE_BAD_VALUE: "refused: the voltage given is not a pack this "
                         "aircraft flies",
}

# The step byte when no face is being sampled, out of range for all six, so
# "between faces" cannot be read as "sampling face zero".
CALIBRATE_NO_STEP = 0xFF

# The verb byte when there is no session to name. Out of range for all six, and
# it is not `status`: the verb is what says how to read the six result slots, so
# a board that has never calibrated must not answer "these are the status verb's
# numbers" beside six zeros.
CALIBRATE_NO_SESSION = 0xFF

# How many faces the accelerometer flow measures, and what the `faces` bitmask
# reads when all of them are in. The mask is the flow's whole progress, so a
# wizard's sixth and final command is the one whose reply carries every bit.
CALIBRATE_FACES = 6
CALIBRATE_ALL_FACES = (1 << CALIBRATE_FACES) - 1

# The result slots, by verb, in the units ak_proto.h fixes them in. The slot
# count is the accelerometer's six; the other three use a prefix and the rest
# are a real zero rather than a placeholder.
CALIBRATE_RESULT_MEANING = {
    CALIBRATE_GYRO: ["bias roll (mdps)", "bias pitch (mdps)", "bias yaw (mdps)"],
    CALIBRATE_RC: ["centre (us)", "roll off centre (us)", "pitch off centre (us)",
                   "yaw off centre (us)"],
    CALIBRATE_VBAT: ["ratio (x1e6)"],
    CALIBRATE_ACCEL: ["bias x (ug)", "bias y (ug)", "bias z (ug)",
                      "scale x (x1e6)", "scale y (x1e6)", "scale z (x1e6)"],
}


def parse_calibration(payload, sent=None):
    """The calibration session's whole state, from any of the six verbs.

    Thirty-seven bytes, one shape for all six, which is the design: the reply to
    `calibrate gyro` is not an acknowledgement but a reading of the session that
    verb started, so a client never has to send a second frame to find out what
    its own command did - and a wizard can show progress from the same reply it
    started with.

    The verb byte is the *session's*, not an echo of the request: a `status`
    poll of a running gyro calibration reports `gyro`, because that byte is what
    says how to read the six result slots. `sent` is therefore checkable only
    for the four verbs that start a session and name themselves in it, and is
    how a caller catches an OK carrying a stale verb - `mission`'s check.
    A board that has never calibrated answers `0xFF` for it, so nothing is read
    as the `status` verb."""
    if len(payload) != 37:
        raise IOError("calibrate: the reply is %u bytes and every calibration "
                      "reply is 37" % len(payload))
    raw = payload[0]
    if raw == UNKNOWN_COMMAND:
        raise IOError(
            "calibrate: the board answered 0x%02x, the status this firmware "
            "gives a command it does not implement - this build predates "
            "`calibrate`" % UNKNOWN_COMMAND)
    if raw not in CALIBRATE_STATUS_NAMES:
        raise IOError("calibrate: the board answered with status %u, which is "
                      "not one of the ten this firmware defines" % raw)
    # Checked only on a successful reading. A *refusal* is still owed the state
    # of the session already running - that is this opcode's whole shape - so
    # the verb beside it is that session's and not the caller's, and a board
    # answering exactly as documented would fail an unconditional check. A
    # refusal's status is the thing to act on; this catches an OK that reports
    # some other session.
    if sent is not None and raw == CALIBRATE_OK and payload[1] != sent:
        raise IOError("calibrate: asked for verb %u and the session reported is "
                      "%u, so this reply is not the one the request started"
                      % (sent, payload[1]))

    verb = payload[1]
    results = [int.from_bytes(payload[13 + 4 * i:17 + 4 * i], "little",
                              signed=True) for i in range(6)]
    return {
        "status": raw,
        "status_name": CALIBRATE_STATUS_NAMES[raw],
        "verb": verb,
        "verb_name": CALIBRATE_VERB_NAMES.get(verb),
        "active": payload[2],
        "step": payload[3],
        "faces": payload[4],
        "samples": int.from_bytes(payload[5:9], "little"),
        "rejected": int.from_bytes(payload[9:13], "little"),
        "results": results,
        # The slots the verb actually fills, so a caller printing all six of a
        # gyro's does not report three zeroes that look like measurements.
        "result_names": CALIBRATE_RESULT_MEANING.get(verb, []),
    }


def calibration_state(client):
    """Poll the running calibration. Never raises on a refusal, because a
    refusal here is the reading: the session ended and the status is *why*.

    `STATUS` answers with the outcome of the last session rather than with an
    echo of its own number, so this is also how a caller finds out how a
    `gyro` or `rc` it started three seconds ago finished."""
    return parse_calibration(client.request(CALIBRATE, bytes([CALIBRATE_STATUS])))


def calibrate(client, verb, face=0, mv=0):
    """Start one of the four calibrations and read the session back, raising on
    a refusal. `GYRO` and `RC` return with the session *running*, because the
    firmware will not hold its flight loop for three seconds - poll
    `calibration_state` until `active` is zero. `ACCEL` starts one face and is
    sent once per face; `VBAT` finishes inside the request, because the only
    thing it waits for is the ADC.

    The refusals are raised rather than returned because they are answers no
    caller can proceed from. A polling wizard that expects to see `BUSY` while
    another session runs should catch, or poll `calibration_state` instead."""
    args = bytes([verb])
    if verb == CALIBRATE_ACCEL:
        args += bytes([face])
    elif verb == CALIBRATE_VBAT:
        args += struct.pack("<I", mv)

    # The echo is checkable only for the four verbs that write their own number
    # into the session's verb byte. STATUS and ABORT report the session they are
    # *about* - that is the whole point of the reply being a reading and not an
    # acknowledgement - so a board answering correctly would fail an echo check
    # on those two.
    echo = verb if verb in (CALIBRATE_GYRO, CALIBRATE_RC, CALIBRATE_ACCEL,
                            CALIBRATE_VBAT) else None
    state = parse_calibration(client.request(CALIBRATE, args), sent=echo)
    if state["status"] != CALIBRATE_OK:
        # Named by the verb that was *asked for*, not by the one the reply
        # reports: a refusal is owed the state of whatever session is running,
        # so the reply's verb is that session's and would label a refusal about
        # `accel` with the `vbat` that happened to run before it.
        raise IOError("calibrate %s: %s (status %u)"
                      % (CALIBRATE_VERB_NAMES.get(verb, "0x%02x" % verb),
                         state["status_name"], state["status"]))
    return state


def open_client(args):
    if args.host:
        import socket
        host, _, port = args.host.rpartition(":")
        connection = socket.create_connection((host or "127.0.0.1", int(port)), 5)
        connection.settimeout(5)
        return Client(connection.recv, connection.sendall)

    if args.port:
        import serial  # only needed for the serial case
        port = serial.Serial(args.port, 115200, timeout=2)
        import atexit
        atexit.register(leave_port_as_found, port)
        return Client(port.read, port.write)

    # Flushed on every write: a buffered writer holds the request and the other
    # end waits for it, which is a deadlock rather than a slow test.
    def write(data):
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()

    return Client(sys.stdin.buffer.read, write)


def leave_port_as_found(port):
    """Put the tty back to VMIN=1, VTIME=0 before the port is let go.

    pyserial sets VMIN=0 for its timed reads and does not undo it, and a tty
    keeps its termios after close. Chrome's Web Serial does not set VMIN when it
    opens a port, so it inherits the 0: its second read returns zero bytes, which
    it reports as "the device has been lost", and the configurator then sees a
    silent board. Measured on 2026-10-01 (strace of Chrome on the Y520). A no-op
    where there is no termios.
    """
    try:
        import termios
        attrs = termios.tcgetattr(port.fd)
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(port.fd, termios.TCSANOW, attrs)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port to talk over")
    parser.add_argument("--host", help="HOST:PORT to talk to over the network")
    parser.add_argument("command",
                        choices=["hello", "status", "list", "get", "set", "save",
                                 "log", "telemetry", "info", "help", "rc",
                                 "motors",
                                 "preflight", "mission", "calibrate"])
    parser.add_argument("--source", choices=sorted(LOG_SOURCES),
                        default="fast",
                        help="which log `log` pulls: fast (RAM), long (retained "
                             "RAM), flash (survives the battery)")
    parser.add_argument("--pull", action="store_true",
                        help="read `log` one record per round trip. The default "
                             "asks the device to push the range instead, which "
                             "is what makes the 5 460-record flash log "
                             "practical; --pull is the old way and the way that "
                             "works against firmware predating LOG_STREAM")
    parser.add_argument("args", nargs="*")
    args = parser.parse_args()

    client = open_client(args)

    if args.command == "hello":
        hello = parse_hello(client.request(HELLO))
        print("protocol %(version)u, %(product)s, %(count)u parameters, "
              "%(changed)s changed since saved" % hello)
        # Three answers, and they are three different facts: this board cannot
        # say what it supports, this board supports none of the optional
        # commands, or here are the ones it does. Collapsing the first two into
        # "none" is what the whole field exists to prevent.
        if hello["features"] is None:
            print("capabilities: this firmware answered without a capability "
                  "word, so it cannot say")
        else:
            names = feature_names(hello["features"])
            print("capabilities: 0x%08x - %s"
                  % (hello["features"], ", ".join(names) if names else "none"))
            print("config hash: 0x%08x" % hello["config_hash"])

    elif args.command == "status":
        status = parse_status(client.request(STATUS))
        print("state %(flight_state)u, link %(link_live)u, fix %(gps_fix_type)u "
              "with %(gps_satellites)u satellites" % status)
        print("attitude roll %(roll_deg).1f, pitch %(pitch_deg).1f, "
              "yaw %(yaw_deg).1f" % status)
        print("position %(lat).7f, %(lon).7f" % status)
        print("motors %s" % ",".join(str(m) for m in status["motors"]))

    elif args.command == "rc":
        # What the receiver is doing, which is the console's `rc` over the wire.
        # This is the command to run with a handset in your hands: it is the
        # bench check on whether RC_CHANNELS agrees with the sticks, and the
        # only place the three zero-looking answers below are told apart.
        try:
            rc = parse_rc_channels(client.request(RC_CHANNELS))
        except IOError as problem:
            print("akproto: %s" % problem, file=sys.stderr)
            return 1
        if rc["status"] == RC_NONE:
            # Nothing after this line is true of this board, so nothing is
            # printed: a column of zeroes would read as a dead link.
            print("rc: this board has no receiver input")
            return 0
        print("rc: %s, %u channel(s), flags %s%s"
              % (rc["protocol_name"] or "protocol %u" % rc["protocol"],
                 rc["count"], ", ".join(rc["flag_names"]) or "none",
                 "  switches " + ", ".join(rc["switch_names"])
                 if rc["switch_names"] else ""))
        print("      " + "  ".join("%u" % c for c in rc["channels"]))
        if "decoded" in rc["flag_names"]:
            print("      sticks roll %i pitch %i yaw %i throttle %i (per-mille)"
                  % tuple(rc["sticks"]))
        else:
            # The counts are real and the sticks are not. Saying "0 0 0 0" here
            # would be reporting a centred handset for a receiver that has
            # never framed.
            print("      sticks: no frame decoded, so there is nothing to show")
        print("      bytes %u frames %u crc %u rejected %u lost %u "
              "failsafe %u dropped %u"
              % (rc["bytes"], rc["frames"], rc["crc_errors"], rc["rejected"],
                 rc["lost"], rc["failsafe_frames"], rc["dropped"]))

    elif args.command == "motors":
        # How fast each motor is turning, which is the console's `dshot` over
        # the wire - except that the console decodes a bit stream you paste into
        # it and this asks the board what it heard. The three answers that look
        # alike on a screen are told apart here for the same reason `rc` tells
        # its three apart.
        try:
            telemetry = parse_motor_telemetry(client.request(MOTOR_TELEMETRY))
        except IOError as problem:
            print("akproto: %s" % problem, file=sys.stderr)
            return 1
        if telemetry["status"] == MOTOR_NONE:
            print("motors: this board has no motor telemetry path")
            return 0
        if telemetry["poles"] == 0:
            # Said once, above the table, rather than repeated per row: it is one
            # fact about the board and it decides whether every rpm below is a
            # number. The console's `dshot` refuses the same way, and for the
            # same reason - fourteen poles is a guess, and a guess printed
            # beside a measurement cannot be told from one.
            print("motors: the board does not know the pole count, so no rpm "
                  "is reported - only eRPM, which needs no poles")
        for index, motor in enumerate(telemetry["motors"]):
            if not motor["measured"]:
                # Not "0 rpm": nothing has been heard from this ESC, which is a
                # different thing from an ESC that reported a stop.
                print("motor %u: no telemetry heard" % index)
                continue
            speed = ("%u rpm" % motor["rpm"]) if motor["rpm"] is not None \
                else "%u eRPM" % motor["erpm"]
            extra = []
            if motor["temperature"] is not None:
                extra.append("%u C (max %u)"
                             % (motor["temperature"], motor["max_temperature"]))
            if motor["millivolts"] is not None:
                extra.append("%u mV" % motor["millivolts"])
            if motor["milliamps"] is not None:
                extra.append("%u mA" % motor["milliamps"])
            print("motor %u: %s%s  packets %u invalid %u"
                  % (index, speed,
                     ("  " + ", ".join(extra)) if extra else "",
                     motor["packets"], motor["invalid"]))

    elif args.command == "preflight":
        # The console's `preflight`, over the wire. The sentences come from the
        # board, so this is the same checklist the console printed at boot -
        # except that boot's copy is as old as the boot and this one is taken
        # now, which is the whole reason to run it here.
        try:
            lines = fetch_preflight(client)
        except IOError as problem:
            print("akproto: %s" % problem, file=sys.stderr)
            return 1
        if not lines:
            print("preflight: this board has no checklist")
            return 0
        for _name, verdict, detail in lines:
            print("%s%s" % (PREFLIGHT_VERDICTS.get(verdict, ("?", "?     "))[1],
                            detail))
        problems = [line for line in lines if line[1] == 0]
        print("preflight: %s"
              % ("the machine is what the firmware thinks it is" if not problems
                 else "SOMETHING IS WRONG - read the FAIL lines above"))
        return 0 if not problems else 1

    elif args.command == "mission":
        # The console's `mission` over the wire, verb for verb. `mission` with
        # no argument is the read, and it is the same state the console prints
        # with ordinary words: this prints the numbers the wire carries, which
        # is what a configurator has to render without a console's prose.
        verbs = {"status": MISSION_STATUS, "start": MISSION_START,
                 "stop": MISSION_STOP, "home-set": MISSION_HOME_SET,
                 "home-clear": MISSION_HOME_CLEAR}
        what = args.args[0] if args.args else "status"
        if what not in verbs:
            print("akproto: mission takes one of %s"
                  % ", ".join(sorted(verbs)), file=sys.stderr)
            return 1
        try:
            state = fetch_mission(client, verbs[what])
        except IOError as problem:
            print("akproto: %s" % problem, file=sys.stderr)
            return 1
        # `index` of 0xFF is "not flying a waypoint", and it must not print as
        # 255 or as 0: those are both waypoints a person could believe in.
        flying_at = ("-" if state["index"] == MISSION_NO_INDEX
                     else str(state["index"]))
        print("mission: %s, %u waypoint(s), %s, flying %s"
              % (state["verb"], state["count"],
                 "flying" if state["active"] else "not flying", flying_at))
        print("         requested %u, channel %u, %u reached"
              % (state["requested"], state["channel"], state["reached"]))
        print("         %u started, %u taken back, holding %d mm"
              % (state["started"], state["cancelled"], state["hold_alt_mm"]))
        return 0

    elif args.command == "calibrate":
        # The console's four calibrations over the wire. The console blocks its
        # flight loop and prints a progress line; this starts a session and
        # polls it, which is what the wire is shaped for and what the wizard
        # needs. Where the console answers in prose this prints the numbers the
        # reply actually carries.
        verbs = {"status": CALIBRATE_STATUS, "gyro": CALIBRATE_GYRO,
                 "rc": CALIBRATE_RC, "accel": CALIBRATE_ACCEL,
                 "vbat": CALIBRATE_VBAT, "abort": CALIBRATE_ABORT}
        what = args.args[0] if args.args else "status"
        if what not in verbs:
            print("akproto: calibrate takes one of %s"
                  % ", ".join(sorted(verbs)), file=sys.stderr)
            return 1

        face = 0
        mv = 0
        if what == "accel":
            if len(args.args) < 2:
                print("akproto: calibrate accel needs a face, 0 to %u"
                      % (CALIBRATE_FACES - 1), file=sys.stderr)
                return 1
            try:
                face = int(args.args[1])
            except ValueError:
                print("akproto: face %r is not a number" % args.args[1],
                      file=sys.stderr)
                return 1
        elif what == "vbat":
            if len(args.args) < 2:
                print("akproto: calibrate vbat needs the pack's millivolts",
                      file=sys.stderr)
                return 1
            try:
                mv = int(args.args[1])
            except ValueError:
                print("akproto: %r is not a number" % args.args[1],
                      file=sys.stderr)
                return 1

        verb = verbs[what]
        try:
            if verb == CALIBRATE_STATUS:
                state = calibration_state(client)
            else:
                state = calibrate(client, verb, face=face, mv=mv)
        except IOError as problem:
            print("akproto: %s" % problem, file=sys.stderr)
            return 1

        # `gyro` and `rc` return with the session running - the firmware will
        # not hold a 1 kHz flight loop for three seconds - so what a person
        # asked to start is followed here until it stops being active or the
        # firmware's own 15 s bound has plainly passed. The loop prints where
        # the sampling has got to, because for `rc` a count that has stopped
        # climbing *is* the diagnosis.
        deadline = time.monotonic() + 20.0
        while state["active"] and time.monotonic() < deadline:
            print("  %s: %u taken, %u rejected"
                  % (state["verb_name"], state["samples"], state["rejected"]))
            time.sleep(0.25)
            state = calibration_state(client)

        if state["active"]:
            print("akproto: the %s calibration is still sampling after 20 s"
                  % state["verb_name"], file=sys.stderr)
            return 1

        print("calibrate %s: %s"
              % (state["verb_name"] or "no session so far",
                 CALIBRATE_STATUS_NAMES[state["status"]]))
        if state["verb"] == CALIBRATE_ACCEL:
            done = bin(state["faces"] & CALIBRATE_ALL_FACES).count("1")
            print("         %u of %u faces measured" % (done, CALIBRATE_FACES))
            # One face's reply carries no result - the six are one measurement
            # of a scale and an offset, and five of them alone are not one. So
            # the slots are printed only once the flow has all six, rather than
            # printing six zeroes that read like a calibrated aircraft.
            if state["faces"] & CALIBRATE_ALL_FACES != CALIBRATE_ALL_FACES:
                return 0 if state["status"] == CALIBRATE_OK else 1
        for name, value in zip(state["result_names"], state["results"]):
            print("         %-22s %d" % (name, value))
        return 0 if state["status"] == CALIBRATE_OK else 1

    elif args.command == "list":
        count = parse_hello(client.request(HELLO))["count"]
        for index in range(count):
            payload = client.request(PARAM_GET, bytes([index]))
            if payload[0] != 0:
                continue
            name = c_string(payload, 1)
            value = c_string(payload, 2 + len(name))
            print("%2u  %-16s %s" % (index, name, value))

    elif args.command == "info":
        # What each parameter *is*. This is the answer the console has always
        # had beside every value and the wire used to have nowhere to put.
        for entry in fetch_param_info(client):
            if entry["name"] is None:
                print("%2u  (the board cannot describe this row in one frame)"
                      % entry["index"])
                continue
            if entry["type"] == PARAM_TEXT:
                extent = "text up to %u" % entry["max_len"]
            else:
                extent = "%s..%s" % (entry["min"], entry["max"])
            print("%2u  %-24s %-8s %-11s %-18s default %s%s"
                  % (entry["index"], entry["name"],
                     {PARAM_FLOAT: "float", PARAM_U32: "u32",
                      PARAM_TEXT: "text"}.get(entry["type"], "?"),
                     entry["group_name"], extent,
                     "***" if entry["secret"] else entry["default"],
                     "  (secret)" if entry["secret"] else ""))

    elif args.command == "help":
        if not args.args:
            print("akproto: help needs a parameter name or index",
                  file=sys.stderr)
            return 1
        entries = fetch_param_info(client)
        wanted = args.args[0]
        entry = next((e for e in entries
                      if e["name"] == wanted or str(e["index"]) == wanted), None)
        if entry is None:
            print("akproto: no parameter named %s" % wanted, file=sys.stderr)
            return 1
        print("%s (%u) - %s" % (entry["name"], entry["index"], entry["group_name"]))
        print(fetch_param_help(client, entry["index"]))

    elif args.command == "get":
        payload = client.request(PARAM_GET, bytes([int(args.args[0])]))
        if payload[0] != 0:
            print("no such parameter", file=sys.stderr)
            return 1
        name = c_string(payload, 1)
        value = c_string(payload, 2 + len(name))
        print("%s = %s" % (name, value))

    elif args.command == "set":
        index = int(args.args[0])
        value = args.args[1].encode()
        payload = client.request(PARAM_SET, bytes([index]) + value)
        status, message = set_reply(payload)
        print("set: %s%s" % (SET_STATUS.get(status, "unknown status %u" % status),
                             (" (%s)" % message) if message else ""))
        return 0 if status == 0 else 1

    elif args.command == "save":
        payload = client.request(PARAM_SAVE)
        print("save: %s" % SET_STATUS.get(payload[0], "unknown status %u" % payload[0]))
        return 0 if payload[0] == 0 else 1

    elif args.command == "log":
        source = LOG_SOURCES[args.source]
        try:
            pull = (lambda write: fetch_log_streamed(client, write, source))
            if not args.pull:
                pull = lambda write: (fetch_log(client, write, source), 0)
            if args.args:
                with open(args.args[0], "w") as handle:
                    count, holes = pull(handle.write)
                print("wrote %u records to %s%s"
                      % (count, args.args[0],
                         "" if holes == 0 else " (%u holes)" % holes))
            else:
                count, holes = pull(sys.stdout.write)
                print("# %u records%s"
                      % (count, "" if holes == 0 else ", %u holes" % holes),
                      file=sys.stderr)
        except IOError as problem:
            # A device without that log is a thing to be told, not a traceback:
            # this is the command somebody runs on a crashed aircraft, at the
            # worst possible moment to be reading a Python stack.
            print("akproto: %s" % problem, file=sys.stderr)
            return 1
        return 0

    elif args.command == "telemetry":
        # A stream, printed one line per frame until stopped. This is what a
        # person watching an aircraft from the network sees - and it is the
        # same frame the configurator would read, so there is one thing to get
        # right rather than two.
        hz = int(args.args[0]) if args.args else 10
        agreed = subscribe(client, hz)
        if agreed == 0:
            print("telemetry: the firmware will not stream", file=sys.stderr)
            return 1
        print("# telemetry at %u Hz, Ctrl-C to stop" % agreed, file=sys.stderr)
        try:
            while True:
                frame = next_telemetry(client)
                print("%8u ms  state %u link %u fix %u/%u  roll %6.1f "
                      "pitch %6.1f yaw %6.1f  %10.7f %11.7f  motors %s"
                      % (frame["uptime_ms"], frame["flight_state"],
                         frame["link_live"], frame["gps_fix_type"],
                         frame["gps_satellites"], frame["roll_deg"],
                         frame["pitch_deg"], frame["yaw_deg"], frame["lat"],
                         frame["lon"],
                         ",".join(str(m) for m in frame["motors"])))
                sys.stdout.flush()
        except KeyboardInterrupt:
            # Control-C is how a stream ends, not an error.
            return 0
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
