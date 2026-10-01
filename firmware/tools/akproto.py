#!/usr/bin/env python3
"""Talk to AerialKit's config protocol.

    akproto.py [--port /dev/ttyACM0] hello|status|list|get N|set N VALUE|save
               |info|help N|log [FILE]|telemetry [HZ]|rc

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
        while True:
            command, payload = self.read_frame()
            if command == wanted:
                return payload
            if command not in allow:
                raise IOError("reply to %02x, wanted %02x" % (command, wanted))

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
    while True:
        command, payload = client.read_frame()
        if command == TELEMETRY:
            return parse_telemetry(payload)


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


LOG_FIELDS = ["time_ms", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y",
              "accel_z", "roll", "pitch", "yaw", "alt_mm", "stick_roll",
              "stick_pitch", "stick_yaw", "stick_throttle", "torque_roll",
              "torque_pitch", "torque_yaw", "motor1", "motor2", "motor3",
              "motor4", "state", "flags", "lat_e7", "lon_e7"]


def parse_log_record(payload):
    """The 51-byte record layout from ak_log.h, decoded the way a client would."""
    body = payload[1:1 + 51]
    if len(body) < 51:
        raise IOError("short log record")

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
    return values


LOG_HEADER = ("# aerialkit blackbox, pulled over the config protocol\n"
              "# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, "
              "alt mm above the take-off reference, sticks per-mille, torque "
              "percent, motor 0..254\n"
              + ",".join(LOG_FIELDS) + "\n")

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
          "# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, "
          "sticks per-mille, torque percent, motor 0..254\n"
          % name
          + ",".join(LOG_FIELDS) + "\n")
    skipped = 0
    for index in range(count):
        payload = client.request(LOG_GET, index.to_bytes(2, "little"))
        if payload[0] != 0:
            skipped += 1
            continue
        write(",".join(str(v) for v in parse_log_record(payload)) + "\n")
    return count - skipped


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
                                 "log", "telemetry", "info", "help", "rc"])
    parser.add_argument("--source", choices=sorted(LOG_SOURCES),
                        default="fast",
                        help="which log `log` pulls: fast (RAM), long (retained "
                             "RAM), flash (survives the battery)")
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
            if args.args:
                with open(args.args[0], "w") as handle:
                    count = fetch_log(client, handle.write, source)
                print("wrote %u records to %s" % (count, args.args[0]))
            else:
                count = fetch_log(client, sys.stdout.write, source)
                print("# %u records" % count, file=sys.stderr)
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
