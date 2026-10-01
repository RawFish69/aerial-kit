#!/usr/bin/env python3
"""MSP - the protocol Betaflight and INAV speak, read from here.

    tools/msp.py --port /dev/ttyACM0            # what is on the other end?
    tools/msp.py --port /dev/ttyACM0 --watch    # and what is it doing?

**Why this exists.** There is no single protocol to be compatible with: this
firmware speaks its own, Betaflight and INAV speak MSP, ArduPilot and PX4 speak
MAVLink. A window that talks to more than one of them knows more than one of
them, and this is the file that knows MSP - what the owner asked for
(`~/aerialkit-goal.md`, and `docs/27-configurator.md`).

**What it is not.** Read-only, deliberately, and for the reason the goal's note
gives: parameter *writes* to somebody else's firmware are how a tool crashes an
aircraft. Nothing here has a write command, and the window says so rather than
offering a button that does nothing.

**Where the bytes come from.** Every frame below is Betaflight's own
implementation, not a summary of it: `upstream/betaflight-2026.6.1/src/main/
msp/msp_serial.c` for the framing and `msp_protocol.h`/`msp.c` for the
commands and their payload layouts. That is what the boards send and what the
existing configurators read, which makes it the only specification worth
testing against:

    $M<  size  command  payload...  checksum      (host asks)
    $M>  size  command  payload...  checksum      (board answers)

`size` counts the command byte and the payload; the checksum is the XOR of
`size`, `command` and every payload byte. MSP v1 has no sequence number and no
response bit: a reply carries the *same* command it answers.

INAV is the same protocol with its own command set (its `MSP_FC_VARIANT` is
`INAV`, its version fields differ - see `decode_version`), and ArduPilot and PX4
are not here at all: they are MAVLink, which is a second file's worth of work
and is recorded as such.
"""

import argparse
import os
import queue
import struct
import sys
import threading
import time

FRAME_REQUEST = b"$M<"
FRAME_REPLY = b"$M>"
# MSP v2, which is a different frame under a different magic byte - and the
# only one that carries the commands a Betaflight board's settings need. The
# magic table is `{'M', 'M', 'X'}` for {v1, v2-over-v1, v2} in
# `src/main/msp/msp.h`, and a v2 frame's checksum is CRC-8/DVB-S2 (poly 0xD5,
# init 0) over the five header bytes and the payload - where v1's is an XOR.
FRAME_REQUEST_V2 = b"$X<"
FRAME_REPLY_V2 = b"$X>"
FRAME_REPLY_V2_ERROR = b"$X!"

# Commands, from msp_protocol.h. The numbers are the protocol.
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ANALOG = 110

# The two commands a Betaflight board's *settings* answer on, from
# `msp_protocol_v2_betaflight.h` - and the reason they are up here rather than
# with the commands above: a v2 command is 16 bits wide, and these are two of
# the three thousandths, which do not fit under an 8-bit command byte.
#
# **There is no "list the settings" command**, and that is the finding worth
# having in writing: `MSP2_CLI_SETTING` takes a *name* and answers the text
# "name = value", and `MSP2_CLI_SETTING_INFO` takes a name and answers that
# setting's description. A ground station that shows a table of them - the
# Betaflight configurator - ships the *list of names* for each firmware version
# itself, because the protocol has no way to ask for one. That is the
# per-firmware parameter model the goal notes, in the protocol's own words.
MSP2_CLI_SETTING = 0x3010
MSP2_CLI_SETTING_INFO = 0x3011

# A v1 size byte is a byte, so a payload is at most 255 - and the frame at most
# 259 bytes. Used to bound what a stream of noise can make this allocate.
MAX_PAYLOAD = 255

# The protocol version this file speaks. Betaflight answers 0 here for MSP v1;
# a board that answers something else is a board to look at before believing
# anything else it says.
MSP_PROTOCOL_VERSION = 0


class Timeout(IOError):
    """The board did not answer in time."""


class ProtocolError(IOError):
    """Something that was not a valid frame arrived, or the frame was wrong."""


class Error(IOError):
    """The board answered, and the answer was no.

    MSP says that with the frame's third byte: `$X!` is a reply that carries an
    error rather than a result (`mspSerialEncode`, which writes '!' when the
    packet's result is MSP_RESULT_ERROR). A client that treated it as data
    would show a person an empty answer where the board said "no such name".
    """


def checksum(command, payload):
    """The XOR MSP defines: size, command, payload."""
    value = (len(payload) + 1) & 0xFF
    value ^= command & 0xFF
    for byte in payload:
        value ^= byte
    return value


def build(command, payload=b""):
    """One request frame, ready for the wire."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload of %u bytes does not fit an MSP v1 frame"
                         % len(payload))
    return (FRAME_REQUEST + bytes([len(payload) + 1, command & 0xFF]) + payload
            + bytes([checksum(command, payload)]))


def parse(frame):
    """`(command, payload)` from a whole frame, checksum checked.

    A frame that is not one raises: the caller here is a client that asked a
    question, so anything else on the wire is a fact worth telling somebody
    about rather than skipping past.
    """
    if len(frame) < 6 or frame[:3] != FRAME_REPLY:
        raise ProtocolError("not an MSP reply: %r" % frame[:8])
    size = frame[3]
    # Three bytes of marker, the size byte, `size` bytes of command and
    # payload, and the checksum: the whole frame is `size + 5`.
    if size < 1 or len(frame) != size + 5:
        raise ProtocolError("frame length %u does not match its size byte %u"
                            % (len(frame), size))
    command = frame[4]
    payload = frame[5:4 + size]
    if frame[4 + size] != checksum(command, payload):
        raise ProtocolError("bad checksum in the reply to %u" % command)
    return command, payload


def crc8_dvb_s2(data):
    """The CRC a v2 frame carries: poly 0xD5, init 0, MSB first.

    The same bit-by-bit loop as `crc8_calc` in Betaflight's
    `src/main/common/crc.c`, which is what `crc8_dvb_s2_update` calls.
    """
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_v2(function, payload=b""):
    """One MSP v2 request: `$X<` flags cmd(2, LE) size(2, LE) payload crc.

    `size` counts only the payload here, where a v1 frame's counts the command
    as well - one more way the two frames are not the same frame.
    """
    header = bytes([0, function & 0xFF, (function >> 8) & 0xFF,
                    len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    return FRAME_REQUEST_V2 + header + payload + bytes([crc8_dvb_s2(header + payload)])


def parse_v2(frame):
    """`(function, payload)` from a whole v2 reply, checksum checked."""
    if len(frame) < 9 or frame[:3] not in (FRAME_REPLY_V2, FRAME_REPLY_V2_ERROR):
        raise ProtocolError("not an MSP v2 reply: %r" % frame[:8])
    flags, function, size = (frame[3], frame[4] | (frame[5] << 8),
                             frame[6] | (frame[7] << 8))
    if len(frame) != 8 + size + 1:
        raise ProtocolError("v2 frame length %u does not match its size %u"
                            % (len(frame), size))
    payload = frame[8:8 + size]
    if frame[8 + size] != crc8_dvb_s2(frame[3:8 + size]):
        raise ProtocolError("bad crc in the v2 reply to 0x%04x" % function)
    if frame[:3] == FRAME_REPLY_V2_ERROR:
        raise Error("the board refused 0x%04x" % function)
    return function, payload


class Deadline:
    """A transport's bytes, with a deadline in front of them.

    Detection is why: "does the other end answer our hello?" is a question with
    a timeout, and a blocking read on a pipe never comes back to ask. A thread
    owns the transport's reads and puts what it gets on a queue; callers take
    bytes out of the queue, either with a deadline (`read_some`) or waiting
    (`read`, which is what the rest of the tools' clients expect).

    `b""` from the transport means the other end stopped; a socket's own
    timeout is the one exception and means "nothing yet", because a serial or
    socket board is allowed to be quiet for a moment without having gone away.
    """

    def __init__(self, transport):
        self.transport = transport
        self.bytes = queue.Queue()
        self.eof = False
        self.thread = threading.Thread(target=self._pump, daemon=True)
        self.thread.start()

    def _pump(self):
        while True:
            try:
                # One byte at a time, because the transports differ in what a
                # read *means*: a socket and a file descriptor return what has
                # arrived, and a pipe's `read(n)` waits for all n, which is a
                # pump that never delivers a short frame. Asking for one byte
                # is the shape all three agree on, and the rates here are a
                # console's.
                data = self.transport.read(1)
            except OSError as problem:
                # A socket that timed out is not a socket that has gone: the
                # board is simply quiet, which is the normal state of a
                # console nobody is typing at.
                if "timed out" in str(problem) or isinstance(problem,
                                                             TimeoutError):
                    continue
                data = b""
            except Exception:                            # noqa: BLE001
                data = b""
            if not data:
                self.eof = True
                self.bytes.put(b"")
                return
            self.bytes.put(data)

    def read_some(self, count, timeout):
        """Up to `count` bytes, or `b""` after `timeout` if nothing came.

        The deadline covers the whole read rather than each byte, which is what
        a frame reader wants: "did a whole reply arrive in time" is one
        question, not one per byte.
        """
        out = b""
        deadline = time.time() + timeout
        while len(out) < count:
            left = deadline - time.time()
            if left <= 0:
                break
            try:
                piece = self.bytes.get(timeout=left)
            except queue.Empty:
                break
            if piece == b"" and self.eof:
                break
            out += piece
        return out

    def read(self, count):
        """Bytes, waiting for them - the contract the other clients expect.

        `b""` means the other end stopped, which is what `akproto`'s client
        treats it as; the 0.5 s is a poll, not a deadline, so a quiet board
        does not look like a gone one to whoever is waiting on a log.
        """
        while True:
            data = self.read_some(count, 0.5)
            if data or self.eof:
                return data


class Msp:
    """One request and its answer, over a byte stream.

    `read` is a callable taking `(count, timeout)` and returning up to `count`
    bytes, or `b""` when the timeout passes with nothing to give - which is the
    shape `Deadline.read_some` and `Port.read` both have, and the shape a
    client needs so that "the board did not answer" is a value rather than a
    hang.
    """

    def __init__(self, read, write, timeout=1.0):
        self.read = read
        self.write = write
        self.timeout = timeout
        self.frames = 0          # answers parsed
        self.unsolicited = 0     # frames for a command nobody asked about
        self.noise = 0           # bytes that were not part of a frame

    def request(self, command, payload=b"", timeout=None):
        self.write(build(command, payload))
        return self._await(command, self.timeout if timeout is None else timeout)

    def request_v2(self, function, payload=b"", timeout=None):
        """The same, in a v2 frame. Returns `(payload, flags)`.

        A v2 command is a 16-bit function rather than an 8-bit command, which
        is the whole reason Betaflight's settings live up here: `0x3010` and
        `0x3011` do not fit under an 8-bit command byte.
        """
        self.write(build_v2(function, payload))
        function_out, payload_out, flags = self._await_v2(
            function, self.timeout if timeout is None else timeout)
        if function_out != function:
            raise ProtocolError("reply to 0x%04x, wanted 0x%04x"
                                % (function_out, function))
        return payload_out, flags

    def _await(self, wanted, timeout):
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise Timeout("no answer to command %u within %.1f s"
                              % (wanted, timeout))
            frame = self._read_frame(left)
            if frame is None:
                raise Timeout("no answer to command %u within %.1f s"
                              % (wanted, timeout))
            version, command, payload = frame
            if version == "v1" and command == wanted:
                return payload
            # A board with a telemetry stream, or one that answers in another
            # order, is not an error: it is a frame this client did not ask
            # for, and it is counted rather than dropped silently.
            self.unsolicited += 1

    def _await_v2(self, wanted, timeout):
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise Timeout("no answer to 0x%04x within %.1f s"
                              % (wanted, timeout))
            frame = self._read_frame(left)
            if frame is None:
                raise Timeout("no answer to 0x%04x within %.1f s"
                              % (wanted, timeout))
            version, function, payload = frame
            if version == "v2" and function == wanted:
                return function, payload, 0
            if version == "v2" and function == wanted | 0x8000:
                # Some boards answer a v2 command with the high bit set, which
                # is what the *v1* command space does with replies; accepting
                # both is what the existing configurators do.
                return function & 0x7FFF, payload, 0
            self.unsolicited += 1

    def _read_frame(self, timeout):
        """The next `$M>` frame, or None if one did not arrive in time."""
        deadline = time.time() + timeout

        def byte():
            left = deadline - time.time()
            if left <= 0:
                return None
            data = self.read(1, left)
            if not data:
                return b"" if time.time() < deadline else None
            return data

        # Find the start marker, counting what had to be skipped: a board that
        # has just printed its banner puts text in front of the first reply.
        # The marker's own three bytes are not "noise", which a window-based
        # scan cannot tell - so this is a small state machine rather than a
        # sliding window. Both magic bytes are accepted here: a board answers
        # v1 or v2 depending on what it was asked, and one connection can see
        # both (a v2 command is answered with `$X>`, everything else with
        # `$M>`).
        state = 0
        while True:
            piece = byte()
            if piece is None:
                return None
            if piece == b"":
                continue
            if state == 0:
                state = 1 if piece == b"$" else 0
                if state == 0:
                    self.noise += 1
            elif state == 1:
                if piece in (b"M", b"X"):
                    magic = piece
                    state = 2
                else:
                    self.noise += 1
                    state = 1 if piece == b"$" else 0
            else:
                if piece in (b">", b"!"):
                    break
                self.noise += 1
                state = 1 if piece == b"$" else 0

        # `parse_v2` raises on the error marker, so the '!' needs no handling
        # of its own here.
        marker = b"$" + magic + piece

        if magic == b"M":
            head = byte()
            if head is None:
                return None
            size = head[0]
            if size < 1 or size > MAX_PAYLOAD + 1:
                raise ProtocolError("size byte %u is not a frame" % size)
            rest = b""
            while len(rest) < size + 1:
                piece = byte()
                if piece is None:
                    raise Timeout("a frame stopped after %u of %u bytes"
                                  % (len(rest) + 4, size + 5))
                rest += piece
            frame = FRAME_REPLY + head + rest
            self.frames += 1
            command, payload = parse(frame)
            return "v1", command, payload

        # v2: five header bytes, then the payload and its crc.
        rest = b""
        while len(rest) < 5:
            piece = byte()
            if piece is None:
                return None
            rest += piece
        size = rest[3] | (rest[4] << 8)
        while len(rest) < 5 + size + 1:
            piece = byte()
            if piece is None:
                raise Timeout("a v2 frame stopped after %u bytes" % len(rest))
            rest += piece
        frame = marker + rest
        self.frames += 1
        function, payload = parse_v2(frame)
        return "v2", function, payload

    # -- a Betaflight board's settings, which are asked for by name --------

    def setting(self, name, timeout=None):
        """One setting's current value, as the board's own sentence.

        The request is the name and the reply is the text `name = value`
        (`msp.c`: `case MSP2_CLI_SETTING`, which for a get echoes what
        `cliGetSettingByName` printed). A name the board does not have comes
        back as an error frame, which `request_v2` raises: a client that showed
        that as an empty value would be inventing one.
        """
        payload, _flags = self.request_v2(MSP2_CLI_SETTING, name.encode(),
                                          timeout)
        return payload.decode("ascii", "replace")

    def setting_info(self, name, timeout=None):
        """One setting's description, in full: what the board says it is.

        The reply is a `u16` total length followed by a *window* of that much
        text starting at the offset the request asked for - so a description
        longer than one frame is read by asking again with a larger offset.
        That is `msp.c`'s `MSP2_CLI_SETTING_INFO` and `cli.c`'s
        `cliGetSettingInfoByName`; the keys it writes (`pgn=`, `type=`,
        `min=`/`max=`, `values=`, `default=`) are what this returns, and a
        client that wanted a *typed* field would be re-inventing them.
        """
        text = ""
        offset = 0
        for _ in range(16):             # bounded: sixteen frames of a description
            request = name.encode() + b"\x00" + struct.pack("<H", offset)
            payload, _flags = self.request_v2(MSP2_CLI_SETTING_INFO, request,
                                              timeout)
            if len(payload) < 2:
                raise ProtocolError("a setting description with no length")
            total = payload[0] | (payload[1] << 8)
            window = payload[2:].decode("ascii", "replace")
            text += window
            offset += len(window)
            if not window or offset >= total:
                break
        return text


# --- the commands this file reads, and what their answers mean -------------
#
# Each decoder is written against the handler in Betaflight's msp.c and cites
# it, because the bytes are the only thing that matters and a summary of them
# is how a client gets a field wrong.

def decode_api_version(payload):
    """`MSP_API_VERSION` -> protocol, major, minor (msp.c: case MSP_API_VERSION)."""
    if len(payload) < 3:
        raise ProtocolError("api version needs 3 bytes, got %u" % len(payload))
    return {"protocol": payload[0], "api_major": payload[1],
            "api_minor": payload[2]}


def decode_variant(payload):
    """`MSP_FC_VARIANT` -> four characters: BTFL, INAV, ... (msp.c)."""
    return {"variant": payload[:4].decode("ascii", "replace")}


def decode_version(payload):
    """`MSP_FC_VERSION`, in both shapes the two MSP firmwares use.

    Betaflight writes year-since-2000, month, patch and then its version
    string (`FC_VERSION_YEAR - FC_CALVER_BASE_YEAR`, then a length-prefixed
    string). INAV writes major, minor, patch - which is what the older
    Betaflight did too, and what the existing configurators still accept. The
    two are told apart by the variant, not by guessing from the numbers.
    """
    if len(payload) < 3:
        raise ProtocolError("version needs 3 bytes, got %u" % len(payload))
    if len(payload) == 3:
        return {"major": payload[0], "minor": payload[1], "patch": payload[2],
                "text": "%u.%u.%u" % (payload[0], payload[1], payload[2])}
    text = payload[4:4 + payload[3]].decode("ascii", "replace") \
        if len(payload) > 4 else ""
    return {"major": payload[0] + 2000, "minor": payload[1], "patch": payload[2],
            "text": text or "%u-%02u-%u" % (2000 + payload[0], payload[1],
                                            payload[2])}


def decode_board_info(payload):
    """`MSP_BOARD_INFO` -> the board's identifier and the target's name.

    Four bytes of identifier, a hardware revision, a board type, the target
    capabilities, and then three length-prefixed strings: the target, the
    board's name and the manufacturer. `sbufWritePString` is a length byte and
    exactly that many characters (`common/streambuf.c`) - not a NUL-terminated
    string, which is how a reader that split on zero would swallow the fields
    after it.
    """
    if len(payload) < 4:
        raise ProtocolError("board info needs 4 bytes, got %u" % len(payload))
    out = {"board_id": payload[:4].decode("ascii", "replace"),
           "hardware_revision": 0, "board_type": 0, "capabilities": 0,
           "name": "", "board_name": "", "manufacturer": ""}
    if len(payload) >= 8:
        out["hardware_revision"] = struct.unpack_from("<H", payload, 4)[0]
        out["board_type"] = payload[6]
        out["capabilities"] = payload[7]

    at = 8
    for key in ("name", "board_name", "manufacturer"):
        if at >= len(payload):
            break
        length = payload[at]
        at += 1
        out[key] = payload[at:at + length].decode("ascii", "replace")
        at += length
    return out


def decode_status(payload):
    """`MSP_STATUS` -> cycle time, sensors, the arming flags (msp.c)."""
    if len(payload) < 11:
        raise ProtocolError("status needs 11 bytes, got %u" % len(payload))
    cycle, i2c_errors, sensors = struct.unpack_from("<HHH", payload, 0)
    modes = struct.unpack_from("<I", payload, 6)[0]
    return {"cycletime_us": cycle, "i2c_errors": i2c_errors,
            "sensors": sensors, "modes": modes, "profile": payload[10]}


def decode_attitude(payload):
    """`MSP_ATTITUDE` -> roll, pitch, yaw in degrees (tenths on the wire)."""
    if len(payload) < 6:
        raise ProtocolError("attitude needs 6 bytes, got %u" % len(payload))
    roll, pitch, yaw = struct.unpack_from("<hhh", payload, 0)
    return {"roll_deg": roll / 10.0, "pitch_deg": pitch / 10.0,
            "yaw_deg": yaw / 10.0}


def decode_analog(payload):
    """`MSP_ANALOG` -> the pack, in the two shapes Betaflight has written.

    Three bytes is the old frame (`vbat` in tenths of a volt, mAh, rssi). The
    frame grew: rssi became a `u16`, then a current in hundredths of an amp and
    the voltage again in hundredths of a volt. A reader has to accept both,
    because both are on real boards.
    """
    if len(payload) < 3:
        raise ProtocolError("analog needs 3 bytes, got %u" % len(payload))
    out = {"vbat_v": payload[0] / 10.0,
           "mah": struct.unpack_from("<H", payload, 1)[0], "rssi": 0,
           "amps": 0.0}
    if len(payload) >= 9:
        out["rssi"] = struct.unpack_from("<H", payload, 3)[0]
        out["amps"] = struct.unpack_from("<h", payload, 5)[0] / 100.0
        out["vbat_v"] = struct.unpack_from("<H", payload, 7)[0] / 100.0
    elif len(payload) >= 6:
        out["rssi"] = struct.unpack_from("<H", payload, 3)[0]
    return out


def decode_raw_gps(payload):
    """`MSP_RAW_GPS` -> fix type, satellites, position, altitude, speed."""
    if len(payload) < 16:
        raise ProtocolError("raw gps needs 16 bytes, got %u" % len(payload))
    fix, sats = payload[0], payload[1]
    lat, lon = struct.unpack_from("<ii", payload, 2)
    alt, speed, course = struct.unpack_from("<HHH", payload, 10)
    return {"gps_fix_type": fix, "gps_satellites": sats,
            "lat": lat / 1e7, "lon": lon / 1e7, "alt_m": float(alt),
            "ground_speed_cms": speed, "ground_course_deg": course / 10.0}


def decode_motor(payload):
    """`MSP_MOTOR` -> eight motor outputs, as the board sends them."""
    return {"motors": list(struct.unpack_from("<%dH" % (len(payload) // 2),
                                              payload, 0))}


def decode_rc(payload):
    """`MSP_RC` -> the channel values, in microseconds on most boards."""
    return {"channels": list(struct.unpack_from("<%dH" % (len(payload) // 2),
                                                payload, 0))}


def identify(msp):
    """Ask the questions that answer "what is on the other end?".

    Four round trips, and the first one is the one that decides whether this is
    MSP at all (`msp_protocol.h` says an API client SHOULD ask for the API
    version first, which is also the smallest frame a board can answer).
    """
    api = decode_api_version(msp.request(MSP_API_VERSION))
    variant = decode_variant(msp.request(MSP_FC_VARIANT))["variant"].strip()
    version = decode_version(msp.request(MSP_FC_VERSION))
    try:
        board = decode_board_info(msp.request(MSP_BOARD_INFO))
    except (Timeout, ProtocolError):
        # A board that does not answer this one is still a board; the other
        # three answered, and saying "no board information" is the honest
        # answer rather than failing the identification.
        board = {"board_id": "", "hardware_revision": 0, "name": ""}
    return {"api": api, "variant": variant, "version": version, "board": board}


def short_name(identity):
    """`Betaflight 2026.6.1` - the two words a sentence about the board needs."""
    names = {"BTFL": "Betaflight", "INAV": "INAV", "EMUF": "EmuFlight",
             "CLFL": "Cleanflight"}
    variant = identity["variant"] or "?"
    return "%s %s" % (names.get(variant, variant), identity["version"]["text"])


def describe(identity):
    """One line for a person: which firmware, which version, which board."""
    board = identity["board"]
    where = board["board_id"] or board["name"] or "an unnamed board"
    return ("%s (%s), API %u.%u, MSP protocol %u, board %s"
            % (short_name(identity), identity["variant"],
               identity["api"]["api_major"], identity["api"]["api_minor"],
               identity["api"]["protocol"], where))


def state(msp):
    """Everything this file reads about a live board, in one dict.

    The keys are the ones the configurator's own status pane uses, so a window
    can show an MSP board and one of ours through the same widgets - and the
    ones MSP does not carry are `None` rather than a zero, because "this
    firmware does not say" and "the value is zero" are different answers.
    """
    status = decode_status(msp.request(MSP_STATUS))
    attitude = decode_attitude(msp.request(MSP_ATTITUDE))
    analog = decode_analog(msp.request(MSP_ANALOG))
    out = {
        "flight_state": status["modes"] & 1,     # Betaflight's ARM box, bit 0
        "link_live": 1,
        "gps_fix_type": None,
        "gps_satellites": None,
        "lat": None,
        "lon": None,
        "motors": [],
        "roll_deg": attitude["roll_deg"],
        "pitch_deg": attitude["pitch_deg"],
        "yaw_deg": attitude["yaw_deg"],
        "vbat_v": analog["vbat_v"],
        "mah": analog["mah"],
        "cycletime_us": status["cycletime_us"],
        "sensors": status["sensors"],
        "modes": status["modes"],
    }
    try:
        gps = decode_raw_gps(msp.request(MSP_RAW_GPS))
        out.update({key: gps[key] for key in ("gps_fix_type", "gps_satellites",
                                              "lat", "lon")})
        out["alt_m"] = gps["alt_m"]
    except (Timeout, ProtocolError):
        out["alt_m"] = None
    try:
        out["motors"] = decode_motor(msp.request(MSP_MOTOR))["motors"]
    except (Timeout, ProtocolError):
        pass
    return out


def open_port(path, baud=115200):
    """A serial port, raw, with the same settings the console uses.

    The same `os`/`termios` approach `akconfig.py` uses for a board's console,
    so a laptop needs nothing installed and there is one place to fix a port
    bug rather than two.
    """
    import termios
    import tty
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    settings = termios.tcgetattr(fd)
    settings[4] = settings[5] = termios.B115200 if baud == 115200 else settings[4]
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    return fd


class Port:
    """A file descriptor as the read/write pair the client wants."""

    def __init__(self, path):
        self.fd = open_port(path)

    def read(self, count, timeout=None):
        deadline = None if timeout is None else time.time() + timeout
        while True:
            try:
                data = os.read(self.fd, count)
            except BlockingIOError:
                data = b""
            if data:
                return data
            if deadline is not None and time.time() >= deadline:
                return b""
            time.sleep(0.002)

    def write(self, data):
        os.write(self.fd, data)

    def close(self):
        os.close(self.fd)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--watch", type=float, default=0.0,
                        help="keep reading and print state every N seconds")
    parser.add_argument("--setting", metavar="NAME", action="append",
                        help="read one of the board's settings by name, and "
                             "say what it is - Betaflight and INAV answer "
                             "these on MSP v2 (there is no way to ask for the "
                             "whole list: see docs/27-configurator.md)")
    args = parser.parse_args()

    port = Port(args.port)
    msp = Msp(port.read, port.write)
    try:
        identity = identify(msp)
        print(describe(identity))
        if args.setting:
            # The settings commands are v2, and a board that speaks them is
            # one that answered the v1 identification above - so a v1-only
            # board says so rather than silently printing nothing.
            for name in args.setting:
                try:
                    print("setting    %s" % msp.setting(name))
                    for line in msp.setting_info(name).splitlines():
                        print("           %s" % line)
                except (Error, ProtocolError) as problem:
                    print("setting    %s: %s" % (name, problem), file=sys.stderr)
                    return 1
            return 0
        print("state      %s" % _state_line(state(msp)))
        while args.watch > 0.0:
            time.sleep(args.watch)
            print("state      %s" % _state_line(state(msp)))
    except Timeout as problem:
        print("no answer: %s" % problem, file=sys.stderr)
        print("a board that does not answer MSP may be running ArduPilot or "
              "PX4 (MAVLink), or AerialKit itself", file=sys.stderr)
        return 1
    finally:
        port.close()
    return 0


def _state_line(now):
    return ("armed %u  roll %5.1f  pitch %5.1f  yaw %5.1f  %.1f V  %u mAh"
            % (now["flight_state"], now["roll_deg"], now["pitch_deg"],
               now["yaw_deg"], now["vbat_v"], now["mah"]))


if __name__ == "__main__":
    sys.exit(main())
