#!/usr/bin/env python3
"""A Betaflight board that is not there - the stand-in for a real one.

    tools/msp_fake_board.py            # speaks MSP on stdin and stdout

This is to a Betaflight board what `tools/fw_sim.c` is to ours: the thing the
client and the window can be pointed at on a machine with no aircraft on it. It
answers the commands `tools/msp.py` reads, with the payload layouts and the
values Betaflight's own source defines, and it is *not* a re-implementation by
memory:

    FC_FIRMWARE_IDENTIFIER  "BTFL"        (src/main/build/version.h)
    FC_VERSION_STRING       "2026.6.1"    (FC_VERSION_YEAR/MONTH/PATCH)
    API_VERSION_MAJOR/MINOR 1 / 48        (src/main/msp/msp_protocol.h)
    MSP_BOARD_INFO          id + hw + type + caps + pstrings
                                          (src/main/msp/msp.c, streambuf.c)
    MSP_ANALOG              the grown frame: vbat u8, mAh, rssi u16,
                            amps i16, vbat u16
    MSP_STATUS              cycle, i2c errors, sensors, modes u32, profile
    MSP_ATTITUDE            roll/pitch/yaw in tenths of a degree
    MSP_RAW_GPS             fix, sats, lat, lon, alt, speed, course
    MSP_MOTOR               8 u16, and MSP_RC the same for the channels

It moves while it is asked: the attitude, the pack and the motor outputs
advance a little on every `MSP_ATTITUDE`, so a stream of polls in a window
looks like a board rather than a frozen one. The arming bit is set once the
test tells it to (`--arm-after SECONDS` on its own command line, or it stays
disarmed) - which is what makes the window's "armed" line checkable in both
states.

What it deliberately does not do: answer anything outside that list. A real
board answers many more commands and says nothing to the ones it does not
know, and "silence for an unknown command" is a behaviour the client has to
survive rather than a case to paper over.
"""

import argparse
import struct
import sys
import time

FRAME_REPLY = b"$M>"
FRAME_REPLY_V2 = b"$X>"
FRAME_REPLY_V2_ERROR = b"$X!"

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

# The two MSP v2 commands a board's *settings* answer on, from
# `src/main/msp/msp_protocol_v2_betaflight.h` - and a small table of real ones.
# The names, types and ranges are Betaflight's own (`src/main/cli/settings.c`:
# `failsafe_throttle` is a `VAR_UINT16` between PWM_PULSE_MIN and
# PWM_PULSE_MAX), and the *text* this board answers with is the format
# `cliGetSettingInfoByName` writes: `pgn=`, `type=`, `min=`, `max=`,
# `default=`.
MSP2_CLI_SETTING = 0x3010
MSP2_CLI_SETTING_INFO = 0x3011

SETTINGS = {
    # name: (pgn, type, min, max, default, value)
    "failsafe_throttle": (21, "uint16", 1000, 2000, 1000, 1050),
    "failsafe_off_delay": (21, "uint16", 0, 200, 200, 200),
}

# How much of a description this board answers in one reply, standing in for a
# real board's output buffer (see `answer_v2`).
WINDOW_BYTES = 40

# Betaflight's own values, from the files named in the docstring.
VARIANT = b"BTFL"
VERSION_YEAR_SINCE_2000 = 2026 - 2000
VERSION_MONTH = 6
VERSION_PATCH = 1
VERSION_STRING = b"2026.6.1"
API_MAJOR, API_MINOR = 1, 48
BOARD_ID = b"S405"
TARGET_NAME = b"STM32F405"
BOARD_NAME = b"AERIALKIT-SIM"
MANUFACTURER = b"AK"

# And INAV's identity, which the window's name list dispatches on: the other
# firmware whose parameters are read by name, and the one whose names come out
# of a YAML file rather than a C table. From its own sources -
# `project(INAV VERSION 9.1.0)` in its CMakeLists, `API_VERSION_MAJOR/MINOR`
# 2/5 in `src/main/msp/msp_protocol.h`, and the variant string its CLI and
# configurator both report. Everything else about this mode is the same board,
# because the frames are MSP's either way - the identity is what is under test.
INAV_VARIANT = b"INAV"
INAV_VERSION = b"9.1.0"
INAV_API_MAJOR, INAV_API_MINOR = 2, 5
INAV_BOARD_ID = b"F405"


def pstring(text):
    """A length byte and that many characters - `sbufWritePString`."""
    return bytes([len(text) & 0xFF]) + text


def checksum(command, payload):
    value = (len(payload) + 1) & 0xFF
    value ^= command
    for byte in payload:
        value ^= byte
    return value


def reply(command, payload):
    return (FRAME_REPLY + bytes([len(payload) + 1, command]) + payload
            + bytes([checksum(command, payload)]))


def crc8_dvb_s2(data):
    """Poly 0xD5, init 0 - the same loop as Betaflight's `crc8_calc`."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def reply_v2(function, payload):
    header = bytes([0, function & 0xFF, (function >> 8) & 0xFF,
                    len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    return FRAME_REPLY_V2 + header + payload + bytes([crc8_dvb_s2(header + payload)])


def refuse_v2(function):
    """What a board answers when it will not do what it was asked: the same
    frame with '!' where '>' goes (`mspSerialEncode`, MSP_RESULT_ERROR)."""
    header = bytes([0, function & 0xFF, (function >> 8) & 0xFF, 0, 0])
    return FRAME_REPLY_V2_ERROR + header + bytes([crc8_dvb_s2(header)])


class Board:
    def __init__(self, arm_after=0.0, inav=False):
        self.started = time.time()
        self.arm_after = arm_after
        self.polls = 0
        # Which firmware this stand-in says it is. Two variants and no more:
        # they are the two the window offers a name list for, and the list is
        # read per release, so the version string is part of the identity.
        self.variant = INAV_VARIANT if inav else VARIANT
        # The version *string* is the fact under test (the window reads it and
        # matches a release's table to it). The three bytes beside it stay the
        # Betaflight stand-in's, because nothing reads them; a real INAV board
        # sends its own.
        self.version = INAV_VERSION if inav else VERSION_STRING
        self.api = (INAV_API_MAJOR, INAV_API_MINOR) if inav else (API_MAJOR,
                                                                  API_MINOR)
        self.board_id = INAV_BOARD_ID if inav else BOARD_ID

    @property
    def armed(self):
        return self.arm_after > 0.0 and \
            time.time() - self.started >= self.arm_after

    def answer(self, command):
        """The v1 commands, or None for one this board does not answer."""
        if command == MSP_API_VERSION:
            return bytes([0, self.api[0], self.api[1]])
        if command == MSP_FC_VARIANT:
            return self.variant
        if command == MSP_FC_VERSION:
            return (bytes([VERSION_YEAR_SINCE_2000, VERSION_MONTH,
                           VERSION_PATCH]) + pstring(self.version))
        if command == MSP_BOARD_INFO:
            return (self.board_id + struct.pack("<H", 0) + bytes([0, 0x0B])
                    + pstring(TARGET_NAME) + pstring(BOARD_NAME)
                    + pstring(MANUFACTURER) + bytes(32) + bytes([0, 0]))
        if command == MSP_STATUS:
            # 1 kHz task delta, no i2c errors, the sensors a board like this
            # has (acc, baro, gps, gyro - bits 0,1,3,5), the ARM box as bit 0
            # of the mode flags when it is armed, profile 0, no load.
            sensors = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5)
            modes = 1 if self.armed else 0
            return struct.pack("<HHHI", 1000, 0, sensors, modes) + bytes([0, 0, 0])
        if command == MSP_ATTITUDE:
            self.polls += 1
            roll = 150 + self.polls * 3
            pitch = -80 - self.polls
            yaw = 2700 + self.polls * 10
            return struct.pack("<hhh", roll, pitch, yaw)
        if command == MSP_ANALOG:
            volts = 12.6 - self.polls * 0.01
            return (bytes([int(volts * 10) & 0xFF])
                    + struct.pack("<H", 210 + self.polls)
                    + struct.pack("<H", 90)
                    + struct.pack("<h", 1250)
                    + struct.pack("<H", int(volts * 100)))
        if command == MSP_RAW_GPS:
            return (bytes([3, 11])
                    + struct.pack("<ii", 521234567, 49876543)
                    + struct.pack("<HHH", 41, 512, 2750))
        if command == MSP_MOTOR:
            return struct.pack("<8H", 1000 + self.polls, 1100, 1200, 1300,
                               0, 0, 0, 0)
        if command == MSP_RC:
            return struct.pack("<8H", 992, 992, 992, 172, 992, 1811, 992, 992)
        return None

    def answer_v2(self, function, payload):
        """The MSP v2 commands: a Betaflight board's settings, by name.

        Returns `(payload, ok)`: `ok` False is the error frame a board sends
        for a name it does not have, which is a reply rather than silence.
        """
        if function == MSP2_CLI_SETTING:
            name = payload.decode("ascii", "replace").strip()
            if name not in SETTINGS:
                return b"", False
            value = SETTINGS[name][5]
            return ("%s = %d" % (name, value)).encode(), True

        if function == MSP2_CLI_SETTING_INFO:
            # `name\0<offset u16 LE>`, and the answer is the total length of
            # the description followed by the window at that offset.
            nul = payload.find(b"\x00")
            name = payload[:nul if nul >= 0 else len(payload)].decode("ascii",
                                                                     "replace")
            offset = 0
            if nul >= 0 and len(payload) >= nul + 3:
                offset = payload[nul + 1] | (payload[nul + 2] << 8)
            if name not in SETTINGS:
                return b"", False
            pgn, kind, lo, hi, default, _value = SETTINGS[name]
            text = ("pgn=%u\ntype=%s\nmin=%d\nmax=%d\ndefault=%d"
                    % (pgn, kind, lo, hi, default))
            # A board answers a description in windows the size of its own
            # output buffer - `cliGetSettingInfoByName` exists because of that
            # buffer - so this one is deliberately smaller than the text. A
            # client that did not ask again would show half a description.
            window = text[offset:offset + WINDOW_BYTES]
            return struct.pack("<H", len(text)) + window.encode(), True

        return b"", False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-after", type=float, default=0.0,
                        help="seconds until the ARM box goes high (0 = never)")
    parser.add_argument("--inav", action="store_true",
                        help="answer as INAV 9.1.0 instead of Betaflight, so "
                             "the window's other name list is exercised")
    args = parser.parse_args()

    board = Board(args.arm_after, inav=args.inav)
    data = sys.stdin.buffer

    def exactly(count):
        """`count` bytes, or None when the other end stopped talking."""
        out = b""
        while len(out) < count:
            piece = data.read(count - len(out))
            if not piece:
                return None
            out += piece
        return out

    while True:
        # Any byte that is not the start of a request frame is skipped, which
        # is what a board's serial parser does with a stream somebody typed at.
        first = data.read(1)
        if not first:
            return 0
        if first != b"$":
            continue
        magic = exactly(2)
        if magic == b"X<":
            # MSP v2: flags, a 16-bit function, a 16-bit size, then the
            # payload and a CRC-8 rather than a v1 frame's XOR.
            head = exactly(5)
            if head is None:
                return 0
            function = head[1] | (head[2] << 8)
            size = head[3] | (head[4] << 8)
            body = exactly(size + 1)
            if body is None:
                return 0
            payload, check = body[:size], body[size]
            if check != crc8_dvb_s2(head + payload):
                continue                        # dropped, not answered
            answer, ok = board.answer_v2(function, payload)
            if not ok:
                sys.stdout.buffer.write(refuse_v2(function))
            else:
                sys.stdout.buffer.write(reply_v2(function, answer))
            sys.stdout.buffer.flush()
            continue
        if magic != b"M<":
            continue

        size = exactly(1)
        if size is None:
            return 0
        body = exactly(size[0])                 # command + payload
        if body is None:
            return 0
        check = exactly(1)
        if check is None:
            return 0

        command = body[0]
        payload = body[1:]
        if check[0] != checksum(command, payload):
            continue                            # dropped, not answered

        answer = board.answer(command)
        if answer is None:
            continue                    # a command this board does not know
        sys.stdout.buffer.write(reply(command, answer))
        sys.stdout.buffer.flush()


if __name__ == "__main__":
    sys.exit(main())
