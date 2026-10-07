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

It also answers the *write* half of a settings protocol, which is a different
kind of thing from the reads above: a read reports what the board holds, and a
write changes it. Both firmwares' settings commands are here, and they are
answered as the two genuinely different protocols they are, not as one protocol
with a name swapped in. The one command both share is `MSP_EEPROM_WRITE`, the
separate act that puts the running configuration into flash, and both refuse it
while armed -- so this stand-in refuses it while armed too.

What it deliberately does not do: answer anything outside that list. A real
board answers many more commands and says nothing to the ones it does not
know, and "silence for an unknown command" is a behaviour the client has to
survive rather than a case to paper over. A *refusal*, by contrast, is a frame
the board sends (`$M!` for v1, `$X!` for v2), and it is a different fact from
silence -- which is why `answer` returns a payload-or-nothing and `answer_v2`
returns a payload, a refusal or silence.
"""

import argparse
import struct
import sys
import time

FRAME_REPLY = b"$M>"
FRAME_REPLY_ERROR = b"$M!"
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

# The one command in this file that changes the board rather than reporting it,
# and the one both firmwares share: value 250 in Betaflight's
# `msp_protocol.h` and in INAV's. It writes the running configuration to flash,
# and **both refuse it while armed** -- Betaflight's `mspFcProcessCommand` and
# INAV's `mspFcProcessOutCommand` each check the arming flag before calling
# their own `writeEEPROM`. The client's own gate is a second copy of that rule,
# so this stand-in has to be able to make the refusal or the rule is untested.
MSP_EEPROM_WRITE = 250

# The two MSP v2 commands a *Betaflight* board's settings answer on, from
# `src/main/msp/msp_protocol_v2_betaflight.h` - and a small table of real ones.
# The names, types and ranges are Betaflight's own (`src/main/cli/settings.c`),
# and the *text* this board answers with is the format `cliGetSettingInfoByName`
# writes: `pgn=`, `type=`, `min=`, `max=`, `default=`.
MSP2_CLI_SETTING = 0x3010
MSP2_CLI_SETTING_INFO = 0x3011

# And INAV's, from its `src/main/msp/msp_protocol_v2_common.h`. These are not a
# renaming of Betaflight's: INAV answers *none* of 0x3010/0x3011 and Betaflight
# answers none of these three, so a client that sent the wrong family's command
# to a real board would get silence rather than a value.
MSP2_COMMON_SETTING = 0x1003
MSP2_COMMON_SET_SETTING = 0x1004
MSP2_COMMON_SETTING_INFO = 0x1007

# Betaflight's own `failsafe` settings, read out of the pinned release.
#
# `pgn` is not the bare `PG_FAILSAFE_CONFIG` (which is 1): `PG_REGISTER_I` in
# `src/main/pg/pg.h` stores `.pgn = _pgn | (_version << 12)`, and
# `src/main/flight/failsafe.c:69` registers `failsafeConfig` at version 2. So
# the number `cliGetSettingInfoByName` prints is `1 | (2 << 12)` = 8193, and a
# stand-in that printed `1` would be printing a number no board prints.
#
# The ranges are the ones in `src/main/cli/settings.c`: `failsafe_throttle` runs
# between `PWM_PULSE_MIN` and `PWM_PULSE_MAX` (1000..2000), and
# `failsafe_throttle_low_delay` between 0 and 300.
#
# This table used to carry a third name, `failsafe_off_delay`. There is no such
# setting in the pinned release -- it was removed years ago -- and a stand-in
# that invents a setting teaches a client a name no board will answer.
#
# name: [pgn, type, min, max, default, value]; `value` is what a write changes.
SETTINGS = {
    "failsafe_throttle": [8193, "uint16", 1000, 2000, 1000, 1050],
    "failsafe_throttle_low_delay": [8193, "uint16", 0, 300, 0, 0],
}

# INAV sends the value as its native width and never says which width on the
# wire, so these two tables are what turn a type name into bytes. The codes are
# `setting_type_e` in INAV's `src/main/fc/settings.h`: VAR_UINT8 = 0 through
# VAR_STRING = 6, and the wire carries the *masked* value -- `SETTING_TYPE(s)`
# is `s->type & SETTING_TYPE_MASK` (0x07).
INAV_TYPE_CODE = {"uint8": 0, "int8": 1, "uint16": 2, "int16": 3,
                  "uint32": 4, "float": 5, "string": 6}
INAV_PACK = {"uint8": "<B", "int8": "<b", "uint16": "<H", "int16": "<h",
             "uint32": "<I", "float": "<f"}

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
# configurator both report.
#
# 2026-10-02: this comment used to end "Everything else about this mode is the
# same board, because the frames are MSP's either way - the identity is what is
# under test". **That was wrong**, and it is the mistake this stand-in was
# rebuilt to stop making. INAV is a different *settings protocol*: different
# command numbers, a binary typed value rather than `name = value` text, a
# binary description struct rather than a windowed text one, and a write that is
# answered with an empty ACK saying nothing about the value. Answering the
# Betaflight commands in INAV mode was the stand-in agreeing with a client that
# had it wrong -- which is exactly what a stand-in must never do.
INAV_VARIANT = b"INAV"
INAV_VERSION = b"9.1.0"
INAV_API_MAJOR, INAV_API_MINOR = 2, 5
INAV_BOARD_ID = b"F405"


def pstring(text):
    """A length byte and that many characters - `sbufWritePString`."""
    return bytes([len(text) & 0xFF]) + text


def inav_name(payload):
    """The setting INAV's request names, out of either request shape.

    `mspReadSetting` accepts two: a NUL-terminated name, or a payload that
    *starts* with a NUL followed by a u16 index. This stand-in only has names,
    so a leading NUL names nothing -- which is the right answer, because it is
    what a real board would say about an index it does not have.
    """
    nul = payload.find(b"\x00")
    return payload[:nul if nul >= 0 else len(payload)].decode("ascii", "replace")


def checksum(command, payload):
    # Size is the payload's length alone (Betaflight msp_serial.c).
    value = len(payload) & 0xFF
    value ^= command
    for byte in payload:
        value ^= byte
    return value


def reply(command, payload):
    return (FRAME_REPLY + bytes([len(payload), command]) + payload
            + bytes([checksum(command, payload)]))


def refuse(command):
    """What a v1 board answers when it will not do what it was asked, which is
    the same frame with '!' where '>' goes (`mspSerialEncode` in Betaflight's
    `src/main/msp/msp_serial.c` picks the byte from `packet->result`)."""
    return (FRAME_REPLY_ERROR + bytes([0, command])
            + bytes([checksum(command, b"")]))


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
        self.inav = inav
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
        """The v1 commands.

        Three outcomes, and the difference between the last two is the point:
        `None` is a command this board does not know and answers with silence;
        `(payload, True)` is a reply; `(b"", False)` is a *refusal*, which a
        real board sends as a frame with `!` in it.
        """
        if command == MSP_API_VERSION:
            return bytes([0, self.api[0], self.api[1]]), True
        if command == MSP_FC_VARIANT:
            return self.variant, True
        if command == MSP_FC_VERSION:
            return (bytes([VERSION_YEAR_SINCE_2000, VERSION_MONTH,
                           VERSION_PATCH]) + pstring(self.version)), True
        if command == MSP_BOARD_INFO:
            return (self.board_id + struct.pack("<H", 0) + bytes([0, 0x0B])
                    + pstring(TARGET_NAME) + pstring(BOARD_NAME)
                    + pstring(MANUFACTURER) + bytes(32) + bytes([0, 0])), True
        if command == MSP_STATUS:
            # 1 kHz task delta, no i2c errors, the sensors a board like this
            # has (acc, baro, gps, gyro - bits 0,1,3,5), the ARM box as bit 0
            # of the mode flags when it is armed, profile 0, no load.
            sensors = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5)
            modes = 1 if self.armed else 0
            return (struct.pack("<HHHI", 1000, 0, sensors, modes)
                    + bytes([0, 0, 0])), True
        if command == MSP_ATTITUDE:
            self.polls += 1
            roll = 150 + self.polls * 3
            pitch = -80 - self.polls
            yaw = 2700 + self.polls * 10
            return struct.pack("<hhh", roll, pitch, yaw), True
        if command == MSP_ANALOG:
            volts = 12.6 - self.polls * 0.01
            return (bytes([int(volts * 10) & 0xFF])
                    + struct.pack("<H", 210 + self.polls)
                    + struct.pack("<H", 90)
                    + struct.pack("<h", 1250)
                    + struct.pack("<H", int(volts * 100))), True
        if command == MSP_RAW_GPS:
            return (bytes([3, 11])
                    + struct.pack("<ii", 521234567, 49876543)
                    + struct.pack("<HHH", 41, 512, 2750)), True
        if command == MSP_MOTOR:
            return struct.pack("<8H", 1000 + self.polls, 1100, 1200, 1300,
                               0, 0, 0, 0), True
        if command == MSP_RC:
            return struct.pack("<8H", 992, 992, 992, 172, 992, 1811, 992,
                               992), True
        if command == MSP_EEPROM_WRITE:
            # The refusal is the whole reason this is here. Both firmwares
            # check the arming flag before writing their configuration to
            # flash, so this is the board saying no in its own voice -- which
            # is a second answer beside the client's own gate, not a
            # replacement for it.
            return (b"", False) if self.armed else (b"", True)
        return None                     # a command this board does not know

    def answer_v2(self, function, payload):
        """The MSP v2 commands.

        Returns `(payload, ok)`: `ok` False is the error frame a board sends
        for a name it does not have or a value it will not take, which is a
        reply rather than silence.

        The two firmwares answer two *different* sets of commands here, so
        which set is answered is decided by which firmware this board is being.
        """
        if self.inav:
            return self.answer_inav(function, payload)
        return self.answer_betaflight(function, payload)

    def answer_betaflight(self, function, payload):
        """Betaflight's settings: text in, text out, and one command for both
        the read and the write.

        `MSP2_CLI_SETTING` carries either a bare ``name`` (read) or
        ``name = value`` (write), and answers ``name = value`` either way --
        so **the reply to a write is itself the read-back**
        (`src/main/msp/msp.c`, which calls `cliSetSettingByName` and then
        `cliGetSettingByName` on the same name).
        """
        if function == MSP2_CLI_SETTING:
            text = payload.decode("ascii", "replace")
            name, sep, asked = text.partition("=")
            name = name.strip()
            if name not in SETTINGS:
                return b"", False
            if sep:
                # `cliSetSettingByName` **refuses rather than clamps**: a value
                # out of range, or one that is not a number at all, returns
                # false and the command becomes MSP_RESULT_ERROR. A stand-in
                # that clamped would teach a client to trust its own request,
                # which is the one thing the read-back exists to prevent.
                written = self.parse_betaflight(name, asked.strip())
                if written is None:
                    return b"", False
                SETTINGS[name][5] = written
            return ("%s = %s" % (name, self.format_betaflight(name))).encode(), True

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

    def answer_inav(self, function, payload):
        """INAV's settings, which are not Betaflight's with a name swapped in.

        The requests are the same shape -- a NUL-terminated name, or a leading
        NUL and a u16 index -- but everything after that differs: the value
        crosses as its **native binary width**, the description is a **binary
        struct** rather than windowed text, and a write is answered with an
        **empty ACK** that says nothing whatever about the value.
        """
        if function == MSP2_COMMON_SETTING:
            name = inav_name(payload)
            if name not in SETTINGS:
                return b"", False
            return self.inav_bytes(name), True

        if function == MSP2_COMMON_SET_SETTING:
            name = inav_name(payload)
            if name not in SETTINGS:
                return b"", False
            written = self.parse_inav(name, payload[len(name.encode()) + 1:])
            if written is None:
                return b"", False
            SETTINGS[name][5] = written
            # `mspSetSettingCommand` takes `dst` and never touches it
            # (`UNUSED(dst)`), so the reply is an ACK with an empty payload.
            # Nothing in it says what the board now holds -- which is exactly
            # why a client cannot call this write confirmed on its own.
            return b"", True

        if function == MSP2_COMMON_SETTING_INFO:
            name = inav_name(payload)
            if name not in SETTINGS:
                return b"", False
            pgn, kind, lo, hi, _default, _value = SETTINGS[name]
            # `mspSettingInfoCommand`, field for field: the name with its
            # terminator, the pgn, then three separate bytes sliced out of the
            # one packed `type` -- the type masked to its low three bits, the
            # section as `type & 0x38`, the mode as `type & 0xC0`, neither of
            # the last two shifted down. Then a signed min, an unsigned max, the
            # absolute index, and the two profile bytes a client may always
            # assume are there. The value follows, which is a courtesy to
            # constrained callers and not part of the description.
            out = (name.encode() + b"\x00" + struct.pack("<H", pgn)
                   + bytes([INAV_TYPE_CODE[kind], 0, 0])
                   + struct.pack("<i", lo) + struct.pack("<I", hi)
                   + struct.pack("<H", 0) + bytes([0, 0])
                   + self.inav_bytes(name))
            return out, True

        return b"", False

    def format_betaflight(self, name):
        """How `sprintValuePointer` prints a MODE_DIRECT setting: its number."""
        return "%d" % SETTINGS[name][5]

    def parse_betaflight(self, name, text):
        """The value `cliSetSettingByName` would store, or None if it would
        refuse. Integer only, because every setting in this table is one."""
        _pgn, _kind, lo, hi, _default, _value = SETTINGS[name]
        try:
            number = int(text, 10)
        except ValueError:
            return None
        return None if number < lo or number > hi else number

    def inav_bytes(self, name):
        """The setting's value at its own native width -- what
        `sbufWriteDataSafe(dst, ptr, settingGetValueSize(setting))` sends."""
        _pgn, kind, _lo, _hi, _default, value = SETTINGS[name]
        return struct.pack(INAV_PACK[kind], value)

    def parse_inav(self, name, rest):
        """The value `mspSetSettingCommand` would store, or None if it would
        refuse. Its own range check, including the asymmetry: the unsigned
        cases test the maximum only, because a `uint8_t` cannot be below zero.
        """
        _pgn, kind, lo, hi, _default, _value = SETTINGS[name]
        want = struct.calcsize(INAV_PACK[kind])
        if len(rest) < want:
            return None
        number = struct.unpack(INAV_PACK[kind], rest[:want])[0]
        if kind.startswith("uint"):
            return None if number > hi else number
        return None if number < lo or number > hi else number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-after", type=float, default=0.0,
                        help="seconds until the ARM box goes high (0 = never)")
    parser.add_argument("--inav", action="store_true",
                        help="answer as INAV 9.1.0 instead of Betaflight. Its "
                             "identity *and* its settings protocol differ: "
                             "0x1003/0x1004/0x1007 with binary typed values, "
                             "not 0x3010/0x3011 with text")
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
        body = exactly(size[0] + 1)             # command + payload
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
        payload, ok = answer
        sys.stdout.buffer.write(refuse(command) if not ok
                                else reply(command, payload))
        sys.stdout.buffer.flush()


if __name__ == "__main__":
    sys.exit(main())
