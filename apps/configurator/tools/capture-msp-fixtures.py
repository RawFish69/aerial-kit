#!/usr/bin/env python3
"""Capture MSP fixtures from the repository's own Betaflight stand-in.

The web configurator's MSP support has to be built against something, and the
something has to be Betaflight's actual bytes rather than a summary of them.
`aerialkit/tools/msp_fake_board.py` is a board that is not there, answering with
Betaflight's own payload layouts, and `aerialkit/tools/msp.py` is a client that
already speaks to it. This script drives one with a *third* implementation --
the framing below is written out longhand rather than imported -- so a fixture
that lands in `tests/fixtures/msp.json` has been agreed on by two independent
readers of the protocol, and a mistake in either shows up as a checksum that
does not match rather than as a test that passes for the wrong reason.

    python3 tools/capture-msp-fixtures.py [--aerialkit PATH]

Writes `tests/fixtures/msp.json`. Exits non-zero if the stand-in does not
answer, so a broken capture is a broken build rather than a missing file.
"""

import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_FAKE = os.path.join(ROOT, "aerialkit", "tools", "msp_fake_board.py")

# MSP v1, from `msp_protocol.h`. The numbers are the protocol.
API_VERSION = 1
FC_VARIANT = 2
FC_VERSION = 3
BOARD_INFO = 4
STATUS = 101
MOTOR = 104
RC = 105
RAW_GPS = 106
ATTITUDE = 108
ANALOG = 110

# MSP v2, from `msp_protocol_v2_betaflight.h`. There is no "list the settings"
# request in the protocol: every command here takes a *name*.
CLI_SETTING = 0x3010
CLI_SETTING_INFO = 0x3011


def crc8_dvb_s2(data):
    """Poly 0xD5, init 0 -- Betaflight's `crc8_calc`."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def request_v1(command, payload=b""):
    value = (len(payload) + 1) & 0xFF
    value ^= command
    for byte in payload:
        value ^= byte
    return b"$M<" + bytes([len(payload) + 1, command]) + payload + bytes([value])


def request_v2(function, payload=b""):
    header = bytes([0, function & 0xFF, (function >> 8) & 0xFF,
                    len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    return b"$X<" + header + payload + bytes([crc8_dvb_s2(header + payload)])


def expect(stream, count, timeout=5.0):
    """`count` bytes, or None if they do not arrive.

    A capture that blocks forever on a board that does not answer reports
    nothing at all, which is the worst outcome for a script whose whole job is
    to say what the board said. `msp.py` has a deadline for the same reason: a
    board that stays silent is the normal case for the *first* question.
    """
    import select
    out = b""
    while len(out) < count:
        ready, _, _ = select.select([stream], [], [], timeout)
        if not ready:
            return None
        piece = stream.read(count - len(out))
        if not piece:
            return None
        out += piece
    return out


def read_reply(stream, is_v2):
    """One whole reply frame, or None if the board stopped talking."""
    magic = expect(stream, 3)
    if magic is None or (magic[:2] != b"$M" and magic[:2] != b"$X"):
        return None
    if is_v2:
        head = expect(stream, 5)
        if head is None:
            return None
        size = head[3] | (head[4] << 8)
        body = expect(stream, size + 1)
        if body is None:
            return None
        return magic + head + body
    size = expect(stream, 1)
    if size is None:
        return None
    body = expect(stream, size[0] + 1)
    if body is None:
        return None
    return magic + size + body


class Board:
    """The stand-in, as a subprocess that answers one request at a time."""

    def __init__(self, path, inav=False):
        command = [sys.executable, path]
        if inav:
            command.append("--inav")
        # Unbuffered, and that is not a performance choice. `expect()` waits on
        # the file descriptor with `select`, and a `BufferedReader` in front of
        # that descriptor reads *ahead*: the bytes for a frame are in Python's
        # buffer while the descriptor is empty, so `select` reports "nothing
        # yet" about data that has already arrived, and the capture fails on a
        # frame it is holding. `bufsize=0` hands back raw pipes, where a byte
        # that has been read is a byte that is gone.
        self.proc = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, bufsize=0,
        )

    def ask(self, frame, is_v2=False):
        self.proc.stdin.write(frame)
        self.proc.stdin.flush()
        return read_reply(self.proc.stdout, is_v2)

    def close(self):
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        self.proc.wait(timeout=5)


def hexed(frame):
    return frame.hex() if frame is not None else None


def capture(board, note):
    """Every read a window does, in the order it does them."""
    out = {}

    def one(key, frame, is_v2=False, text=None):
        reply = board.ask(frame, is_v2)
        if reply is None:
            raise SystemExit(
                "the stand-in did not answer %s -- a fixture that is not there "
                "is worse than a missing one, because a client tested against "
                "it passes and a board then refuses" % key
            )
        out[key] = {
            "note": text or key,
            "request": hexed(frame),
            "reply": hexed(reply),
        }
        return reply

    # `text=` on every one of these, and the first draft did not: the notes
    # were passed positionally into `is_v2`, so every v1 frame was read as a v2
    # one and the capture failed on the first request it made.
    one("api_version", request_v1(API_VERSION),
        text="what protocol version the board speaks")
    one("fc_variant", request_v1(FC_VARIANT),
        text="BTFL or INAV -- the string that decides which name list applies")
    one("fc_version", request_v1(FC_VERSION), text="the release, as three bytes")
    one("board_info", request_v1(BOARD_INFO), text="board identifier and target name")
    one("status", request_v1(STATUS),
        text="arming flags, sensor health, the current mode box, and the profile")
    one("attitude", request_v1(ATTITUDE), text="roll, pitch, yaw in tenths of a degree")
    one("analog", request_v1(ANALOG),
        text="pack voltage and current, in the board's units")
    one("raw_gps", request_v1(RAW_GPS), text="fix, satellites and position")
    one("motor", request_v1(MOTOR), text="four outputs, in the board's units")
    one("rc", request_v1(RC), text="the channel values the board is flying on")

    # The settings half. A v2 command is 16 bits wide, which is why these are
    # not above with the others -- 0x3010 does not fit under an 8-bit command.
    #
    # The two requests do *not* take the name the same way, and the first draft
    # sent both with a trailing NUL. `MSP2_CLI_SETTING` takes the bare text --
    # the board strips whitespace, not NUL, so a copied-over terminator asks
    # for a setting whose name has a NUL in it and gets a refusal.
    # `MSP2_CLI_SETTING_INFO` is the one that is `name\0` plus a u16 offset.
    one("setting_known", request_v2(CLI_SETTING, b"failsafe_throttle"), True,
        text="ask a setting by name; the board answers 'name = value'")
    one("setting_info_known", request_v2(CLI_SETTING_INFO, b"failsafe_throttle\x00"), True,
        text="the same name's description: pgn, type, min, max, default")
    one("setting_absent", request_v2(CLI_SETTING, b"no_such_setting_at_all"), True,
        text="a name the board does not have -- a refusal, not an empty answer")

    out["_captured"] = {
        "from": "aerialkit/tools/msp_fake_board.py",
        "note": note,
        "protocol": "MSP v1 for state, MSP v2 for settings",
    }
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake-board", default=DEFAULT_FAKE)
    parser.add_argument("--out", default=os.path.join(HERE, "..", "tests", "fixtures", "msp.json"))
    args = parser.parse_args()

    if not os.path.exists(args.fake_board):
        raise SystemExit("no Betaflight stand-in at %s" % args.fake_board)

    every = {}
    for variant, inav in (("betaflight", False), ("inav", True)):
        board = Board(args.fake_board, inav=inav)
        try:
            every[variant] = capture(board, "%s stand-in" % variant)
        finally:
            board.close()

    # The two variants must actually differ, or the capture has proved nothing
    # about the string that selects a setting-name table.
    bf = bytes.fromhex(every["betaflight"]["fc_variant"]["reply"])
    iv = bytes.fromhex(every["inav"]["fc_variant"]["reply"])
    if bf == iv:
        raise SystemExit("both stand-ins answered the same variant string (%s)" % bf.hex())
    if b"BTFL" not in bf or b"INAV" not in iv:
        raise SystemExit("the variant strings are not the ones Betaflight and INAV use")

    with open(args.out, "w") as handle:
        json.dump(every, handle, indent=2, sort_keys=True)
        handle.write("\n")

    count = sum(len([k for k in v if not k.startswith("_")]) for v in every.values())
    print("wrote %s (%d frames across 2 variants)" % (args.out, count))
    print("  betaflight answers %s, inav answers %s" % (
        every["betaflight"]["fc_variant"]["reply"][6:14],
        every["inav"]["fc_variant"]["reply"][6:14],
    ))
    reply = bytes.fromhex(every["betaflight"]["setting_known"]["reply"])
    # A v2 reply is `$X>` + flags + function(2) + size(2) = eight header bytes,
    # then the payload, then the CRC. So the text starts at 8.
    print("  failsafe_throttle -> %s" % reply[8:-1].decode("latin1"))
    info = bytes.fromhex(every["betaflight"]["setting_info_known"]["reply"])
    # That payload is a u16 total length and then the window, so the text is
    # at 10 -- and it is a *window*, which is the point of the command.
    print("  its description   -> %s..." % info[10:-1].decode("latin1").replace("\n", " "))
    return 0


if __name__ == "__main__":
    sys.exit(main())
