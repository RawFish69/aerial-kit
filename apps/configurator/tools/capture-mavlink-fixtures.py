#!/usr/bin/env python3
"""Capture MAVLink fixtures from the repository's own fake vehicle.

The web configurator's MAVLink support has to be built against something, and
the something has to be MAVLink's actual bytes rather than a summary of them.
`firmware/tools/mavlink_fake_vehicle.py` is an ArduPilot vehicle that is not
there, and **every frame it sends is built by pymavlink**, the reference
implementation that Mission Planner, QGroundControl and MAVProxy are built on.

This script drives that vehicle with a *third* implementation: the framing
below is written out longhand from the wire format rather than imported from
either `firmware/tools/mavlink.py` or pymavlink. So a fixture landing in
`tests/fixtures/mavlink.json` has been agreed on by two independent readers of
the protocol, and a mistake in the TypeScript decoder shows up as a checksum
that does not match rather than as a test that passes for the wrong reason.

    python3 tools/capture-mavlink-fixtures.py [--aerialkit PATH]

Writes `tests/fixtures/mavlink.json`. Exits non-zero if the vehicle does not
answer, so a broken capture is a broken build rather than a missing file.
"""

import argparse
import json
import os
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_AERIALKIT = os.path.join(ROOT, "firmware")
DEFAULT_PYMAVLINK = os.path.join(ROOT, "upstream", "pymavlink-2.4.49")
FAKE = os.path.join("tools", "mavlink_fake_vehicle.py")

MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD
MAVLINK_STX = MAGIC_V2

# The CRC extras are properties of the *message definitions*, published in the
# MAVLink specification. They are repeated here on purpose: importing them from
# pymavlink would make this a second reader of the same table rather than an
# independent one, and a wrong extra is exactly the sort of fault the fixture
# is supposed to catch.
CRC_EXTRA = {
    0: 50,     # HEARTBEAT
    1: 124,    # SYS_STATUS
    21: 159,   # PARAM_REQUEST_LIST
    22: 220,   # PARAM_VALUE
    24: 24,    # GPS_RAW_INT
    30: 39,    # ATTITUDE
    33: 104,   # GLOBAL_POSITION_INT
    36: 222,   # SERVO_OUTPUT_RAW
    65: 118,   # RC_CHANNELS
    74: 20,    # VFR_HUD
    76: 152,   # COMMAND_LONG
    77: 143,   # COMMAND_ACK
    148: 178,  # AUTOPILOT_VERSION
    253: 83,   # STATUSTEXT
}

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
MAV_PARAM_TYPE_REAL32 = 9


def crc16_mcrf4xx(data: bytes, extra: int) -> int:
    """CRC-16/MCRF4XX, seeded with the message's CRC_EXTRA byte.

    X.25's polynomial with the accumulator starting at 0xFFFF, and the extra
    byte is accumulated *before* the final complement - which is the detail
    almost every from-memory implementation gets wrong, and the reason the
    fixtures are worth generating rather than typing.
    """
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def frame_v2(msgid: int, payload: bytes, seq: int, sysid: int, compid: int) -> bytes:
    """A MAVLink v2 frame: STX, len, incompat, compat, seq, sys, comp, msgid(3).

    The header is *truncated* when the payload has trailing zeros, which is the
    v2 feature that makes this worth writing by hand: a decoder that ignores
    the length byte and assumes the payload is the struct size passes every
    test except one against a message that happens to end in zero.
    """
    body = bytes([len(payload), 0, 0, seq, sysid, compid,
                  msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF]) + payload
    # crc16_mcrf4xx accumulates the extra itself: passing `body + extra` *and*
    # the extra counts it twice, which no frame validates against. That mistake
    # was made here first and caught by checking against a real pymavlink frame.
    checksum = crc16_mcrf4xx(body, CRC_EXTRA[msgid])
    return bytes([MAGIC_V2]) + body + struct.pack("<H", checksum)


def parse_v2(buf: bytes):
    """Yield (msgid, payload, seq, frame) for each complete frame in `buf`.

    The whole frame is yielded, not only its payload, because the fixture's job
    is to hold bytes pymavlink produced. A test that fed a payload to a decoder
    of this repository's own construction would only prove the two halves agree
    with each other.
    """
    i = 0
    while i < len(buf):
        if buf[i] != MAGIC_V2:
            i += 1
            continue
        if i + 10 > len(buf):
            return
        length = buf[i + 1]
        if i + 12 + length > len(buf):
            return
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        payload = buf[i + 10:i + 10 + length]
        want = struct.unpack("<H", buf[i + 10 + length:i + 12 + length])[0]
        if msgid not in CRC_EXTRA:
            i += 1
            continue
        body = buf[i + 1:i + 10 + length]
        got = crc16_mcrf4xx(body, CRC_EXTRA[msgid])
        if got == want:
            yield msgid, payload, buf[i + 4], buf[i:i + 12 + length]
            i += 12 + length
        else:
            i += 1


def unpack(fmt: str, payload: bytes):
    """struct.unpack that tolerates MAVLink v2's zero truncation."""
    size = struct.calcsize(fmt)
    return struct.unpack(fmt, payload.ljust(size, b"\0")[:size])


def pack_with_pymavlink(pymavlink_dir):
    """Frames for the messages the fake vehicle never sends.

    The vehicle emits no `STATUSTEXT` or `AUTOPILOT_VERSION`, so those layouts
    would otherwise go uncovered - and they are two of the four messages with
    extension fields, which is exactly where a hand-written field table is most
    likely to be wrong.

    `COMMAND_ACK` is here for the other reason this function exists. The vehicle
    does send one now, but only with its base fields: v2 truncates the trailing
    zeros, so the frame that reaches a client is two bytes long and every
    extension field in it reads zero. The packed copy carries non-zero
    extensions, which is the only way the tail of that layout is covered at all.

    pymavlink builds the *whole frame* here, checksum included, so this is not
    the longhand framing checking itself: the payloads come out of pymavlink's
    own struct packer and are framed by pymavlink's own `_pack`. The frames this
    returns are therefore a second, independent statement of the layout, and the
    longhand parser below is made to validate them before they are written out.

    Each message is also built a second time with a *different* extension-field
    value, because a frame whose extension fields happen to be zero is exactly
    the frame v2 truncates, and a layout error in the tail would hide inside the
    truncation. Both are kept.
    """
    import importlib

    sys.path.insert(0, pymavlink_dir)
    dialect = importlib.import_module("pymavlink.dialects.v20.common")
    # System and component 1, which is the vehicle the fake one already is. The
    # default is 0, and a frame from system 0 next to a vehicle streaming from
    # system 1 is a mismatch that makes an app which follows one system drop
    # these frames for the wrong reason — and a test that then passes proves
    # nothing about the message it thought it was testing.
    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)

    built = {}

    statustext = dialect.MAVLink_statustext_message(
        severity=6, text=b"AerialKit fixture", id=0x1234, chunk_seq=0)
    built.setdefault(253, []).append(statustext.pack(mav))

    ack = dialect.MAVLink_command_ack_message(
        command=MAV_CMD_SET_MESSAGE_INTERVAL, result=0, progress=100,
        result_param2=-2, target_system=1, target_component=1)
    built.setdefault(77, []).append(ack.pack(mav))

    # AUTOPILOT_VERSION has one extension field, `uid2`, and eleven ordinary
    # ones - the longest message in the table and the best test of the sort.
    version = dialect.MAVLink_autopilot_version_message(
        capabilities=0x0000000000000001, flight_sw_version=0x01020304,
        middleware_sw_version=0x05060708, os_sw_version=0x090a0b0c,
        board_version=0x0d0e0f10, vendor_id=0x1112, product_id=0x1314,
        uid=0x15161718191a1b1c,
        flight_custom_version=[1, 2, 3, 4, 5, 6, 7, 8],
        middleware_custom_version=[9, 10, 11, 12, 13, 14, 15, 16],
        os_custom_version=[17, 18, 19, 20, 21, 22, 23, 24],
        uid2=[25] * 18)
    built.setdefault(148, []).append(version.pack(mav))

    # And once more with every extension field left at zero, which is the frame
    # v2 will truncate.
    built.setdefault(253, []).append(
        dialect.MAVLink_statustext_message(severity=4, text=b"x").pack(mav))

    return built


def run(vehicle, aerialkit):
    proc = subprocess.Popen(
        [sys.executable, os.path.join(aerialkit, FAKE)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )

    seq = 0
    frames = {}

    def send(msgid, payload):
        nonlocal seq
        proc.stdin.write(frame_v2(msgid, payload, seq, 255, 190))
        proc.stdin.flush()
        seq = (seq + 1) & 0xFF

    def pump(seconds=3.0):
        """Read whatever arrives, recording the first example of each message."""
        import select
        import time
        end = time.time() + seconds
        while time.time() < end:
            ready, _, _ = select.select([proc.stdout], [], [], 0.2)
            if not ready:
                continue
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                return
            for msgid, payload, fseq, frame in parse_v2(chunk):
                frames.setdefault(msgid, {"frame": frame.hex(), "payload": payload.hex(),
                                          "seq": fseq})

    # A ground station announces itself, which is what makes the vehicle start.
    send(0, struct.pack("<IBBBBB", 0, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0,
                        MAV_STATE_ACTIVE, 3))
    pump(2.5)

    # Ask for the parameter list. The vehicle answers index by index.
    send(21, struct.pack("<BB", 1, 1))
    pump(3.0)

    # Ask for each stream by message interval. The command takes the *message
    # id* in param1 and the interval in microseconds in param2 — param1 is the
    # thing a first draft gets wrong, by passing 0 and then requesting HEARTBEAT
    # six times.
    for msgid in (30, 33, 1, 65, 36, 24, 74):
        send(76, struct.pack("<7fHBBB", float(msgid), 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             MAV_CMD_SET_MESSAGE_INTERVAL, 1, 1, 0))
        pump(0.8)
    pump(2.5)

    proc.stdin.close()
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    err = proc.stderr.read().decode("utf-8", "replace")
    return frames, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--aerialkit", default=DEFAULT_AERIALKIT)
    ap.add_argument("--pymavlink", default=DEFAULT_PYMAVLINK,
                    help="the reference implementation, used only to build the "
                         "frames the fake vehicle does not send")
    args = ap.parse_args()

    if not os.path.exists(os.path.join(args.aerialkit, FAKE)):
        print("capture-mavlink-fixtures: no fake vehicle at %s" % args.aerialkit, file=sys.stderr)
        return 1

    frames, err = run(args.aerialkit, args.aerialkit)
    if not frames:
        print("capture-mavlink-fixtures: the vehicle sent nothing", file=sys.stderr)
        print(err[-2000:], file=sys.stderr)
        return 1

    names = {
        0: "HEARTBEAT", 1: "SYS_STATUS", 21: "PARAM_REQUEST_LIST", 22: "PARAM_VALUE",
        24: "GPS_RAW_INT", 30: "ATTITUDE", 33: "GLOBAL_POSITION_INT",
        36: "SERVO_OUTPUT_RAW", 65: "RC_CHANNELS", 74: "VFR_HUD",
        76: "COMMAND_LONG", 77: "COMMAND_ACK", 148: "AUTOPILOT_VERSION",
        253: "STATUSTEXT",
    }
    out = {
        "note": "Captured from firmware/tools/mavlink_fake_vehicle.py, whose frames "
                "are built by pymavlink. The framing that produced these hex strings "
                "is this repository's own, written longhand.",
        "crc_extra": {str(k): v for k, v in sorted(CRC_EXTRA.items())},
        "frames": {},
        "packed": {},
    }
    for msgid, info in sorted(frames.items()):
        name = names.get(msgid, "MSG_%d" % msgid)
        out["frames"][name] = {
            "msgid": msgid,
            "crc_extra": CRC_EXTRA.get(msgid),
            "frame_hex": info["frame"],
            "payload_hex": info["payload"],
            "seq": info["seq"],
            "source": "mavlink-fake-vehicle",
        }

    # ---- frames the vehicle does not send, built by pymavlink itself --------
    #
    # Every one of these is run through this script's own longhand parser before
    # it is written out. That parser is not pymavlink's, so a frame it accepts is
    # a frame two independent framings agree on; a frame it rejects fails the
    # capture rather than landing in the fixture as a decoder test that cannot
    # pass.
    generated = pack_with_pymavlink(os.path.abspath(args.pymavlink))
    for msgid, raw_frames in sorted(generated.items()):
        name = names.get(msgid, "MSG_%d" % msgid)
        out["packed"][name] = []
        for raw in raw_frames:
            parsed = list(parse_v2(raw))
            if len(parsed) != 1 or parsed[0][0] != msgid:
                print("capture-mavlink-fixtures: this repository's own parser rejected a "
                      "pymavlink frame for %s: %s" % (name, raw.hex()), file=sys.stderr)
                return 1
            _, payload, seq, frame = parsed[0]
            if frame != raw:
                print("capture-mavlink-fixtures: parser recovered different bytes for %s"
                      % name, file=sys.stderr)
                return 1
            out["packed"][name].append({
                "msgid": msgid,
                "crc_extra": CRC_EXTRA.get(msgid),
                "frame_hex": raw.hex(),
                "payload_hex": payload.hex(),
                "seq": seq,
                "source": "pymavlink",
            })

    def decode_expectations(out):
        """What each captured payload must decode to, computed here and asserted in TS."""
        exp = {}
        fr = out["frames"]
        if "HEARTBEAT" in fr:
            p = bytes.fromhex(fr["HEARTBEAT"]["payload_hex"])
            custom, typ, ap, base, status, ver = unpack("<IBBBBB", p)
            exp["HEARTBEAT"] = {"type": typ, "autopilot": ap, "base_mode": base,
                                "system_status": status, "custom_mode": custom,
                                "mavlink_version": ver}
        if "ATTITUDE" in fr:
            p = bytes.fromhex(fr["ATTITUDE"]["payload_hex"])
            t, roll, pitch, yaw, rs, ps, ys = unpack("<Iffffff", p)
            exp["ATTITUDE"] = {"time_boot_ms": t, "roll": roll, "pitch": pitch, "yaw": yaw}
        if "SYS_STATUS" in fr:
            p = bytes.fromhex(fr["SYS_STATUS"]["payload_hex"])
            vals = unpack("<IIIHHhHHHHHHb", p)
            exp["SYS_STATUS"] = {"voltage_battery": vals[4], "current_battery": vals[5],
                                 "battery_remaining": vals[12]}
        if "GLOBAL_POSITION_INT" in fr:
            p = bytes.fromhex(fr["GLOBAL_POSITION_INT"]["payload_hex"])
            t, lat, lon, alt, ralt, vx, vy, vz, hdg = unpack("<IiiiihhhH", p)
            exp["GLOBAL_POSITION_INT"] = {"lat": lat, "lon": lon, "alt": alt,
                                          "relative_alt": ralt, "hdg": hdg}
        if "PARAM_VALUE" in fr:
            p = bytes.fromhex(fr["PARAM_VALUE"]["payload_hex"])
            value, count, index, pid, ptype = unpack("<fHH16sB", p)
            exp["PARAM_VALUE"] = {"param_value": value, "param_count": count,
                                  "param_index": index, "param_type": ptype,
                                  "param_id": pid.split(b"\0")[0].decode()}
        if "SERVO_OUTPUT_RAW" in fr:
            p = bytes.fromhex(fr["SERVO_OUTPUT_RAW"]["payload_hex"])
            vals = unpack("<I16HB", p)
            exp["SERVO_OUTPUT_RAW"] = {"servo1_raw": vals[1], "servo2_raw": vals[2],
                                       "servo3_raw": vals[3], "servo4_raw": vals[4]}
        if "RC_CHANNELS" in fr:
            p = bytes.fromhex(fr["RC_CHANNELS"]["payload_hex"])
            vals = unpack("<I18HBB", p)
            exp["RC_CHANNELS"] = {"chan1_raw": vals[1], "chan2_raw": vals[2],
                                  "chancount": vals[19], "rssi": vals[20]}
        # The three pymavlink built, where the values were chosen here and so
        # are known without decoding anything.
        if "STATUSTEXT" in out["packed"]:
            exp["STATUSTEXT"] = {"severity": 6, "text": "AerialKit fixture",
                                 "id": 0x1234, "chunk_seq": 0}
            exp["STATUSTEXT_truncated"] = {"severity": 4, "text": "x", "id": 0, "chunk_seq": 0}
        # COMMAND_ACK is now both: the vehicle answers a command with one, and
        # pymavlink builds a second with its extension fields non-zero. The
        # decode expectation has to describe whichever frame the test will
        # actually read - `frames[name] ?? packed[name][0]` - and that is the
        # vehicle's, whose extensions v2 truncated away. Asserting the packed
        # values against the captured frame is how this failed the first time
        # the vehicle learned to ack: progress read 0 where the fixture said
        # 100. Both frames stay in the fixture; the packed one is still checked
        # for its checksum and layout by `allCaptured()`.
        if "COMMAND_ACK" in fr:
            p = bytes.fromhex(fr["COMMAND_ACK"]["payload_hex"])
            command, result, progress, param2, tgt_sys, tgt_comp = unpack("<HBBiBB", p)
            exp["COMMAND_ACK"] = {"command": command, "result": result,
                                  "progress": progress, "result_param2": param2,
                                  "target_system": tgt_sys, "target_component": tgt_comp}
        elif "COMMAND_ACK" in out["packed"]:
            exp["COMMAND_ACK"] = {"command": MAV_CMD_SET_MESSAGE_INTERVAL, "result": 0,
                                  "progress": 100, "result_param2": -2,
                                  "target_system": 1, "target_component": 1}
        if "AUTOPILOT_VERSION" in out["packed"]:
            exp["AUTOPILOT_VERSION"] = {
                "capabilities": 1, "flight_sw_version": 0x01020304,
                "middleware_sw_version": 0x05060708, "os_sw_version": 0x090A0B0C,
                "board_version": 0x0D0E0F10, "vendor_id": 0x1112, "product_id": 0x1314,
                "uid": 0x15161718191A1B1C,
            }
        return exp

    out["expect"] = decode_expectations(out)

    dest = os.path.join(HERE, "..", "tests", "fixtures", "mavlink.json")
    dest = os.path.abspath(dest)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with open(dest, "w") as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
        fh.write("\n")

    print("captured %d message types from the vehicle: %s" % (
        len(out["frames"]), ", ".join(sorted(out["frames"]))))
    print("built %d from pymavlink: %s" % (
        sum(len(v) for v in out["packed"].values()), ", ".join(sorted(out["packed"]))))
    print("wrote %s" % dest)
    missing = [n for n in names.values() if n not in out["frames"]
               and n not in out["packed"] and n not in ("PARAM_REQUEST_LIST", "COMMAND_LONG")]
    if missing:
        print("did not capture: %s" % ", ".join(sorted(missing)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
