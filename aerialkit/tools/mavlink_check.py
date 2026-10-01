#!/usr/bin/env python3
"""AerialKit's MAVLink client, checked against the reference implementation.

    make proto-test        # which runs this with the rest of the clients

Everything here exists because MAVLink has three things per message that no
amount of care can guess - the message id, the `crc_extra` byte, and the field
order and padding of the payload - and a client that gets any of them wrong
*looks* fine as long as its own encoder and decoder agree with each other. So
this file does not test AerialKit's client against itself: it imports
pymavlink (`upstream/pymavlink-2.4.49/`) and compares

* the id, `crc_extra` and wire format of every message in the client's table
  against the reference's own numbers,
* the frames the client builds against the frames pymavlink builds, byte for
  byte, in both MAVLink versions,
* the client's decoding of pymavlink's frames against what pymavlink put in
  them,
* and a whole conversation against `tools/mavlink_fake_vehicle.py`, which is a
  vehicle *built on pymavlink* - so the heartbeat, the streamed state and the
  parameter list all arrive from the reference implementation.

Three of the client's message formats were wrong when this file was first run
(`PARAM_VALUE`'s sixteen-byte id, `STATUSTEXT`'s fifty-byte text and
`AUTOPILOT_VERSION`'s three byte-arrays). The reference's *generated* file has
two formats per message - a `native_format` for the older API and the
`unpacker` the wire actually uses - and the wrong one is plausible enough to
pass any check that only asks the client about itself.
"""

import os
import select
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import mavlink                                                  # noqa: E402
import msp                                                      # noqa: E402

ORACLE = os.path.normpath(os.path.join(HERE, "..", "..", "upstream",
                                       "pymavlink-2.4.49"))
FAKE = os.path.join(HERE, "mavlink_fake_vehicle.py")

#: The two commands the stand-in's fidelity is asserted with below: one it
#: carries out (`MAV_CMD_SET_MESSAGE_INTERVAL`, 511 in the reference dialect)
#: and one it does not (`MAV_CMD_DO_SET_MODE`, 176) - because "it acks" is only
#: half the fact, and the other half is that it acks a refusal as a refusal
#: rather than as silence or as success.
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_SET_MESSAGE_INTERVAL = 511

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def skip(reason):
    print("the MAVLink client: not checked here - %s" % reason)
    return 0


def oracle():
    if not os.path.isdir(ORACLE):
        return None
    sys.path.insert(0, ORACLE)
    try:
        from pymavlink.dialects.v20 import common
    except ImportError:
        return None
    return common


def parse_format(fmt):
    """`[(char, count), ...]` from a struct format - the authority for what
    each field *is*, so this check does not have to know a message's types by
    name (which is the same mistake one level up)."""
    out = []
    body = fmt[1:] if fmt[:1] in "<>=@!" else fmt
    count = ""
    for char in body:
        if char.isdigit():
            count += char
        else:
            out.append((char, int(count) if count else 1))
            count = ""
    return out


def values_for(name):
    """A value per field, from the format rather than from the field's name:
    distinct numbers, none of them zero, and the right shape for the field."""
    _id, _extra, fmt, names = mavlink.MESSAGES[name]
    shapes = parse_format(fmt)
    out = {}
    for index, field in enumerate(names):
        char, count = shapes[index]
        if field == "param_id" or char == "c":
            out[field] = b"AK"[:count]
        elif char == "s":
            out[field] = b"AerialKit"[:count]
        elif count > 1:
            out[field] = list(range(1, count + 1))
        elif char in "fd":
            out[field] = 1.5 + index
        else:
            out[field] = 1 + index
    return out


def same_frame(mine, theirs):
    """Whether two senders built the same frame.

    MAVLink v2 lets a sender *truncate* the trailing zero bytes of a payload,
    and pymavlink does - which shortens the frame *and* its length byte. So the
    comparison is the magic, everything after the length byte, and the payload
    with the trailing zeros taken off: identical messages, whatever the sender
    chose to send of them.
    """
    if mine[0] != theirs[0]:
        return False
    head = 10 if mine[0] == mavlink.MAGIC_V2 else 6
    if mine[2:head] != theirs[2:head]:
        return False
    return mine[head:head + mine[1]].rstrip(b"\x00") == \
        theirs[head:head + theirs[1]].rstrip(b"\x00")


def oracle_frame(common, name, fields):
    """The reference implementation's own bytes for one message."""
    mav = common.MAVLink(None)
    mav.srcSystem = mavlink.GCS_SYSTEM
    mav.srcComponent = mavlink.GCS_COMPONENT
    cls = getattr(common, "MAVLink_%s_message" % name.lower())
    filled = {}
    for field in cls.fieldnames:
        value = fields.get(field, 0)
        if cls.fieldtypes[cls.fieldnames.index(field)] == "char" and \
                not isinstance(value, bytes):
            value = str(value).encode()
        filled[field] = value
    return cls(**filled).pack(mav)


def main():
    common = oracle()
    if common is None:
        return skip("no pymavlink at %s (run scripts/fetch-upstreams.sh "
                    "pymavlink-2.4.49)" % ORACLE)
    ref = common            # the same module, named for what it is used as at
                            # the bottom of this file: the authority the
                            # stand-in is held to.

    print("AerialKit's MAVLink client, against pymavlink")

    # --- the table, against the reference's own numbers -------------------
    #
    # The one check that catches a remembered constant: every id, every
    # crc_extra and every field order in the client's table, against the class
    # pymavlink generated from the message definitions.
    bad = []
    for name, (msg_id, extra, fmt, names) in sorted(mavlink.MESSAGES.items()):
        cls = getattr(common, "MAVLink_%s_message" % name.lower(), None)
        if cls is None:
            bad.append("%s: not a message" % name)
            continue
        if cls.id != msg_id:
            bad.append("%s: id %u vs %u" % (name, msg_id, cls.id))
        if cls.crc_extra != extra:
            bad.append("%s: crc_extra %u vs %u" % (name, extra, cls.crc_extra))
        if tuple(cls.ordered_fieldnames) != tuple(names):
            bad.append("%s: field order" % name)
        if cls.unpacker.format != fmt:
            bad.append("%s: format %s vs %s" % (name, fmt, cls.unpacker.format))
    expect("every id, crc_extra, field order and format in the client's table "
           "is the reference's",
           not bad, " (%s)" % ("; ".join(bad[:4]) if bad else
                               "%u messages" % len(mavlink.MESSAGES)))

    # --- the frames, byte for byte ---------------------------------------
    encoded = []
    mismatched = []
    for name in sorted(mavlink.MESSAGES):
        fields = values_for(name)
        mine = mavlink.build(name, fields)
        theirs = oracle_frame(common, name, fields)
        encoded.append(name)
        if not same_frame(mine, theirs):
            mismatched.append("%s: %s vs %s" % (name, mine.hex(), theirs.hex()))
    expect("and the frames the client builds are the frames pymavlink builds, "
           "apart from v2's trailing-zero truncation",
           not mismatched,
           " (%s)" % (mismatched[0] if mismatched else
                      "%u messages, byte for byte" % len(encoded)))

    # --- and the same in the older dialect -------------------------------
    #
    # MAVLink v1 has a different header and the same payloads, and an older
    # vehicle answers v1 however it was asked - so a client that only speaks
    # v2 is a client that works on half the vehicles in the field.
    name = "GLOBAL_POSITION_INT"
    fields = values_for(name)
    mine = mavlink.build(name, fields, version=1)
    parsed, got, header = mavlink.parse(mine)
    expect("a v1 frame is written and read, and says which it was",
           parsed == name and header["version"] == 1 and
           abs(got["lat"] - fields["lat"]) < 1.0,
           " (%u bytes, version %u)" % (len(mine), header["version"]))
    oracle_v1 = oracle_frame(common, "PING", values_for("PING"))
    expect("a v2 frame from the reference is read as v2",
           mavlink.parse(oracle_v1)[2]["version"] == 2,
           " (%s)" % mavlink.parse(oracle_v1)[0])

    # --- what the client refuses -----------------------------------------
    frame = mavlink.build("ATTITUDE", values_for("ATTITUDE"))
    broken = bytearray(frame)
    broken[-1] ^= 0xFF
    try:
        mavlink.parse(bytes(broken))
        expect("a frame whose checksum is wrong is refused", False)
    except ValueError:
        expect("a frame whose checksum is wrong is refused rather than "
               "decoded", True)
    try:
        mavlink.parse(frame[:-3])
        expect("a frame that stops early is refused", False)
    except ValueError:
        expect("a frame that stops early is refused", True)
    # An id this client does not know is not an error on a real link: a
    # vehicle streams a dozen messages this window has no use for.
    other = bytearray(frame)
    other[7] = 0x7F
    other[8] = 0x7F
    try:
        mavlink.parse(bytes(other))
        expect("an unknown message id is refused, so the reader can skip it",
               False)
    except ValueError:
        expect("an unknown message id is refused, so the reader can skip it",
               True)

    # --- a whole conversation, against a vehicle pymavlink built ----------
    process = subprocess.Popen([sys.executable, FAKE],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    class Pipe:
        def read(self, count):
            return process.stdout.read(count)

        def write(self, data):
            process.stdin.write(data)
            process.stdin.flush()

        def close(self):
            process.kill()

    transport = Pipe()
    reader = msp.Deadline(transport)
    client = mavlink.Mavlink(reader.read_some, transport.write)
    client.send_heartbeat()
    identity = client.identify(5.0)
    expect("a vehicle announces itself when a ground station says hello",
           identity["autopilot"] == 3 and identity["system"] == 1 and
           identity["component"] == 1,
           " (%s)" % mavlink.describe(identity))

    # And the *other* MAVLink firmware is named as itself. PX4 and ArduPilot
    # speak the same protocol and are different words to a person, and the
    # window refuses to write in whichever name answered - so the mapping from
    # the heartbeat's `autopilot` field (MAV_AUTOPILOT_PX4 is 12 in the
    # reference dialect) is worth pinning here rather than only through the
    # stand-in's flag.
    expect("and PX4 is named PX4, not ArduPilot, when its heartbeat says so",
           mavlink.describe({"autopilot": 12, "system": 1, "component": 1,
                             "system_status": 4, "version": None})
           .startswith("PX4, system 1"),
           " (%s)" % mavlink.describe({"autopilot": 12, "system": 1,
                                       "component": 1, "system_status": 4,
                                       "version": None}))

    client.request("ATTITUDE", 5.0)
    now = mavlink.state(client)
    expect("and the state comes back from the messages it streams",
           now["roll_deg"] is not None and abs(now["lat"] - 52.1234567) < 1e-6
           and abs(now["vbat_v"] - 12.6) < 0.01 and
           now["motors"][:4] == [1500, 1520, 1480, 1510],
           " (roll %.1f, %.7f, %.1f V, motors %s)"
           % (now["roll_deg"], now["lat"], now["vbat_v"], now["motors"][:2]))

    listing = client.parameters(timeout=10.0)
    expect("and the parameter list is the vehicle's own, index by index",
           listing["count"] == 40 and len(listing["items"]) == 40 and
           listing["items"][0][0] == "RATE_RLL_P" and
           abs(listing["items"][7][1] - 2.25) < 1e-6,
           " (%s parameters, first %s)"
           % (listing["count"], listing["items"][0][0] if listing["items"]
              else None))
    transport.close()

    # --- and the parameter the link lost --------------------------------
    #
    # A frame that never arrives is the normal state of a radio link, and the
    # protocol's answer is `param_index`/`param_count`: ask for the one that is
    # missing. This vehicle drops one index from the list burst and answers it
    # only when it is asked for - so a client that does not re-ask comes back
    # with thirty-nine parameters, and one that does comes back with forty.
    process = subprocess.Popen([sys.executable, FAKE, "--drop-param", "7"],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    transport = Pipe()
    reader = msp.Deadline(transport)
    client = mavlink.Mavlink(reader.read_some, transport.write)
    client.send_heartbeat()
    client.identify(5.0)
    got = client.parameters(timeout=20.0)
    expect("a parameter frame the link lost is asked for again, not left as a "
           "hole in the table",
           got["count"] == 40 and len(got["items"]) == 40 and
           got["items"][7][0] == "WPNAV_SPEED",
           " (%s of %s, index 7 is %s)"
           % (len(got["items"]), got["count"],
              got["items"][7][0] if len(got["items"]) > 7 else None))
    transport.close()

    # --- and the stand-in is a vehicle, in the two places it can lie ------
    #
    # Everything above takes `mavlink_fake_vehicle.py` as the authority on what
    # a vehicle does. That makes two of the stand-in's own properties
    # load-bearing — that it raises `MAV_MODE_FLAG_SAFETY_ARMED` when it arms,
    # and that it answers a `COMMAND_LONG` — and until 2026-09-30 neither was
    # checked by anything here. Both were wrong for months: the armed bit was
    # set to 1, which is `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`, under a comment
    # naming the right constant; and a `COMMAND_LONG` was answered with silence.
    # Neither could be seen from this file, because neither of the two clients
    # exercised here reads `base_mode` or decodes a `COMMAND_ACK`. The browser
    # check did see them, as four red lines that were attributed to the app.
    #
    # So they are asserted here, against the reference implementation and
    # against the published constants rather than against a transcription of
    # them — which is the same rule the rest of this file follows, applied to
    # the instrument instead of the client.
    if ref is not None:
        process = subprocess.Popen([sys.executable, FAKE, "--arm-after", "0.4"],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        link = ref.MAVLink(None, srcSystem=255, srcComponent=190)
        link.robust_parsing = True
        buf = b""
        heartbeats = []
        acks = []

        def pump(seconds):
            nonlocal buf
            deadline = time.time() + seconds
            while time.time() < deadline:
                if select.select([process.stdout], [], [], 0.05)[0]:
                    chunk = os.read(process.stdout.fileno(), 4096)
                    if not chunk:
                        break
                    buf += chunk
                for message in link.parse_buffer(buf) or []:
                    kind = message.get_type()
                    if kind == "HEARTBEAT":
                        heartbeats.append((time.time(), message.base_mode))
                    elif kind == "COMMAND_ACK":
                        acks.append((message.command, message.result))
                buf = b""

        def send(command):
            process.stdin.write(ref.MAVLink_command_long_message(
                target_system=1, target_component=1, command=command,
                confirmation=0, param1=30, param2=250000.0,
                param3=0, param4=0, param5=0, param6=0, param7=0).pack(link))
            process.stdin.flush()

        pump(1.2)
        send(MAV_CMD_SET_MESSAGE_INTERVAL)
        pump(1.2)
        send(MAV_CMD_DO_SET_MODE)
        pump(0.6)
        process.kill()

        before = [mode for at, mode in heartbeats if at < heartbeats[0][0] + 0.4]
        after = [mode for at, mode in heartbeats if at > heartbeats[0][0] + 0.4]
        expect("the stand-in arms by raising the armed flag, not by setting "
               "some other bit",
               bool(before) and not any(mode & ref.MAV_MODE_FLAG_SAFETY_ARMED
                                         for mode in before)
               and bool(after) and all(mode & ref.MAV_MODE_FLAG_SAFETY_ARMED
                                       for mode in after),
               " (base_mode %s before, %s after)"
               % (before[:1], after[:1]))
        expect("and it acks a command it carries out, and one it does not",
               [ack for ack in acks
                if ack == (MAV_CMD_SET_MESSAGE_INTERVAL,
                           ref.MAV_RESULT_ACCEPTED)] != []
               and [ack for ack in acks
                    if ack == (MAV_CMD_DO_SET_MODE,
                               ref.MAV_RESULT_UNSUPPORTED)] != [],
               " (%s)" % (acks,))

    print("mavlink: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
