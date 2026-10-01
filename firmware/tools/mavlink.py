#!/usr/bin/env python3
"""MAVLink - the protocol ArduPilot and PX4 speak, read from here.

    tools/mavlink.py --port /dev/ttyACM0            # what is on the other end?
    tools/mavlink.py --port /dev/ttyACM0 --watch    # and what is it doing?
    tools/mavlink.py --port /dev/ttyACM0 --params   # and every parameter

**Why this file exists.** This window talks to AerialKit's own protocol and now
to MSP, which is Betaflight and INAV; ArduPilot and PX4 are the other two of
the five firmwares the owner named, and they speak MAVLink. That makes this the
third and last protocol the configurator needs (`docs/27-configurator.md`).

**Where the bytes come from.** MAVLink is a *self-describing* wire format with
per-message constants that no amount of care can guess: message ids, the
`crc_extra` byte each message contributes to the frame's checksum, and the
field order and padding of every payload. All three are read from
`upstream/pymavlink-2.4.49/` - the reference implementation, kept beside the
firmware for exactly this - and `tools/mavlink_check.py` compares this file's
frames against pymavlink's byte for byte rather than trusting the table.

The frame, MAVLink v2 (`pymavlink/dialects/v20/common.py` and mavlink.io):

    FD  len  incompat  compat  seq  sysid  compid  msgid(3, LE)  payload  crc(2)

and v1, which older vehicles still send:

    FE  len  seq  sysid  compid  msgid(1)  payload  crc(2)

The CRC is X.25 (poly 0x1021, init 0xFFFF) over everything from `len` to the
end of the payload, **plus one extra byte**: the message's own `crc_extra`,
which is what makes a MAVLink parser refuse a frame whose id and payload do not
belong together.

**Read-only.** Like the MSP client, and for the same reason: parameter writes to
somebody else's flight controller are how a tool crashes an aircraft. There is
no `PARAM_SET` here, no `COMMAND_LONG` that changes anything, and the only
commands sent are the ones a *ground station* has to send to be answered - a
heartbeat, and a request for the messages this window reads.
"""

import argparse
import struct
import sys
import time

MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD

# System and component ids. 255 is the "ground station" address every MAVLink
# tool uses for itself, and the autopilot's heartbeat is what makes it start
# talking to whoever asked.
GCS_SYSTEM = 255
GCS_COMPONENT = 0

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8

MAV_CMD_SET_MESSAGE_INTERVAL = 511

# The messages this file knows. The three constants per message are the ones
# from `pymavlink/dialects/v20/common.py`: the id, the `crc_extra`, and the
# struct format of the payload (which is the *ordered* field list - MAVLink
# sorts fields by size, and getting that wrong is a frame that decodes into
# plausible nonsense).
#
#   name: (id, crc_extra, struct format, ordered field names)
MESSAGES = {
    "HEARTBEAT": (0, 50, "<IBBBBB",
                  ("custom_mode", "type", "autopilot", "base_mode",
                   "system_status", "mavlink_version")),
    "SYS_STATUS": (1, 124, "<IIIHHhHHHHHHb",
                   ("onboard_control_sensors_present",
                    "onboard_control_sensors_enabled",
                    "onboard_control_sensors_health", "load",
                    "voltage_battery", "current_battery", "drop_rate_comm",
                    "errors_comm", "errors_count1", "errors_count2",
                    "errors_count3", "errors_count4", "battery_remaining")),
    "SYSTEM_TIME": (2, 137, "<QI", ("time_unix_usec", "time_boot_ms")),
    "PING": (4, 237, "<QIBB",
             ("time_usec", "seq", "target_system", "target_component")),
    "PARAM_REQUEST_LIST": (21, 159, "<BB",
                           ("target_system", "target_component")),
    # Asking for one parameter again, by index - which is what `param_index`
    # and `param_count` are for, and what a client does about a frame that was
    # lost on the way. A ground station that only ever asks for the whole list
    # is a ground station that shows a table with holes in it whenever the link
    # drops one.
    "PARAM_REQUEST_READ": (20, 214, "<hBB16s",
                           ("param_index", "target_system",
                            "target_component", "param_id")),
    "PARAM_VALUE": (22, 220, "<fHH16sB",
                    ("param_value", "param_count", "param_index", "param_id",
                     "param_type")),
    "GPS_RAW_INT": (24, 24, "<QiiiHHHHBBiIIIIH",
                    ("time_usec", "lat", "lon", "alt", "eph", "epv", "vel",
                     "cog", "fix_type", "satellites_visible", "alt_ellipsoid",
                     "h_acc", "v_acc", "vel_acc", "hdg_acc", "yaw")),
    "ATTITUDE": (30, 39, "<Iffffff",
                 ("time_boot_ms", "roll", "pitch", "yaw", "rollspeed",
                  "pitchspeed", "yawspeed")),
    "GLOBAL_POSITION_INT": (33, 104, "<IiiiihhhH",
                            ("time_boot_ms", "lat", "lon", "alt",
                             "relative_alt", "vx", "vy", "vz", "hdg")),
    "SERVO_OUTPUT_RAW": (36, 222, "<IHHHHHHHHBHHHHHHHH",
                         ("time_usec", "servo1_raw", "servo2_raw",
                          "servo3_raw", "servo4_raw", "servo5_raw",
                          "servo6_raw", "servo7_raw", "servo8_raw", "port",
                          "servo9_raw", "servo10_raw", "servo11_raw",
                          "servo12_raw", "servo13_raw", "servo14_raw",
                          "servo15_raw", "servo16_raw")),
    "RC_CHANNELS": (65, 118, "<IHHHHHHHHHHHHHHHHHHBB",
                    ("time_boot_ms", "chan1_raw", "chan2_raw", "chan3_raw",
                     "chan4_raw", "chan5_raw", "chan6_raw", "chan7_raw",
                     "chan8_raw", "chan9_raw", "chan10_raw", "chan11_raw",
                     "chan12_raw", "chan13_raw", "chan14_raw", "chan15_raw",
                     "chan16_raw", "chan17_raw", "chan18_raw", "chancount",
                     "rssi")),
    "VFR_HUD": (74, 20, "<ffffhH",
                ("airspeed", "groundspeed", "alt", "climb", "heading",
                 "throttle")),
    "COMMAND_LONG": (76, 152, "<fffffffHBBB",
                     ("param1", "param2", "param3", "param4", "param5",
                      "param6", "param7", "command", "target_system",
                      "target_component", "confirmation")),
    "AUTOPILOT_VERSION": (148, 178, "<QQIIIIHH8B8B8B18B",
                          ("capabilities", "uid", "flight_sw_version",
                           "middleware_sw_version", "os_sw_version",
                           "board_version", "vendor_id", "product_id",
                           "flight_custom_version",
                           "middleware_custom_version", "os_custom_version",
                           "uid2")),
    "STATUSTEXT": (253, 83, "<B50sHB",
                   ("severity", "text", "id", "chunk_seq")),
}

BY_ID = {value[0]: (name, value[1], value[2], value[3])
         for name, value in MESSAGES.items()}


def x25(data):
    """The checksum MAVLink uses: X.25, poly 0x1021, init 0xFFFF.

    Its `crc_extra` argument is the message's own byte - see the frame note at
    the top of this file.
    """
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc_extra(name):
    return MESSAGES[name][1]


def frame_crc(body, name):
    """The CRC of a frame's `len`..payload bytes plus the message's extra."""
    return x25(bytes(body) + bytes([crc_extra(name)]))


def build(name, fields, sequence=0, system=GCS_SYSTEM, component=GCS_COMPONENT,
          version=2):
    """One frame, ready for the wire.

    `fields` is a dict by name; anything missing is zero, which is what a real
    sender does for the fields it has no value for (MAVLink's own encoders take
    every field, and a zero is a value rather than an absence).
    """
    if name not in MESSAGES:
        raise KeyError("no such message in this client: %s" % name)
    msg_id, _extra, fmt, names = MESSAGES[name]
    values = []
    for field in names:
        value = fields.get(field, 0)
        if isinstance(value, str):          # PARAM_VALUE's param_id is a char[16]
            value = value.encode()[:15]
        if isinstance(value, (list, tuple)):
            # The byte arrays (`8B`, `18B`): a list is what both struct.pack
            # and the reference's own encoder take for them.
            values.extend(value)
        else:
            values.append(value)
    payload = struct.pack(fmt, *values)

    if version == 2:
        body = bytes([len(payload), 0, 0, sequence & 0xFF, system & 0xFF,
                      component & 0xFF,
                      msg_id & 0xFF, (msg_id >> 8) & 0xFF,
                      (msg_id >> 16) & 0xFF])
        crc = frame_crc(body + payload, name)
        return bytes([MAGIC_V2]) + body + payload + struct.pack("<H", crc)

    body = bytes([len(payload), sequence & 0xFF, system & 0xFF,
                  component & 0xFF, msg_id & 0xFF])
    crc = frame_crc(body + payload, name)
    return bytes([MAGIC_V1]) + body + payload + struct.pack("<H", crc)


def parse(frame):
    """`(name, fields, header)` from a whole frame, checksum verified.

    v1 and v2 both arrive on real links: a modern autopilot answers v2 to a v2
    ground station, and an older one answers v1 however it was asked, so a
    reader that only knows one of them is a reader that works on half the
    vehicles in the field.
    """
    if len(frame) < 8:
        raise ValueError("too short to be a MAVLink frame: %r" % frame[:8])
    magic = frame[0]
    if magic == MAGIC_V2:
        length, incompat, compat = frame[1], frame[2], frame[3]
        seq, system, component = frame[4], frame[5], frame[6]
        msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
        head = 10
        if incompat != 0:
            raise ValueError("a v2 frame with incompat flags 0x%02x, which "
                             "this reader does not know" % incompat)
    elif magic == MAGIC_V1:
        length, seq, system, component = frame[1], frame[2], frame[3], frame[4]
        msg_id = frame[5]
        head = 6
        compat = 0
    else:
        raise ValueError("not a MAVLink frame: magic 0x%02x" % magic)

    if msg_id not in BY_ID:
        raise ValueError("message id %u is not one this client knows" % msg_id)
    name, _extra, fmt, names = BY_ID[msg_id]
    payload = frame[head:head + length]
    if len(payload) != length:
        raise ValueError("frame says %u payload bytes and carries %u"
                         % (length, len(payload)))
    crc = struct.unpack_from("<H", frame, head + length)[0]
    if crc != frame_crc(frame[1:head + length], name):
        raise ValueError("bad checksum in %s" % name)

    # A v2 sender may truncate the trailing zero bytes, so the payload is
    # padded back to the struct's size before unpacking - which is what the
    # reference implementation does too.
    full = struct.calcsize(fmt)
    values = struct.unpack(fmt, payload + bytes(full - len(payload)))
    fields = dict(zip(names, values))
    text = fields.get("param_id")
    if isinstance(text, bytes):
        fields["param_id"] = text.split(b"\x00")[0].decode("ascii", "replace")
    return name, fields, {"version": 2 if magic == MAGIC_V2 else 1,
                          "system": system, "component": component,
                          "sequence": seq, "compat": compat}


class Timeout(IOError):
    """The vehicle did not say anything in time."""


class Mavlink:
    """A ground station's side of the conversation, read-only.

    `read` is the same timeout-aware callable the MSP client takes (see
    `msp.py`): a vehicle that is quiet is the normal state of a link before
    anything has been asked for, so it is a value rather than a hang.
    """

    def __init__(self, read, write, system=GCS_SYSTEM, component=GCS_COMPONENT,
                 timeout=2.0):
        self.read = read
        self.write = write
        self.system = system
        self.component = component
        self.timeout = timeout
        self.sequence = 0
        self.frames = 0
        self.dropped = 0        # bytes that were not part of a frame
        self.unknown = 0        # frames whose message id this client does not read
        self.vehicle = None     # (system, component) of the last heartbeat
        self.heartbeats = 0

    def send(self, name, fields=None, version=2):
        data = build(name, fields or {}, self.sequence, self.system,
                     self.component, version)
        self.sequence = (self.sequence + 1) & 0xFF
        self.write(data)

    def send_heartbeat(self, version=2):
        """What a ground station says to be answered: a vehicle streams to a
        system it has heard from, and says nothing at all to one it has not."""
        self.send("HEARTBEAT",
                  {"type": MAV_TYPE_GCS, "autopilot": MAV_AUTOPILOT_INVALID,
                   "mavlink_version": 3},
                  version=version)

    def _read_frame(self, timeout):
        """The next frame, or None - and it *resynchronises* rather than
        giving up: a link that carries a vehicle's console output (which this
        one does - the same UART the console is on) has text between frames."""
        deadline = time.time() + timeout

        def byte():
            left = deadline - time.time()
            if left <= 0:
                return None
            data = self.read(1, left)
            return data if data else None

        while True:
            first = byte()
            if first is None:
                return None
            if first[0] not in (MAGIC_V1, MAGIC_V2):
                self.dropped += 1
                continue
            magic = first[0]
            head = 10 if magic == MAGIC_V2 else 6
            rest = b""
            wanted = None
            while True:
                piece = byte()
                if piece is None:
                    return None
                rest += piece
                if wanted is None and len(rest) >= (4 if magic == MAGIC_V2
                                                    else 1):
                    length = rest[0]
                    wanted = head - 1 + length + 2
                if wanted is not None and len(rest) >= wanted:
                    break
            return first + rest

    def wait(self, wanted, timeout=None, predicate=None):
        """The next frame of one message, skipping the others.

        A vehicle streams many messages at once, so "wait for ATTITUDE" has to
        mean "read until one arrives" - and a heartbeat is always welcome,
        because it is how the vehicle's own address is learned.
        """
        deadline = time.time() + (self.timeout if timeout is None
                                  else timeout)
        wanted = {wanted} if isinstance(wanted, str) else set(wanted)
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise Timeout("no %s within %.1f s" % (", ".join(sorted(wanted)),
                                                       timeout or self.timeout))
            frame = self._read_frame(left)
            if frame is None:
                raise Timeout("no %s within %.1f s" % (", ".join(sorted(wanted)),
                                                       timeout or self.timeout))
            try:
                name, fields, header = parse(frame)
            except ValueError:
                self.unknown += 1
                continue
            self.frames += 1
            if name == "HEARTBEAT" and fields["autopilot"] != MAV_AUTOPILOT_INVALID:
                self.heartbeats += 1
                self.vehicle = (header["system"], header["component"])
            if predicate is not None and not predicate(fields):
                continue
            if name in wanted:
                return name, fields, header

    def identify(self, timeout=None):
        """Who is on the other end: the autopilot's heartbeat, then its
        version, if it answers."""
        name, fields, header = self.wait("HEARTBEAT", timeout)
        out = {"autopilot": fields["autopilot"], "type": fields["type"],
               "system": header["system"], "component": header["component"],
               "system_status": fields["system_status"],
               "version": None}
        # AUTOPILOT_VERSION is answered when it is asked for; a vehicle that
        # does not answer still has a heartbeat, which is the identification
        # that matters.
        try:
            self.send("COMMAND_LONG", {
                "command": MAV_CMD_SET_MESSAGE_INTERVAL, "param1": 148.0,
                "param2": 1e6, "target_system": out["system"],
                "target_component": out["component"]})
            _n, version, _h = self.wait("AUTOPILOT_VERSION", 2.0)
            out["version"] = version
        except Timeout:
            pass
        return out

    def request(self, name, hz, system=None, component=None):
        """Ask for a message at a rate - MAV_CMD_SET_MESSAGE_INTERVAL, in
        microseconds, which is what a ground station says instead of the
        rate-table dance older vehicles needed."""
        system = self.vehicle[0] if system is None else system
        component = self.vehicle[1] if component is None else component
        self.send("COMMAND_LONG", {
            "command": MAV_CMD_SET_MESSAGE_INTERVAL,
            "param1": float(MESSAGES[name][0]),
            "param2": 1e6 / hz if hz > 0 else -1.0,
            "target_system": system, "target_component": component})

    def parameters(self, timeout=30.0):
        """Every parameter, by asking the vehicle for the list.

        PARAM_REQUEST_LIST makes a vehicle send PARAM_VALUE for each one,
        indexed, and the last one carries the count - so this reads until it
        has them all or the deadline passes, and returns them sorted by index
        with the count the vehicle gave. A vehicle that answers slowly is why
        the deadline is generous: ArduPilot's parameter stream at 57600 over a
        telemetry radio takes tens of seconds, and this is a read-only view
        rather than a flow-control problem to solve.
        """
        if self.vehicle is None:
            self.identify()
        system, component = self.vehicle
        self.send("PARAM_REQUEST_LIST", {"target_system": system,
                                         "target_component": component})
        found = {}
        count = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            if count is not None and len(found) >= count:
                break
            try:
                _n, fields, _h = self.wait("PARAM_VALUE", 2.0)
            except Timeout:
                # The list went quiet with parameters still missing, which is
                # what a lost frame looks like. The protocol has an answer for
                # that and this is it: ask for the ones that are not here,
                # rather than returning a table with holes in it - which is
                # also how the check under the sanitizers turned a slow machine
                # into a missing parameter once, in the first version of this.
                if count is not None:
                    for index in range(count):
                        if index not in found:
                            self.send("PARAM_REQUEST_READ", {
                                "param_index": index, "param_id": b"",
                                "target_system": system,
                                "target_component": component})
                continue
            found[fields["param_index"]] = (fields["param_id"],
                                            fields["param_value"],
                                            fields["param_type"])
            count = fields["param_count"]
        return {"count": count, "items": [found[i] for i in sorted(found)]}


def describe(identity):
    """One line for a person: which autopilot, which system."""
    names = {3: "ArduPilot", 12: "PX4", 0: "a generic autopilot",
             8: "no autopilot (a ground station)"}
    where = names.get(identity["autopilot"], "autopilot %u"
                      % identity["autopilot"])
    version = ""
    if identity["version"]:
        number = identity["version"]["flight_sw_version"]
        version = ", firmware %u.%u.%u" % ((number >> 24) & 0xFF,
                                           (number >> 16) & 0xFF,
                                           (number >> 8) & 0xFF)
    return ("%s, system %u component %u, state %u%s"
            % (where, identity["system"], identity["component"],
               identity["system_status"], version))


def state(mav):
    """Everything this file reads about a live vehicle, in one dict.

    The keys are the ones the configurator's own status pane uses, so a window
    can show a MAVLink vehicle and one of ours through the same widgets - and
    asks for the messages by name first, because a vehicle streams what its
    ground station asked for and nothing else.
    """
    out = {"flight_state": None, "link_live": 1, "gps_fix_type": None,
           "gps_satellites": None, "lat": None, "lon": None, "alt_m": None,
           "roll_deg": None, "pitch_deg": None, "yaw_deg": None,
           "vbat_v": None, "mah": None, "motors": [], "rc": []}
    try:
        _n, fields, _h = mav.wait("ATTITUDE", 2.0)
        import math
        out["roll_deg"] = math.degrees(fields["roll"])
        out["pitch_deg"] = math.degrees(fields["pitch"])
        out["yaw_deg"] = math.degrees(fields["yaw"]) % 360.0
    except Timeout:
        pass
    try:
        _n, fields, _h = mav.wait("GLOBAL_POSITION_INT", 2.0)
        out["lat"] = fields["lat"] / 1e7
        out["lon"] = fields["lon"] / 1e7
        out["alt_m"] = fields["relative_alt"] / 1000.0
    except Timeout:
        pass
    try:
        _n, fields, _h = mav.wait("GPS_RAW_INT", 2.0)
        out["gps_fix_type"] = fields["fix_type"]
        out["gps_satellites"] = fields["satellites_visible"]
    except Timeout:
        pass
    try:
        _n, fields, _h = mav.wait("SYS_STATUS", 2.0)
        out["vbat_v"] = fields["voltage_battery"] / 1000.0
        out["battery_remaining"] = fields["battery_remaining"]
    except Timeout:
        pass
    try:
        _n, fields, _h = mav.wait("SERVO_OUTPUT_RAW", 2.0)
        out["motors"] = [fields["servo%u_raw" % i] for i in (1, 2, 3, 4)]
    except Timeout:
        pass
    try:
        _n, fields, _h = mav.wait("RC_CHANNELS", 2.0)
        out["rc"] = [fields["chan%u_raw" % i] for i in range(1, 9)]
    except Timeout:
        pass
    return out


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--watch", type=float, default=0.0,
                        help="print the state every N seconds")
    parser.add_argument("--params", action="store_true",
                        help="pull the whole parameter list")
    args = parser.parse_args()

    from msp import Deadline, Port

    port = Port(args.port)
    reader = Deadline(port)
    mav = Mavlink(reader.read_some, port.write)
    try:
        # A ground station has to say something first: a vehicle streams to a
        # system it has heard from.
        mav.send_heartbeat()
        print(describe(mav.identify(5.0)))
        if args.params:
            listing = mav.parameters()
            print("%s parameters" % listing["count"])
            for index, (name, value, kind) in enumerate(listing["items"]):
                print("  %4d  %-16s %12.4f  (type %u)" % (index, name, value,
                                                           kind))
            return 0
        for name in ("ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT",
                     "SYS_STATUS", "VFR_HUD", "RC_CHANNELS",
                     "SERVO_OUTPUT_RAW"):
            mav.request(name, 5.0)
        while True:
            now = state(mav)
            print("state      roll %s pitch %s yaw %s  %s V  %s m up  fix %s "
                  "(%s sats)  rc %s"
                  % (now["roll_deg"], now["pitch_deg"], now["yaw_deg"],
                     now["vbat_v"], now["alt_m"], now["gps_fix_type"],
                     now["gps_satellites"], (now["rc"] or ["?"])[0]))
            if args.watch <= 0.0:
                return 0
            time.sleep(args.watch)
    except Timeout as problem:
        print("no answer: %s" % problem, file=sys.stderr)
        print("a vehicle that never heartbeats may be speaking MSP "
              "(Betaflight, INAV) or AerialKit's own protocol", file=sys.stderr)
        return 1
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())
