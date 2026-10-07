#!/usr/bin/env python3
"""An ArduPilot vehicle that is not there - and the *reference* built it.

    tools/mavlink_fake_vehicle.py            # speaks MAVLink v2 on stdin/stdout
    tools/mavlink_fake_vehicle.py --v1       # the same vehicle, older dialect

This is to an ArduPilot board what `tools/fw_sim.c` is to ours and
`tools/msp_fake_board.py` is to a Betaflight board: the thing the client can be
pointed at on a machine with no vehicle on it. It is different in one way that
matters - **it is not written from a description of MAVLink**:

* every frame it sends is built by **pymavlink itself**
  (`upstream/pymavlink-2.4.49/`, the reference implementation), and
* every frame it receives is parsed by pymavlink.

So `tools/mavlink_check.py` drives AerialKit's client against the reference
implementation's own bytes, which is the strongest evidence available on a
machine with no autopilot: not "my encoder and my decoder agree" (two wrongs
agreeing), but "my client and pymavlink agree, and pymavlink is what
Mission Planner, QGroundControl and MAVProxy are built on".

What it does, which is what a vehicle does when a ground station says hello:

* heartbeats at 1 Hz from boot (a vehicle announces itself; it does not wait
  to be asked), with `MAV_MODE_FLAG_SAFETY_ARMED` set once it is armed;
* answers `COMMAND_LONG`/`MAV_CMD_SET_MESSAGE_INTERVAL` by streaming that
  message at the rate asked for - attitude, position, GPS, battery, servos and
  RC are all available - and acks it, as a vehicle does: `ACCEPTED` for the two
  commands below, `UNSUPPORTED` for anything else it is sent;
* answers `PARAM_REQUEST_LIST` with `PARAM_VALUE` frames, forty of them, the
  way the parameter protocol works (`param_count` on every one, the index
  rising, and the last one closing the list);
* and says nothing at all to anything else, which is what a vehicle does and
  what a client has to survive. `PARAM_SET` is in that "anything else": it is
  deliberately not answered and not acked, so that a client which tried to write
  to this vehicle would get silence rather than a success it never had.

That last line was written while the configurator was read-only, and it is kept
now that it is not. It was a rule about not lying; it turns out to be the more
useful peer for the opposite reason. MAVLink has no reply that means "refused" -
ArduPilot answers a read-only parameter with an ordinary `PARAM_VALUE` carrying
the old value, and answers nothing at all on a link that dropped the frame - so
*"the write stands unresolved"* is a real outcome a client has to be able to
report, and this vehicle is the one that produces it. Making it answer
`PARAM_SET` would delete the only end-to-end test of that path. If a fake
vehicle that *accepts* a write is ever wanted, add it as a flag beside this one
rather than by changing this default.
"""

import argparse
import math
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ORACLE = os.path.join(HERE, "..", "..", "upstream", "pymavlink-2.4.49")
sys.path.insert(0, os.path.normpath(ORACLE))

try:
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError as problem:         # pragma: no cover - the check reports it
    print("mavlink_fake_vehicle: no pymavlink oracle at %s (%s)"
          % (os.path.normpath(ORACLE), problem), file=sys.stderr)
    sys.exit(2)

SYSTEM_ID = 1
COMPONENT_ID = 1

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512

#: Taken from the reference implementation rather than typed, because typing it
#: is exactly how this file came to set the wrong bit for months (see
#: `heartbeat`). A constant read out of pymavlink cannot disagree with pymavlink.
MAV_MODE_FLAG_SAFETY_ARMED = mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
MAV_RESULT_ACCEPTED = mavlink2.MAV_RESULT_ACCEPTED
MAV_RESULT_UNSUPPORTED = mavlink2.MAV_RESULT_UNSUPPORTED

PARAMETER_COUNT = 40
PARAMETER_NAMES = ["RATE_RLL_P", "RATE_PIT_P", "RATE_YAW_P", "ANGLE_MAX",
                   "THR_MID", "ARMING_CHECK", "FS_THR_ENABLE", "WPNAV_SPEED"]


class Vehicle:
    def __init__(self, version=2, drop_param=None, px4=False):
        self.mav = mavlink2.MAVLink(None)
        # pymavlink picks the frame version from this string, which is what
        # makes "the same vehicle speaking v1" one line rather than a second
        # encoder.
        self.mav.WIRE_PROTOCOL_VERSION = "1.0" if version == 1 else "2.0"
        # pymavlink's default source is 0, which no vehicle uses: a real one
        # is system 1 component 1, and the client reads those off the frames.
        self.mav.srcSystem = SYSTEM_ID
        self.mav.srcComponent = COMPONENT_ID
        self.version = version
        self.started = time.time()
        # Which autopilot this stand-in says it is. The two are the same
        # protocol and different words: the window names whichever answered,
        # and refuses to write in that name (docs/27-configurator.md), so the
        # second one is a check rather than a nicety. The identity is the
        # heartbeat's `autopilot` field - MAV_AUTOPILOT_PX4 is 12 in the
        # reference dialect (pymavlink's `common.py`), and PX4 streams the same
        # messages ArduPilot does, which is why nothing else changes.
        self.autopilot = 12 if px4 else 3     # PX4 / ARDUPILOTMEGA
        self.streaming = False         # any message asked for at all
        self.state_period = 0.2        # seconds between state groups
        self.next_state = 0.0
        self.params_pending = 0        # parameters still to send
        self.armed = False
        # One parameter the link "loses": dropped from the list burst and
        # answered the moment it is asked for by index. That is how the
        # recovery path is checked deterministically rather than by hoping a
        # timeout happens.
        self.drop_param = drop_param

    # -- sending ----------------------------------------------------------

    def emit(self, message):
        data = message.pack(self.mav)
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()

    def heartbeat(self):
        # `MAV_MODE_FLAG_SAFETY_ARMED` is 128, not 1 — and this line said 1
        # under a comment naming the right constant, from the day it was
        # written until 2026-09-30. 1 is `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`, so
        # the vehicle armed and the bit saying so was never set: the client's
        # armed-state transition could not be observed against this oracle at
        # all, and the check that read "disarmed" off it was passing because the
        # *bit* was wrong, not because the vehicle was disarmed. Taken from the
        # reference implementation rather than from memory, which is the whole
        # reason this file exists:
        #
        #   python3 -c "import sys; sys.path.insert(0, 'upstream/pymavlink-2.4.49');
        #               from pymavlink.dialects.v20 import common as m;
        #               print(m.MAV_MODE_FLAG_SAFETY_ARMED)"   ->  128
        base_mode = MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0
        self.emit(mavlink2.MAVLink_heartbeat_message(
            type=2,                                # MAV_TYPE_QUADROTOR
            autopilot=self.autopilot,
            base_mode=base_mode,
            custom_mode=4,                         # ArduCopter's GUIDED
            system_status=3 if self.armed else 4,  # ACTIVE / STANDBY
            mavlink_version=3))

    def ack(self, command, result):
        """The answer to a `COMMAND_LONG`, which every autopilot sends."""
        self.emit(mavlink2.MAVLink_command_ack_message(command=command, result=result))

    def state_messages(self, now):
        t = now - self.started
        self.emit(mavlink2.MAVLink_attitude_message(
            time_boot_ms=int(t * 1000), roll=math.radians(12.0 + 2.0 * math.sin(t)),
            pitch=math.radians(-4.0), yaw=math.radians(271.0),
            rollspeed=0.1, pitchspeed=0.0, yawspeed=0.0))
        self.emit(mavlink2.MAVLink_global_position_int_message(
            time_boot_ms=int(t * 1000), lat=521234567, lon=49876543,
            alt=45000, relative_alt=23000, vx=100, vy=-200, vz=30, hdg=27100))
        self.emit(mavlink2.MAVLink_gps_raw_int_message(
            time_usec=int(t * 1e6), lat=521234567, lon=49876543, alt=45000,
            eph=90, epv=140, vel=250, cog=27100, fix_type=3,
            satellites_visible=11))
        self.emit(mavlink2.MAVLink_sys_status_message(
            onboard_control_sensors_present=0x2F1, onboard_control_sensors_enabled=0x2F1,
            onboard_control_sensors_health=0x2F1, load=210,
            voltage_battery=12600, current_battery=1250,
            drop_rate_comm=0, errors_comm=0, errors_count1=0, errors_count2=0,
            errors_count3=0, errors_count4=0, battery_remaining=78))
        self.emit(mavlink2.MAVLink_servo_output_raw_message(
            time_usec=int(t * 1e6), servo1_raw=1500, servo2_raw=1520,
            servo3_raw=1480, servo4_raw=1510, servo5_raw=0, servo6_raw=0,
            servo7_raw=0, servo8_raw=0, port=0))
        self.emit(mavlink2.MAVLink_rc_channels_message(
            time_boot_ms=int(t * 1000), chan1_raw=1500, chan2_raw=1490,
            chan3_raw=1200, chan4_raw=1510, chan5_raw=1000, chan6_raw=1000,
            chan7_raw=1000, chan8_raw=1800, chan9_raw=0, chan10_raw=0,
            chan11_raw=0, chan12_raw=0, chan13_raw=0, chan14_raw=0,
            chan15_raw=0, chan16_raw=0, chan17_raw=0, chan18_raw=0,
            chancount=8, rssi=90))
        self.emit(mavlink2.MAVLink_vfr_hud_message(
            airspeed=14.5, groundspeed=15.2, alt=45.0, climb=1.2, heading=271,
            throttle=52))

    def parameter(self, index):
        name = PARAMETER_NAMES[index % len(PARAMETER_NAMES)]
        if index >= len(PARAMETER_NAMES):
            name = "%s_%u" % (name, index // len(PARAMETER_NAMES))
        self.emit(mavlink2.MAVLink_param_value_message(
            # pymavlink's own encoder takes the id as bytes (the field is a
            # `char[16]` on the wire) - which is one of the things this oracle
            # is kept for.
            param_id=name.encode(), param_value=0.5 + index * 0.25, param_type=9,
            param_count=PARAMETER_COUNT, param_index=index))

    # -- receiving --------------------------------------------------------

    def handle(self, message):
        kind = message.get_type()
        if kind == "HEARTBEAT":
            # A ground station said hello: answer at once rather than waiting
            # for the next second's beat.
            self.heartbeat()
        elif kind == "COMMAND_LONG":
            # Every `COMMAND_LONG` gets a `COMMAND_ACK`, which is what an
            # autopilot does and what a ground station is entitled to wait for.
            # This vehicle used to answer 511 by starting the stream and saying
            # nothing at all, and the cost of that was not a missing frame: the
            # client's whole ack path — correlating an ack to the request it
            # answers, and reporting a refusal as a refusal — was dead code
            # against the only oracle available here, so nothing could tell a
            # working correlation from a broken one.
            result = MAV_RESULT_UNSUPPORTED
            if message.command == MAV_CMD_SET_MESSAGE_INTERVAL:
                period = message.param2 / 1e6 if message.param2 > 0 else 0.0
                if period <= 0.0:
                    self.streaming = False
                else:
                    self.streaming = True
                    self.state_period = max(0.02, min(period, 5.0))
                    self.next_state = 0.0
                result = MAV_RESULT_ACCEPTED
            elif message.command == MAV_CMD_REQUEST_MESSAGE:
                self.streaming = True
                self.next_state = 0.0
                result = MAV_RESULT_ACCEPTED
            self.ack(message.command, result)
        elif kind == "PARAM_REQUEST_LIST":
            self.params_pending = PARAMETER_COUNT
        elif kind == "PARAM_REQUEST_READ":
            # One parameter again, by index: what a vehicle answers when a
            # ground station notices a gap in the list.
            self.parameter(int(message.param_index))
        elif kind == "PARAM_SET":
            # Silent, deliberately, and now for a second reason as well.
            #
            # The first was that a vehicle which answered this would be a
            # vehicle our client could reconfigure by accident. That stopped
            # being the whole story when the configurator was given a parameter
            # write: the client can now send this frame on purpose, and it must
            # report the write as *unresolved* rather than as done. Silence is
            # the only peer that produces that outcome, because MAVLink has no
            # reply meaning "refused" - a real ArduPilot answers a read-only
            # parameter with an ordinary PARAM_VALUE carrying the old value,
            # which is a different case (`acceptsParamSet = false` in
            # apps/configurator/tests/mavlink-link.ts). So this stays silent,
            # and `tools/browser-check.mjs` checks the unresolved sentence it
            # produces end to end.
            pass

    def tick(self, now):
        if self.streaming and now >= self.next_state:
            self.next_state = now + self.state_period
            self.state_messages(now)
        # The parameter list goes out in one burst, which is what every
        # autopilot does with it: it is a stream of small frames and the link
        # is the only thing that slows it down.
        while self.params_pending > 0:
            index = PARAMETER_COUNT - self.params_pending
            if index != self.drop_param:
                self.parameter(index)
            self.params_pending -= 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v1", action="store_true",
                        help="answer with MAVLink v1 frames instead of v2")
    parser.add_argument("--arm-after", type=float, default=0.0,
                        help="seconds until the vehicle arms (0 = never)")
    parser.add_argument("--drop-param", type=int, default=None,
                        help="leave this parameter index out of the list burst, "
                             "and answer it only when it is asked for")
    parser.add_argument("--px4", action="store_true",
                        help="announce PX4 (autopilot 12) instead of ArduPilot, "
                             "so the window's naming and refusals are exercised "
                             "for the other of the two MAVLink firmwares")
    args = parser.parse_args()

    vehicle = Vehicle(version=1 if args.v1 else 2, drop_param=args.drop_param,
                      px4=args.px4)
    parser_v2 = mavlink2.MAVLink(None)
    # A link carries other traffic - our own detection probe is AerialKit's
    # protocol, and a console prints text - and pymavlink *raises* on a byte it
    # does not recognise unless it is told to be robust. Real vehicle firmware
    # ignores noise, so this one does too: the first version of this file died
    # on the very first byte of a probe, which is exactly the behaviour a
    # stand-in must not have.
    parser_v2.robust_parsing = True

    # The file descriptor, not the buffered stream: `select` on a
    # `BufferedReader` is a `select` on the file *descriptor*, and Python may
    # already have the next bytes in its own buffer - so a vehicle that waits
    # for the descriptor to be readable stops with a request sitting in
    # Python's buffer and nothing new arriving. Reading the descriptor
    # directly is the only version of this that cannot stall.
    fd = sys.stdin.fileno()
    next_beat = 0.0
    while True:
        now = time.time()
        if now >= next_beat:
            next_beat = now + 1.0
            vehicle.heartbeat()
        if args.arm_after > 0.0 and not vehicle.armed and \
                now - vehicle.started >= args.arm_after:
            vehicle.armed = True
            vehicle.heartbeat()
        vehicle.tick(now)

        # Reading with a deadline rather than blocking, so a client that is
        # waiting for the stream to arrive is not waiting for a byte to come
        # the other way before it gets one.
        import select
        if not select.select([fd], [], [], 0.02)[0]:
            continue
        byte = os.read(fd, 1)
        if not byte:
            return 0
        # pymavlink's byte-at-a-time entry point returns the message a byte
        # completed, or None - and it keeps its own buffer, so one byte in is
        # the right way to drive it.
        message = parser_v2.parse_char(byte)
        if message is not None and not message.get_type().startswith("BAD_DATA"):
            vehicle.handle(message)


if __name__ == "__main__":
    sys.exit(main())
