#!/usr/bin/env python3
"""The MSP client, against a board that is not there.

    make proto-test        # which runs this with the rest of the clients

`tools/msp_fake_board.py` answers with Betaflight's own payload layouts and
values, so what is checked here is the *client*: the frame it builds, the
checksum, what it makes of each answer, and - the half that matters most - what
it does when the board says nothing, says the wrong thing, or is not an MSP
board at all.

The oracle is upstream/betaflight-2026.6.1 (the framing in `msp_serial.c`, the
commands and layouts in `msp_protocol.h`/`msp.c`, the constants in
`build/version.h`). Where a number here looks arbitrary it is one of those
files' numbers, and the fake board's docstring says which.
"""

import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import akconfig                                                # noqa: E402
import msp                                                     # noqa: E402

SIM = os.environ.get("AK_SIM", "build-host/aerialkit-sim")
FAKE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "msp_fake_board.py")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def missing_simulator():
    """The simulator this check drives our own firmware through, or a reason.

    The MSP half of this file is a stand-in board that is spawned here; the
    other half is *our* firmware, which is a simulator that somebody else
    builds. The default path is the one a person typing `make test` by hand
    gets, and a check in a target build - where the host tools live in
    `$(HOST_OUT)`, not in `build-host` - has to be told which one to use. When
    it is not, the honest answer is a sentence: a traceback from
    `subprocess.Popen` names a path and nothing about the mistake, and the
    mistake is easy to make and hard to see (04-traps.md §27).
    """
    if os.path.exists(SIM):
        return None
    return ("no simulator at %s - this check drives our own firmware too, and "
            "the build it is part of passes AK_SIM=$(HOST_OUT)/aerialkit-sim "
            "(tools/msp_check.py)" % SIM)


class Pipe:
    """A child process on its stdin and stdout - the same three-line transport
    the other checks use, kept here so this file stands alone."""

    def __init__(self, argv):
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE)

    def read(self, count):
        return self.process.stdout.read(count)

    def write(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def close(self):
        self.process.kill()


def board(argv):
    """A pipe and the MSP client on it, ready to ask."""
    transport = Pipe(argv)
    reader = msp.Deadline(transport)
    return transport, msp.Msp(reader.read_some, transport.write, timeout=2.0)


def main():
    problem = missing_simulator()
    if problem is not None:
        # A failure, not a skip: this is not an optional oracle, it is the
        # firmware the check is about.
        print("msp: FAILED - %s" % problem)
        return 1

    print("MSP, against a board that is not there")

    # --- the frame on the wire -------------------------------------------
    #
    # `$M<` + size + command + payload + checksum, with size the payload's
    # length (Betaflight's mspHeaderV1_t is {size, cmd}, and dataSize = size)
    # and the checksum the XOR of size, command and payload. The first check is
    # the exact six bytes a real board receives for "who are you?" - which this
    # said was `$M< 01 01 00` until 2026-10-06, size counting the command: a
    # frame no real board accepts, written by a client and a fake board that
    # agreed with each other.
    expect("a request is the frame Betaflight's parser expects",
           msp.build(msp.MSP_API_VERSION) == b"$M<\x00\x01\x01",
           " (%s)" % msp.build(msp.MSP_API_VERSION).hex())
    # size is the payload's length, so two here - and the checksum is that,
    # the command and every payload byte, XORed.
    expect("and the checksum is the XOR of size, command and payload",
           msp.checksum(0x64, b"\x01\x02") == (2 ^ 0x64 ^ 0x01 ^ 0x02) == 0x65,
           " (0x%02x)" % msp.checksum(0x64, b"\x01\x02"))

    # --- what the board says ---------------------------------------------
    transport, client = board([sys.executable, FAKE])
    identity = msp.identify(client)
    board_id = identity["board"]

    expect("the API version is read as protocol, major and minor",
           identity["api"] == {"protocol": 0, "api_major": 1, "api_minor": 48},
           " (%s)" % identity["api"])
    expect("the variant is four characters, which is what a board sends",
           identity["variant"] == "BTFL", " (%s)" % identity["variant"])
    expect("the version is the calendar one Betaflight writes, string and all",
           identity["version"]["text"] == "2026.6.1" and
           identity["version"]["major"] == 2026,
           " (%s)" % identity["version"])
    expect("the board identifier and the target name come out of the "
           "length-prefixed strings rather than out of a split on zero",
           board_id["board_id"] == "S405" and board_id["name"] == "STM32F405" and
           board_id["board_name"] == "AERIALKIT-SIM",
           " (%s / %s / %s)" % (board_id["board_id"], board_id["name"],
                                board_id["board_name"]))
    expect("and one line names the firmware, its version and the board",
           msp.describe(identity) ==
           "Betaflight 2026.6.1 (BTFL), API 1.48, MSP protocol 0, board S405",
           " (%s)" % msp.describe(identity))

    now = msp.state(client)
    expect("the attitude comes back in degrees, not in tenths of one",
           abs(now["roll_deg"] - 15.3) < 0.001 and now["yaw_deg"] > 270.0,
           " (roll %.1f, yaw %.1f)" % (now["roll_deg"], now["yaw_deg"]))
    expect("the pack is the grown analog frame: hundredths of a volt",
           abs(now["vbat_v"] - 12.6) < 0.05 and now["mah"] > 0,
           " (%.2f V, %u mAh)" % (now["vbat_v"], now["mah"]))
    expect("the fix, the satellites and the position are the module's",
           now["gps_fix_type"] == 3 and now["gps_satellites"] == 11 and
           abs(now["lat"] - 52.1234567) < 1e-6,
           " (fix %s, %s sats, %.7f)" % (now["gps_fix_type"],
                                         now["gps_satellites"], now["lat"]))
    expect("the motor outputs are the eight the protocol carries",
           now["motors"][:4] == [1001, 1100, 1200, 1300] and
           len(now["motors"]) == 8, " (%s)" % now["motors"][:4])
    expect("and the state is the one the status frame describes: "
           "cycle time, sensors, and the arm bit",
           now["cycletime_us"] == 1000 and now["sensors"] == 43 and
           now["modes"] == 0, " (%s)" % {k: now[k] for k in
                                         ("cycletime_us", "sensors", "modes")})
    expect("the second poll is not the same as the first - a board that did "
           "not move would pass a check like this by being frozen",
           msp.state(client)["roll_deg"] != now["roll_deg"])
    expect("and nothing of the stream was noise or an unasked-for frame",
           client.noise == 0 and client.unsolicited == 0,
           " (%u noise bytes, %u unsolicited)" % (client.noise,
                                                  client.unsolicited))
    transport.close()

    # --- the arming bit, which is a *flag* and not a state name ----------
    transport, client = board([sys.executable, FAKE, "--arm-after", "0.05"])
    # Identified first, so the 0.2 s is measured from a board that has
    # certainly started: a Python interpreter takes longer to come up on a busy
    # machine than the arming delay itself, and the first version of this check
    # read the arm bit before the board had begun counting.
    msp.identify(client)
    time.sleep(0.2)
    armed = msp.state(client)
    expect("a board whose ARM box is high reads as armed, from the mode "
           "flags", (armed["modes"] & 1) == 1 and armed["flight_state"] == 1,
           " (modes 0x%x)" % armed["modes"])
    transport.close()

    # --- the settings, which a Betaflight board answers by name ----------
    #
    # MSP has no "list the settings" command: `MSP2_CLI_SETTING` takes a name
    # and answers "name = value", and `MSP2_CLI_SETTING_INFO` takes a name and
    # answers that setting's description (which is why every configurator ships
    # its own list of names - see docs/27-configurator.md).
    def crc8_bitwise(data):
        """CRC-8/DVB-S2 written the long way, so the check does not use the
        same code it is checking: poly 0xD5, init 0, MSB first."""
        crc = 0
        for byte in data:
            crc = crc ^ byte
            for bit in range(8):
                top = crc & 0x80
                crc = (crc << 1) & 0xFF
                if top:
                    crc ^= 0xD5
        return crc

    request = msp.build_v2(msp.MSP2_CLI_SETTING, b"failsafe_throttle")
    expect("a v2 request is the frame Betaflight's v2 parser expects",
           request[:3] == b"$X<" and request[3] == 0 and
           request[4] | (request[5] << 8) == msp.MSP2_CLI_SETTING and
           request[6] | (request[7] << 8) == len(b"failsafe_throttle") and
           request[8:-1] == b"failsafe_throttle",
           " (%s)" % request.hex())
    expect("and its checksum is CRC-8/DVB-S2 over the v2 header and payload",
           request[-1] == crc8_bitwise(request[3:-1]) and
           request[-1] == msp.crc8_dvb_s2(request[3:-1]),
           " (0x%02x)" % request[-1])

    transport, client = board([sys.executable, FAKE])
    msp.identify(client)
    value = client.setting("failsafe_throttle")
    expect("a setting is read by name, and the board answers with its own "
           "sentence",
           value == "failsafe_throttle = 1050", " (%s)" % value)

    info = client.setting_info("failsafe_throttle")
    expect("and its description carries the keys the reference writes",
           "pgn=8193\n" in info and "type=uint16\n" in info and
           "min=1000\n" in info and "max=2000\n" in info and
           "default=1000" in info,
           " (%r)" % info)
    expect("including when it is longer than one reply: the window is asked "
           "for again rather than half a description being shown",
           len(info) > 40 and info.count("pgn=") == 1,
           " (%u bytes, the board answers 40 at a time)" % len(info))

    try:
        client.setting("not_a_setting")
        expect("a name the board does not have is an error, not an empty "
               "value", False)
    except msp.Error:
        expect("a name the board does not have is an error, not an empty "
               "value", True)

    # --- writing one, which is the same command as reading it -----------
    #
    # `MSP2_CLI_SETTING` is a read *and* a write, and the reply to a write is
    # the read-back: `msp.c` calls `cliSetSettingByName` and then
    # `cliGetSettingByName` on the same name. So these three checks are about
    # the difference between "the board said yes" and "the board now holds
    # this", which is the whole of the four-fact discipline on this wire.
    echo = client.set_setting("failsafe_throttle", 1200)
    expect("a write is answered with what the board now holds, not with an "
           "acknowledgement",
           echo == "failsafe_throttle = 1200", " (%s)" % echo)
    expect("and the next read agrees with that reply, because it is the same "
           "answer", client.setting("failsafe_throttle") == echo)

    try:
        client.set_setting("failsafe_throttle", 9999)
        expect("a value the board will not take is refused rather than "
               "clamped", False)
    except msp.Error:
        expect("a value the board will not take is refused rather than "
               "clamped", True, " (9999 is outside 1000..2000)")
    expect("and the refusal did not change what the board holds",
           client.setting("failsafe_throttle") == "failsafe_throttle = 1200")

    # The save, which is a separate act and a separate command.
    client.save()
    expect("the running configuration can be put into flash, as its own "
           "command rather than as part of a write", True)
    transport.close()

    # --- and a board that is armed refuses to save ----------------------
    #
    # Both firmwares check the arming flag before writing their configuration
    # to flash, so the refusal is the board's own. `--arm-after 0.0` would
    # mean "never", so this asks for the bit immediately.
    transport, client = board([sys.executable, FAKE, "--arm-after", "0.01"])
    msp.identify(client)
    time.sleep(0.2)
    state = msp.state(client)
    expect("the stand-in can be made to report itself armed",
           state["flight_state"] == 1, " (modes 0x%x)" % state["modes"])
    try:
        client.save()
        expect("and an armed board refuses to save its configuration, in its "
               "own voice", False)
    except msp.Error:
        expect("and an armed board refuses to save its configuration, in its "
               "own voice", True, " (MSP_EEPROM_WRITE = 250)")
    expect("while still taking a set, which is a different act",
           client.set_setting("failsafe_throttle", 1300) ==
           "failsafe_throttle = 1300")
    transport.close()

    # --- a board that says nothing ---------------------------------------
    #
    # Betaflight answers the commands it knows and is silent for the ones it
    # does not - and a client that waits for ever is a window that hangs.
    transport, client = board([sys.executable, FAKE])
    client.timeout = 0.3
    try:
        client.request(0x7E)          # not a command this board knows
        expect("a command the board does not answer times out", False)
    except msp.Timeout:
        expect("a command the board does not answer times out rather than "
               "hanging", True, " (0.3 s)")
    transport.close()

    # --- and a board that answers wrongly --------------------------------
    transport, client = board([sys.executable, FAKE])
    try:
        msp.parse(b"$M>\x04\x01\x00\x01\x30\xFF")     # one byte of checksum off
        expect("a reply whose checksum is wrong is refused", False)
    except msp.ProtocolError:
        expect("a reply whose checksum is wrong is refused rather than "
               "decoded", True)
    try:
        msp.parse(b"$M>\x04\x01\x00\x01")
        expect("a frame that stops early is refused", False)
    except msp.ProtocolError:
        expect("a frame that stops early is refused", True)
    transport.close()

    # --- which firmware is on the other end ------------------------------
    #
    # The window's first question, and the answer decides whether it shows a
    # parameter table or a read-only view of somebody else's board.
    transport = Pipe([sys.executable, FAKE])
    reader = msp.Deadline(transport)
    kind, configurator = akconfig.detect(reader.read_some, transport.write)
    expect("detection asks our protocol first and finds MSP when it is not "
           "answered", kind == "msp", " (%s)" % kind)
    expect("and what it builds describes the board it found",
           configurator is not None and
           "Betaflight" in configurator.describe(),
           " (%s)" % (configurator.describe() if configurator else None))
    expect("the window's own questions are answered by it: status, no "
           "parameters, no logs",
           configurator.status()["vbat_v"] > 0 and
           configurator.parameters() == [] and
           configurator.log_counts() == {},
           " (%u parameters, %u logs)" % (len(configurator.items),
                                          len(configurator.log_counts())))
    expect("and the panes that would write say what they cannot do, in the "
           "board's name rather than in a number",
           configurator.set(0, "1")[1].startswith("this is a Betaflight") and
           configurator.save().startswith("not saved"),
           " (%s)" % configurator.save())
    transport.close()

    # --- and the same question of our own firmware ------------------------
    transport = Pipe([SIM])
    reader = msp.Deadline(transport)
    kind, configurator = akconfig.detect(reader.read_some, transport.write)
    expect("our own board is recognised before MSP is ever asked",
           kind == "aerialkit" and configurator.product.startswith("aerialkit"),
           " (%s, %s)" % (kind, configurator.product))
    expect("and it is the same window that would have shown it: the "
           "parameter table is there",
           configurator.parameters() and configurator.count > 20,
           " (%u parameters)" % configurator.count)
    transport.close()

    # --- and the names, which the protocol has no way to ask for -----------
    #
    # MSP answers a *name* and has no request that lists them, so every MSP
    # configurator ships the list itself. Ours reads the pinned reference
    # checkout of the release the board says it is (`tools/msp_settings.py`),
    # and what is checked here is that the reading works for both firmwares and
    # both shapes of entry in Betaflight's table - a literal name and one that
    # is a macro (`gyro_hardware_lpf` appears in no literal in settings.c,
    # which is the entry a one-line grep loses).
    import msp_settings

    print("the names a release has, read from its own table")
    expect("the variant strings a board reports map to a reader",
           msp_settings.firmware_for("BTFL") == "betaflight" and
           msp_settings.firmware_for("INAV") == "inav" and
           msp_settings.firmware_for("EMUF") is None,
           " (BTFL -> betaflight, INAV -> inav, EMUF -> nothing)")

    for firmware, version, macro_named in (("betaflight", "2026.6.1",
                                            "gyro_hardware_lpf"),
                                           ("inav", "9.1.0", None)):
        found, source = msp_settings.names(firmware, version)
        if not found:
            # No checkout of this release on this machine: the same honest
            # skip the MAVLink checks make for their oracle, and for the same
            # reason - `upstream/` is recreated by scripts/fetch-upstreams.sh
            # rather than tracked.
            print("  ----     %s %s: not checked here - %s"
                  % (firmware, version, source))
            continue
        expect("every name %s %s has is read from its own table"
               % (firmware, version),
               len(found) > 500 and "failsafe_throttle" in found and
               (macro_named is None or macro_named in found) and
               found == sorted(found) and len(set(found)) == len(found),
               " (%u names, %s%s)"
               % (len(found), source,
                  "" if macro_named is None
                  else ", including the macro-named %s" % macro_named))

    # And a release this machine has no checkout of gets a reason rather than
    # an empty list: "this firmware has no settings" and "this machine cannot
    # read it" are different sentences, and only one of them is true.
    found, source = msp_settings.names("betaflight", "1.0.0")
    expect("a release with no checkout here is a reason, not an empty board",
           found == [] and "1.0.0" in source,
           " (%s)" % source)

    print("msp: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
