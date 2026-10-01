#!/usr/bin/env python3
"""The config protocol against the ESP32 port, under QEMU.

    scripts/esp32-proto.sh          # builds, runs, and drives it

Why this exists: `make proto-test` proves the client and the firmware agree on
a host, and QEMU proves the firmware boots on the ESP32. Neither proves the two
together - a byte lost between the emulated UART and the frame parser, a
parameter table that differs between targets, a console that shares the port
badly on a second chip. This drives the *real client* against the *real
firmware* running on an emulated ESP32, which is as close to a bench session as
this machine gets.
"""

import glob
import os
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from akproto import (Client, HELLO, LOG_SOURCES, PARAM_GET, PARAM_SET,
                     PARAM_SAVE, STATUS, TELEMETRY, c_string, fetch_log,
                     next_telemetry, parse_status, select_log,
                     subscribe)  # noqa: E402

# The port the firmware listens on, forwarded to this machine by QEMU.
NET_PORT = 5555

# The number of parameters the firmware registers. The two targets compile the
# same main.c and the same flight core, so their tables cannot differ by
# accident - which makes this a change detector rather than a comparison: add a
# parameter and this check fails until somebody comes here and confirms the
# ESP32 half still works. It has earned its keep twice, once when the
# accelerometer's six arrived and once with the mission's nine.
EXPECTED_PARAMETER_COUNT = 95  # 91 shared with the F405, plus the ESP32's
                               # wifi_ssid, wifi_pass, wifi_ap_ssid, wifi_mode.
                               # It went up by three when the quadrotor's return
                               # gained its own gains (quad_return_kp,
                               # quad_return_speed, quad_hover_m) - which is
                               # this constant doing its job: a parameter added
                               # on one target and not the other fails here -
                               # and by one more with quad_arrive_m, the radius
                               # the quadrotor starts its descent inside, and
                               # by one more with battery_rth, the pack's own
                               # reason to come home, and by one more with
                               # gps_min_sats, the satellite floor a fix has
                               # to clear before a navigator will use it.
                               # And by three with the rangefinder:
                               # range_land_mm, range_agree_m and
                               # quad_hold_land_s - and by one more with
                               # quad_alt_ki, the vertical loop's integral, and
                               # by one more with rth_alt_ki, the wing's.
                               # And by one more with gyro_lpf_hz, the gyro's
                               # low pass - added on every target at once, which
                               # is what this constant is here to insist on.

# AK_PARAMS_TEXT_MAX: the configuration record is "name=value\n" per
# parameter, and a table that outgrows it is a `save` that writes half of
# itself. That is what it was doing - 64 parameters came to 1135 bytes in a
# 1024-byte record - until the Wi-Fi credentials needed room and somebody
# measured it. The check below measures the same thing on the real firmware.
CONFIG_RECORD_BYTES = 2048

PORT_DIR = "ports/esp32"
CHIP = "esp32"
FLASH_IMAGE = os.path.join(PORT_DIR, "build", "qemu_flash.bin")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail),
          flush=True)
    if not condition:
        failures += 1


def wait_for_banner(stream, needle=b"boot: ok", timeout=30.0):
    """Reads until the firmware says it has finished booting.

    Not politeness: bytes written to the emulated UART before the guest has
    configured it go nowhere, and the first version of this check wrote its
    request into that void and then waited for a reply that could not come. It
    also collects the banner, which is the other half of the evidence.
    """
    seen = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        byte = stream.read(1)
        if not byte:
            break
        seen += byte
        if needle in seen:
            return True, bytes(seen)
    return False, bytes(seen)


def find_qemu():
    roots = [os.environ.get("IDF_TOOLS_PATH", os.path.expanduser("~/.espressif"))]
    for root in roots:
        found = glob.glob(os.path.join(root, "tools", "qemu-xtensa", "*", "qemu",
                                      "bin", "qemu-system-xtensa"))
        if found:
            return sorted(found)[-1]
    return None


def console_command(stream, write, command, needle, timeout=25.0):
    """Type a line at the console and read until `needle` comes back.

    The console and the config protocol share this UART, which is why this runs
    before the client starts: the client parses frames out of the same byte
    stream and skips everything that is not one, so console text left behind is
    harmless to it - while console output written *during* a request is a reply
    arriving out of order.

    The timeout is generous on purpose. It was eight seconds, which is fifteen
    times what the emulated board needs *on an idle machine* - and this check
    ran red once when the rest of the suite was running beside it, because a
    guest under QEMU on a loaded host is a slow guest rather than a broken one.
    The firmware is what is under test here; the host's load is not.
    """
    write(command.encode() + b"\r\n")
    seen = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        byte = stream.read(1)
        if not byte:
            break
        seen += byte
        if needle in seen:
            break
    return bytes(seen)


def build_flash_image():
    """Merges bootloader, partition table and app into the image QEMU boots.

    This is what `idf.py qemu` does before it runs; doing it here is what lets
    the test own the QEMU command line and put the client on its stdio.
    """
    args = [
        sys.executable, "-m", "esptool", "--chip", CHIP, "merge_bin",
        "-o", FLASH_IMAGE, "--fill-flash-size", "2MB",
        "--flash_mode", "dio", "--flash_freq", "40m",
        "0x1000", os.path.join(PORT_DIR, "build", "bootloader", "bootloader.bin"),
        "0x8000", os.path.join(PORT_DIR, "build", "partition_table",
                               "partition-table.bin"),
        "0x10000", os.path.join(PORT_DIR, "build", "aerialkit-esp32.bin"),
    ]
    result = subprocess.run(args, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout + result.stderr, file=sys.stderr)
        raise SystemExit("could not build the flash image")


def connect_to_guest(timeout=20.0):
    """Waits for the firmware's listening socket to answer.

    The port is forwarded by QEMU the moment it starts, but the guest is still
    booting and its own listener is not up until the netif has an address - so
    a refused connection here is a question, not an answer.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            connection = socket.create_connection(("127.0.0.1", NET_PORT), 1.0)
            connection.settimeout(5.0)
            return connection
        except OSError:
            time.sleep(0.5)
    return None


def guest_command(qemu):
    """The QEMU command line, in one place.

    Two checks boot this image now - this one, and the configurator's window
    over the forwarded socket (`tools/akconfig_net_check.py`) - and a guest
    started with a different command line would be a different machine, which
    is the kind of drift a check exists to prevent.

    `-nographic` with `-monitor none` puts the console on stdio and keeps the
    monitor's bytes out of it: the console client needs a clean byte stream in
    both directions. The socket client does not read stdio at all, and gets its
    bytes from the forwarded port.
    """
    return [
        qemu, "-M", "esp32", "-m", "4M",
        "-drive", "file=%s,if=mtd,format=raw" % FLASH_IMAGE,
        "-global", "driver=timer.esp32.timg,property=wdt_disable,value=true",
        # User-mode networking with the firmware's port forwarded to this
        # machine, which is how the network half of these checks reaches the
        # guest at all.
        "-nic", "user,model=open_eth,hostfwd=tcp:127.0.0.1:%d-:%d" %
                (NET_PORT, NET_PORT),
        "-nographic", "-monitor", "none",
    ]


def launch_guest(qemu):
    """The QEMU process, booting the firmware's image. The caller kills it."""
    return subprocess.Popen(guest_command(qemu), stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)


def main():
    # `expect` counts into the module-level total; assigning to it here without
    # this line makes the name local to main() and every later read fails.
    global failures

    qemu = find_qemu()
    if qemu is None:
        raise SystemExit("no qemu-system-xtensa found; see docs/17-esp32-port.md")

    build_flash_image()

    guest = launch_guest(qemu)

    def write(data):
        guest.stdin.write(data)
        guest.stdin.flush()

    # The client skips everything that is not a frame, so the boot banner, the
    # selftest and the preflight report all pass under it harmlessly.
    client = Client(guest.stdout.read, write)

    print("the config protocol, against the firmware on an emulated ESP32",
          flush=True)

    ready, banner = wait_for_banner(guest.stdout)
    for line in banner.decode(errors="replace").splitlines():
        if line.strip():
            print("  board   | %s" % line.strip().replace("\r", ""), flush=True)
    expect("the board booted and said so", ready)
    if not ready:
        guest.kill()
        return 1

    # The receiver and the GPS: the two ports this chip gained, neither of which
    # has ever had a device on it, and the emulator has nothing to attach to
    # either. What it can be asked is that the firmware listens on both, says
    # what it found rather than guessing, and keeps flying while it does it.
    # The needle has to be the *whole* line being waited for, not its first
    # words: this loop stops the moment the needle matches, so "uart:" would
    # stop before "0 bytes dropped" had arrived. That is exactly what the first
    # version of this check did, and it reported a dropped count of nothing.
    rc_text = console_command(guest.stdout, write, "rc", b"bytes dropped")
    for line in rc_text.decode(errors="replace").splitlines():
        if line.strip():
            print("  board   | %s" % line.strip().replace("\r", ""), flush=True)
    expect("the console reports the receiver", b"receiver:" in rc_text)
    expect("with a port that lost nothing", b"0 bytes dropped" in rc_text)

    gps_text = console_command(guest.stdout, write, "gps", b"fix:")
    expect("the console reports the gps",
           b"gps:" in gps_text and b"fix:" in gps_text)

    # SBUS is where this chip differs from the F405, and the difference shows
    # from outside: the F405's `rc` warns that no inverter is fitted in front of
    # the receiver pin, and this port inverts the line in its GPIO matrix, so it
    # must not print that warning. Back to CRSF afterwards, which is what a
    # receiver here would normally be - and what `save` stores below.
    sbus = console_command(guest.stdout, write, "set rc_protocol 1",
                           b"rc_protocol = 1")
    expect("the receiver's protocol is a parameter and takes effect",
           b"rc_protocol = 1" in sbus)
    sbus_rc = console_command(guest.stdout, write, "rc", b"bytes dropped")
    expect("and sbus needs no inverter on this chip",
           b"receiver:  sbus" in sbus_rc and b"no inverter" not in sbus_rc)
    console_command(guest.stdout, write, "set rc_protocol 0", b"rc_protocol =")

    report = console_command(guest.stdout, write, "preflight", b"--    gps:")
    expect("and the preflight counts both ports",
           b"--    receiver:" in report and b"--    gps:" in report)

    # The flight pack. This chip has a converter and the board has a divider
    # drawn for it (GPIO34, ADC1 channel 6), and on a bare devkit nothing is
    # soldered to either - which is a fact the console has to report rather
    # than a voltage: a floating pin reads something entirely plausible, and
    # the core would multiply it by eleven and call it a healthy pack.
    battery = console_command(guest.stdout, write, "battery", b"none fitted")
    expect("and the pack reports no divider rather than inventing volts",
           b"battery:" in battery and b"none fitted" in battery)

    try:
        hello = client.request(HELLO)
        product = c_string(hello, 1)
        count = int.from_bytes(hello[2 + len(product):4 + len(product)], "little")
        expect("hello answers from the ESP32", product == "aerialkit-esp32")
        expect("with the parameter table the F405 build has",
               count == EXPECTED_PARAMETER_COUNT,
               "" if count == EXPECTED_PARAMETER_COUNT else
               " (the board reports %u; update EXPECTED_PARAMETER_COUNT)" % count)

        status = parse_status(client.request(STATUS))
        expect("status answers with a flight state and a link",
               status["flight_state"] == 0 and status["link_live"] == 0)

        first = client.request(PARAM_GET, bytes([0]))
        name = c_string(first, 1)
        value = c_string(first, 2 + len(name))
        expect("a parameter reads back", len(name) > 0 and len(value) > 0)

        # The whole table, measured the way `save` measures it. A secret reads
        # back as "***" and its real value can be up to 63 characters, so the
        # worst case is what counts here rather than what came back.
        serialised = 0
        for index in range(count):
            payload = client.request(PARAM_GET, bytes([index]))
            name = c_string(payload, 1)
            value = c_string(payload, 2 + len(name))
            longest = 63 if value == "***" else len(value)
            serialised += len(name) + 1 + longest + 1
        expect("and the table fits the configuration record `save` writes",
               serialised <= CONFIG_RECORD_BYTES,
               " (%u bytes of %u)" % (serialised, CONFIG_RECORD_BYTES))

        accepted = client.request(PARAM_SET, bytes([0]) + b"0.5")
        expect("and a value can be set", accepted[0] == 0)
        read_back = client.request(PARAM_GET, bytes([0]))
        read_name = c_string(read_back, 1)
        expect("and reads back as what was set",
               c_string(read_back, 2 + len(read_name)) == "0.500")

        saved = client.request(PARAM_SAVE)
        expect("save reaches the ESP32's own storage (NVS)", saved[0] == 0)

        # The log in flash, which this target has because partitions.csv
        # reserves a region for it and src/arch/esp32/flashlog.c hands that
        # region to the same ak_flashlog.c the F405 uses. It is being written
        # while the board sits there - five records a second - so the check
        # waits for the first one rather than assuming the boot was slow enough.
        flash = select_log(client, LOG_SOURCES["flash"])
        expect("the ESP32 has a log in flash, not a refusal", flash is not None)
        if flash is not None:
            deadline = time.time() + 12.0
            while flash == 0 and time.time() < deadline:
                time.sleep(0.5)
                flash = select_log(client, LOG_SOURCES["flash"])
            expect("and it fills while the board runs", flash > 0,
                   " (%s records)" % flash)

            rows = []
            expect("and its records come back over the protocol",
                   fetch_log(client, rows.append, LOG_SOURCES["flash"]) == flash)
            # The first line is the header and the rest are records, in the
            # same columns as the RAM rings - so a log is a log whichever log
            # it came out of.
            expect("in the same shape as the other two",
                   rows and rows[0].startswith("# aerialkit blackbox (flash)")
                   and "time_ms,gyro_x" in rows[0])
    except IOError as error:
        print("  FAILED   the conversation broke: %s" % error)
        failures += 1

    # The network, which is the reason this target exists. Same firmware, same
    # protocol, same parameter table - over a socket instead of a UART, on a
    # stack that is not the one on this machine.
    #
    # Wait for the firmware to *say* it is listening first. QEMU accepts the
    # host's connection the moment it is made, before the guest has a listener,
    # so a connection test passes while the firmware is still booting - and the
    # request then goes into a buffer nobody will ever read. The line on the
    # console is the only honest signal, the same lesson as the boot banner one
    # layer down.
    listening, lines = wait_for_banner(guest.stdout, b"net: ", timeout=20.0)
    for line in lines.decode(errors="replace").splitlines():
        if line.strip():
            print("  board   | %s" % line.strip().replace("\r", ""), flush=True)

    print("and the same protocol over the network", flush=True)
    try:
        connection = connect_to_guest()
        expect("a client can reach the firmware's port", connection is not None)
        if connection is not None:
            net = Client(connection.recv, connection.sendall)

            hello = net.request(HELLO)
            product = c_string(hello, 1)
            count = int.from_bytes(hello[2 + len(product):4 + len(product)],
                                   "little")
            expect("the ESP32 answers over the network", product == "aerialkit-esp32")
            expect("with the same parameter table the console has",
                   count == EXPECTED_PARAMETER_COUNT,
                   "" if count == EXPECTED_PARAMETER_COUNT else
                   " (the board reports %u)" % count)

            # Change a parameter over the network and read it back over the
            # console: the two links are the same firmware, and this is what
            # says so rather than assuming it.
            net.request(PARAM_SET, bytes([0]) + b"0.75")
            read_back = client.request(PARAM_GET, bytes([0]))
            read_name = c_string(read_back, 1)
            expect("a value set over the network is the value the console reads",
                   c_string(read_back, 2 + len(read_name)) == "0.750")

            asked = subscribe(net, 20)
            expect("the firmware agrees to stream telemetry at 20 Hz", asked == 20)

            # The stream is pushed, and it arrives on its own - no request in
            # flight. The bounds are loose on purpose: what is being checked is
            # that frames keep coming at about the rate asked for.
            first = next_telemetry(net)
            expect("telemetry arrives without being asked for",
                   first["flight_state"] == 0 and first["uptime_ms"] > 0)
            print("  frame   | %s" % net.last_frame.hex(), flush=True)

            started = time.time()
            for _ in range(9):
                last = next_telemetry(net)
            elapsed = time.time() - started
            expect("at about the rate that was asked for",
                   0.2 < elapsed < 1.5)
            expect("and the uptime in it is moving",
                   last["uptime_ms"] >= first["uptime_ms"])

            stopped = subscribe(net, 0)
            expect("and it stops when asked to stop", stopped == 0)
            connection.close()

            # And a client that reconnects is not sent the last one's stream.
            # The subscription lives in the link's parser, which lives as long
            # as the firmware does - so without resetting it on connect, a new
            # client gets telemetry before it has asked for anything, and its
            # first request is answered out of order. This check is here
            # because that is exactly what happened.
            #
            # Two attempts, and not because the firmware is at fault: an
            # immediate reconnect through QEMU's user-mode network occasionally
            # lands on a connection the host stack has already torn down, and
            # the request goes nowhere. It is intermittent (a dozen isolated
            # reconnects, one hang) and it is not the listener - the firmware
            # accepts every connection that reaches it, which is what the rest
            # of this file shows. A client that reconnects to a device retries;
            # so does this one, and it says so here rather than being flaky.
            second = None
            for attempt in range(2):
                connection = connect_to_guest(timeout=5.0)
                if connection is None:
                    continue
                try:
                    candidate = Client(connection.recv, connection.sendall)
                    hello = candidate.request(HELLO)
                    second = (connection, candidate, hello)
                    break
                except IOError:
                    connection.close()
                    time.sleep(0.5)

            expect("a second client can connect", second is not None)
            if second is not None:
                connection, net, hello = second
                expect("and its first request is answered first",
                       c_string(hello, 1) == "aerialkit-esp32")
                reply = net.request(TELEMETRY, bytes([5]))
                expect("and it is not already streaming",
                       reply[0] == 5)
                connection.close()
    except (IOError, OSError) as error:
        print("  FAILED   the network conversation broke: %s" % error)
        failures += 1

    # And the configuration this check just saved, asked about the way the boot
    # report asks about it. It has to come back "present and intact" - it did
    # not, before this check existed: the preflight handed the board a 64-byte
    # buffer for a record of about 1200 bytes, and every target answered that
    # with "damaged (checksum mismatch)". The board was right and the question
    # was wrong, which is the kind of thing only a question about real storage
    # can find.
    # The needle is the end of the line, not its first words: this reader stops
    # the moment the needle matches, and "ok    saved configuration" is already
    # in the buffer one byte before the answer is.
    saved = console_command(guest.stdout, write, "preflight",
                            b"present and intact")
    expect("the board calls the configuration it just saved intact",
           b"ok    saved configuration: present and intact" in saved)

    guest.kill()
    guest.wait(timeout=5)

    print("esp32 protocol: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
