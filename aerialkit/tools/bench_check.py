#!/usr/bin/env python3
"""The bring-up checklist, typed at the console.

    python3 tools/bench_check.py --sim
    python3 tools/bench_check.py --port /dev/ttyACM0 \\
        --out docs/evidence/bench-f405-2026-09-16.txt

Why this exists: docs/05-bringup.md is a list of commands to type at a new
board, and doing it by hand has three problems. It is slow. It is easy to skip
the step that is awkward. And what it produces is a terminal scrollback rather
than an artifact - the goal's rule is that bench evidence goes next to the
claim, and "I remember it printed the banner" is not evidence.

So this types the same list, reads the firmware's own answers back, and writes
down what it saw. The verdict is three lists rather than one pass mark, because
those are three different things:

  verified    the board answered, and the answer was checked
  not fitted  the firmware says the hardware is not on this board
  unverified  needs something this script cannot do (a jumper, a receiver, a
              steady hand for the calibration)

An "unverified" line is not a failure and does not fail the run: a bare bench
board has no receiver and no barometer, and pretending otherwise would make the
one command that matters untrustworthy. What *does* fail the run is a board
that does not answer, or a preflight that reports a problem about the firmware
itself.

`--sim` drives the software-in-the-loop board over a pipe instead of a serial
port, which is how this script is tested without the hardware it is for.
"""

import argparse
import datetime
import os
import select
import subprocess
import sys
import time

# The prompt, which is the only thing the firmware prints that means "the last
# command has finished". Read as bytes off the wire, matched as text once the
# answer has been decoded.
PROMPT = b"ak> "
PROMPT_TEXT = "ak> "


class PipeConsole:
    """The simulator: a process whose stdin is the keyboard."""

    def __init__(self, argv):
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT)

    def read_until(self, needle, timeout):
        seen = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            ready, _, _ = select.select([self.process.stdout], [], [], 0.05)
            if not ready:
                if self.process.poll() is not None:
                    break
                continue
            chunk = os.read(self.process.stdout.fileno(), 4096)
            if not chunk:
                break
            seen += chunk
            # The prompt ends the *read*, not the *answer*: a board with
            # something of its own to say carries on after it. Reading on until
            # the prompt is the last thing seen was tried on 2026-09-28 and
            # reverted the same day - it costs a full timeout on every command
            # against a board that talks (175 s where this takes 0.27 s, traps
            # 201) and does not recover the answer it was meant to, because a
            # stray prompt ends a read with nothing behind it either way. What
            # recovers an answer is the caller saying what a whole one looks
            # like; see `Runner.command`'s `until`.
            if needle in seen:
                break
        return bytes(seen)

    def drain(self, quiet=0.2, cap=0.5):
        """Throw away what the wire already holds, so that a prompt nobody
        asked for cannot end the read that follows.

        Two bounds, and the second is the one that bit: it stops after `quiet`
        seconds of silence, and never later than `cap` in total. Waiting for
        silence *alone* is waiting forever on a far end that has something of
        its own to say. The software-in-the-loop board does: its 10 s heartbeat
        line (AK_HEARTBEAT_MS, main.c) arrives every ~17 ms of wall time,
        because its clock runs about 600x real time - 181 heartbeat lines in
        the 3 s a `timeout 3` allows, the last reading `alive: 1810811 ms`.
        Each one pushed the deadline back before it could expire, so the
        checklist stalled at the first command after the boot report and never
        reached the second."""
        start = time.time()
        deadline = start + quiet
        while time.time() < deadline and time.time() - start < cap:
            ready, _, _ = select.select([self.process.stdout], [], [], quiet)
            if not ready:
                continue
            chunk = os.read(self.process.stdout.fileno(), 4096)
            if not chunk:
                break
            deadline = time.time() + quiet

    def write(self, text):
        self.process.stdin.write(text)
        self.process.stdin.flush()

    def close(self):
        try:
            self.process.stdin.close()
        except OSError:
            pass
        self.process.wait(timeout=10)


def leave_port_as_found(port):
    """Put the tty back to VMIN=1, VTIME=0 before the port is let go.

    pyserial sets VMIN=0 for its timed reads and does not undo it, and a tty
    keeps its termios after close. Chrome's Web Serial does not set VMIN when it
    opens a port, so it inherits the 0: its second read returns zero bytes, which
    it reports as "the device has been lost", and the configurator then sees a
    silent board. Measured on 2026-10-01 (strace of Chrome on the Y520). A no-op
    where there is no termios.
    """
    try:
        import termios
        attrs = termios.tcgetattr(port.fd)
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(port.fd, termios.TCSANOW, attrs)
    except Exception:
        pass


class SerialConsole:
    """A board: its console is a serial port, usually the USB one."""

    def __init__(self, port, baud, timeout):
        try:
            import serial  # optional dependency, imported only if used
        except ImportError:
            raise SystemExit("--port needs pyserial: pip install pyserial")
        self.serial = serial.Serial(port, baud, timeout=timeout)

    def read_until(self, needle, timeout):
        seen = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.serial.read(1)
            if not chunk:
                continue
            seen += chunk
            if needle in seen:      # as `PipeConsole.read_until`, and for the
                break               # same reason
        return bytes(seen)

    def drain(self, quiet=0.2, cap=0.5):
        """As `PipeConsole.drain`, and bounded the same way - a board that
        streams telemetry on its own account is the same hazard as a
        simulator that runs fast, and a bench is where that would show up."""
        start = time.time()
        deadline = start + quiet
        while time.time() < deadline and time.time() - start < cap:
            if self.serial.read(4096):
                deadline = time.time() + quiet

    def write(self, text):
        self.serial.write(text)

    def close(self):
        leave_port_as_found(self.serial)
        self.serial.close()


def text_of(raw):
    return raw.decode("utf-8", errors="replace").replace("\r\n", "\n")


def one_line(answer, *names):
    """The first line of the answer that starts with one of `names`."""
    for line in answer.splitlines():
        for name in names:
            if line.startswith(name):
                return line
    return ""


class Runner:
    def __init__(self, console, transcript):
        self.console = console
        self.transcript = transcript
        self.verified = []
        self.absent = []
        self.unverified = []
        self.failed = []

    def record(self, raw):
        self.transcript.append(text_of(raw))
        return text_of(raw)

    def command(self, line, timeout=8.0, until=None):
        # One terminator, not two: this console treats CR and LF as separate end
        # of line, so "command\r\n" runs the command and then an empty line -
        # which prints a second prompt, and two prompts per command is exactly
        # the kind of thing that makes a script's framing wrong in a way that
        # only shows up as answers that look truncated.
        #
        # Throw away whatever is already on the wire first. The prompt is what
        # this file reads to mean "the last command has finished", and it is the
        # only thing the firmware prints that says so - but it is not the only
        # thing that can put those four bytes on the wire. The console shares
        # its port with the binary config protocol (docs/16-protocol.md), and
        # until 2026-09-28 a frame the firmware's parser abandoned over a bad
        # length left its tail arriving at the console, CR and LF and all. A
        # stray prompt already in the buffer ended the *next* read on its first
        # byte: the checklist asked for the parameter table and got the leftover
        # prompt from the command before, found no table in it, and said so -
        # while the table itself arrived a moment later and was discarded unread.
        # That is the "no table came back" flake, and it was never the board.
        self.console.drain()
        self.console.write((line + "\r").encode())
        deadline = time.time() + timeout
        text = self.record(self.console.read_until(PROMPT, timeout))
        # `until` is for the answers the prompt does not frame on its own. A
        # prompt that arrives *while* an answer is still coming looks exactly
        # like the end of it, so the caller that knows what a whole answer looks
        # like says so and the read carries on until it sees it.
        while until is not None and not until(text) and time.time() < deadline:
            text += self.record(self.console.read_until(
                PROMPT, max(0.1, deadline - time.time())))
        return text

    def ok(self, what, note=""):
        self.verified.append((what, note))
        print("  ok        %s%s" % (what, (" - " + note) if note else ""),
              flush=True)

    def not_fitted(self, what, note=""):
        self.absent.append((what, note))
        print("  --        %s%s" % (what, (" - " + note) if note else ""),
              flush=True)

    def cannot(self, what, note=""):
        self.unverified.append((what, note))
        print("  ..        %s%s" % (what, (" - " + note) if note else ""),
              flush=True)

    def failed_check(self, what, note=""):
        self.failed.append((what, note))
        print("  FAILED    %s%s" % (what, (" - " + note) if note else ""),
              flush=True)


def run_checks(runner, boot):
    """The checklist. Every check is the firmware's own answer, read back."""

    if PROMPT_TEXT not in boot:
        # No prompt in what was read at open. That is *not* the same thing as a
        # dead board, and it took a port test to notice: this is what a board
        # that had already booted looks like, which is the usual case at a bench
        # (plug the cable in, then type the command). The boot report went out
        # before this process had the port open, and neither a USB CDC device
        # nor a pty replays it. So the board is asked the smallest question a
        # console answers - a bare newline - and the prompt that comes back is
        # the same evidence the boot report would have given. What is *not* in
        # the transcript then is the boot report itself, and the note says so
        # rather than leaving somebody to wonder why the first line is missing.
        answer = runner.command("", timeout=3.0)
        if PROMPT_TEXT in answer:
            runner.ok("the console is listening",
                      "the port was opened after the board had booted; reset it "
                      "while this command waits if you want the boot report too")
        else:
            runner.failed_check("the console is listening",
                                "no prompt, before or after a bare newline")
            return
    else:
        runner.ok("the console is listening", "the boot report ended at a prompt")

    version = runner.command("version")
    product = one_line(version, "product:")
    revision = one_line(version, "rev:")
    built = one_line(version, "built:")
    if product and revision and built:
        runner.ok("it says what it is",
                  "%s %s, %s" % (product.split(":", 1)[1].strip(),
                                 revision.split(":", 1)[1].strip(),
                                 built.split(":", 1)[1].strip()))
    else:
        runner.failed_check("it says what it is", "no version block came back")

    # The preflight is the one check whose failure is about the firmware rather
    # than about the bench: it means the board and the firmware disagree.
    preflight = runner.command("preflight", timeout=12.0)
    if "preflight: the machine is what the firmware thinks it is" in preflight:
        runner.ok("the preflight passes", "the board is what the firmware believes")
    else:
        problems = [l for l in preflight.splitlines() if l.startswith("FAIL")]
        runner.failed_check("the preflight passes",
                            "; ".join(problems) if problems else "no verdict line")

    # Two facts a pilot reads this list for and could not find before: what the
    # fix is worth to the navigator, and whether the return they may be
    # trusting would actually happen. Both are facts rather than faults - an
    # aircraft with RTH off is not broken - so this checks that the lines are
    # there and says which answer they gave.
    # The fact lines carry the console's "--    " marker, and one_line() reads
    # from the start of a line, so the marker is part of the name.
    fix_line = one_line(preflight, "--    fix:")
    return_line = one_line(preflight, "--    return:")
    if fix_line and return_line:
        runner.ok("and says what the fix is worth and whether a return would work",
                  "%s / %s" % (fix_line.split(":", 1)[1].strip(),
                               return_line.split(":", 1)[1].strip()))
    else:
        runner.failed_check("and says what the fix is worth and whether a return would work",
                            "no fix or return line in the preflight")

    # And two more facts that decide whether the flight is the one the pilot
    # thinks: which airframe is loaded (the mix and the return profile have to
    # agree, or a wing flies a quadrotor's mix), and whether the gyro's offset
    # was measured by the aircraft itself or is the number left over from a
    # bench. Neither is a fault; both are things a pilot should not have to
    # guess.
    airframe_line = one_line(preflight, "ok    airframe")
    bias_line = one_line(preflight, "--    gyro bias:")
    if airframe_line and bias_line:
        runner.ok("says which airframe is loaded and which gyro bias it flies on",
                  "%s / %s" % (airframe_line.split(":", 1)[1].strip(),
                               bias_line.split(":", 1)[1].strip()))
    else:
        runner.failed_check("says which airframe is loaded and which gyro bias it flies on",
                            "no airframe or gyro bias line in the preflight")

    # And the return's own answer with the switch on: on the bench the aircraft
    # has a fix and (from the console's `home`) somewhere to come back to, so
    # this asks for the ready case rather than for a particular board's state.
    runner.command("set rth_enable 1")
    preflight_on = runner.command("preflight", timeout=12.0)
    said = one_line(preflight_on, "--    return:")
    if said and "ready" in said:
        runner.ok("with the return switched on it says it is ready",
                  said.split(":", 1)[1].strip())
    elif said:
        runner.cannot("with the return switched on it says it is ready",
                          said.split(":", 1)[1].strip())
    else:
        runner.failed_check("with the return switched on it says it is ready",
                            "no return line came back")
    runner.command("set rth_enable 0")

    status = runner.command("status")
    state = one_line(status, "state:")
    loops = one_line(status, "loops:")
    if state and loops:
        runner.ok("the flight loop is running",
                  "%s, %s" % (state.split(":", 1)[1].strip(),
                              loops.strip()))
    else:
        runner.failed_check("the flight loop is running", "no status came back")

    # Sensors: the driver naming a part is the strongest thing this script can
    # hear, and "none fitted" is an answer rather than a fault.
    imu = runner.command("imu")
    if "none fitted" in imu or "no inertial sensor" in imu:
        runner.not_fitted("the inertial sensor", "the board reports none")
    elif "who am i" in imu.lower() or "no answer" in imu.lower():
        runner.failed_check("the inertial sensor", "the bus answered nothing")
    else:
        runner.ok("the inertial sensor", one_line(imu, "imu:") or "reported")

    baro = runner.command("baro")
    if "none fitted" in baro:
        runner.not_fitted("the barometer", "the board reports none")
    else:
        runner.ok("the barometer", one_line(baro, "baro:", "pressure:") or
                  "reported")

    # The rangefinder, which is the part a *landing* leans on. "None fitted" is
    # an answer here too - the aircraft lands on the barometer, which is what
    # every flight before this one did - and the check that matters with one
    # fitted is that it has an answer rather than "nothing in range" while it
    # is standing on the bench.
    rangefinder = runner.command("range")
    if "none fitted" in rangefinder or "no rangefinder" in rangefinder:
        runner.not_fitted("the rangefinder", "the board reports none")
    elif "nothing in range" in rangefinder:
        runner.failed_check("the rangefinder",
                            "a part is fitted but sees no ground below it")
    else:
        runner.ok("the rangefinder", one_line(rangefinder, "range:", "ground:")
                  or "reported")

    battery = runner.command("battery")
    if "none fitted" in battery:
        runner.not_fitted("the flight pack", "no divider on this board")
    else:
        runner.ok("the flight pack", one_line(battery, "battery:") or "reported")

    rc = runner.command("rc")
    frames = one_line(rc, "receiver:")
    telemetry = one_line(rc, "telemetry:")
    if frames:
        got = int(frames.split(",")[1].split()[0]) if "," in frames else 0
        if got > 0:
            runner.ok("the receiver", frames.strip())
        else:
            runner.not_fitted("the receiver", "the port is open, no frames yet")
    else:
        runner.failed_check("the receiver", "no receiver report came back")

    # The other direction of the same wire: what the handset would have been
    # sent. It goes out whether or not a receiver is there to hear it, so a
    # bare board is expected to have frames - what this catches is a telemetry
    # path that never runs at all.
    if "none - sbus" in telemetry:
        runner.cannot("the telemetry out", "sbus has no return path")
    elif telemetry and " 0 frames out" not in telemetry:
        runner.ok("the telemetry out", telemetry.strip())
    else:
        runner.failed_check("the telemetry out",
                            telemetry.strip() or "no telemetry line came back")

    gps = runner.command("gps")
    fix = one_line(gps, "fix:")
    if fix and "none yet" in fix:
        runner.not_fitted("the gps", "no fix (no module, or no sky)")
    elif fix:
        runner.ok("the gps", fix.strip())
    else:
        runner.failed_check("the gps", "no gps report came back")

    # The bus loopback is a jumper away from being a check: without one it says
    # what a floating MISO looks like, which is not a failure of the firmware.
    spi = runner.command("spi")
    if "loopback matches" in spi:
        runner.ok("the sensor bus", "MOSI reaches MISO")
    elif "transfer timed out" in spi:
        runner.failed_check("the sensor bus", "the transfer timed out")
    else:
        runner.cannot("the sensor bus", "jumper MOSI to MISO and run `spi`")

    output = runner.command("output")
    # A board with DShot says so with its timing; the simulator says what it is
    # simulating. Either is an answer from the output layer, and the F405's
    # detail (rate, ARR, the two compare values) is checked in the preflight.
    rate = one_line(output, "dshot:", "outputs:")
    if rate:
        runner.ok("the outputs answer", rate.strip())
    else:
        runner.not_fitted("the outputs", "this board has none")

    # And the servos' own plumbing, which is the half of the output layer a
    # person sets rather than the firmware: `output` prints each servo's
    # reversal, centre trim and travel, and a board that drives servos has to
    # say what they are set to - a reversed elevon that nobody noticed is a wing
    # that does not turn.
    servo = one_line(output, "servo 1:", "servos:")
    if servo:
        runner.ok("the servos' plumbing is reported", servo.strip())
    elif rate:
        runner.failed_check("the servos' plumbing is reported",
                            "output named no servo settings")
    else:
        runner.not_fitted("the servos' plumbing", "this board drives no servos")

    log = runner.command("log flash", timeout=15.0)
    if "no log in flash" in log or "not here" in log:
        runner.not_fitted("the blackbox in flash", "no log region on this board")
    elif "time_ms" in log:
        records = max(0, len(log.strip().splitlines()) - 1)
        runner.ok("the blackbox in flash", "%u records" % records)
    else:
        runner.failed_check("the blackbox in flash", "no log came back")

    check_parameter_table(runner)

    runner.cannot("the calibration", "needs the aircraft still and the props off")


def table_counts(text):
    """What the table's header declared, and how many rows actually arrived."""
    header = one_line(text, "params (")
    try:
        declared = int(header.split("(")[1].split(")")[0])
    except (IndexError, ValueError):
        declared = 0
    listed = [l for l in text.splitlines()
              if l.startswith("  ") and len(l.split()) >= 2]
    return declared, len(listed)


def check_parameter_table(runner):
    """The longest answer the console gives, and the check that found this
    file's read bug.

    It is a function of its own so that `tools/bench_read_check.py` can drive
    *this* check against a console that misbehaves on purpose, rather than
    against a copy of it that would agree with whatever it was written to
    expect. See `Runner.command` for what went wrong.

    And it is the one check that says what a whole answer looks like, because
    it is the one answer long enough for a stray prompt to land inside: the
    header counts the rows, so the read stops when the rows are there rather
    than at the first thing on the wire shaped like a prompt."""
    params = runner.command(
        "params", timeout=12.0,
        until=lambda text: table_counts(text)[0] > 20 and
        table_counts(text)[1] >= table_counts(text)[0])
    declared, listed = table_counts(params)
    if declared > 20 and listed >= declared:
        runner.ok("the parameter table", "%u parameters" % declared)
    elif listed:
        runner.failed_check("the parameter table",
                            "%u lines for %u declared parameters"
                            % (listed, declared))
    else:
        runner.failed_check("the parameter table", "no table came back")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device (the F405's USB console)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sim", action="store_true",
                        help="drive the software-in-the-loop board instead")
    parser.add_argument("--sim-binary", default="build-host/aerialkit-fw-sim")
    parser.add_argument("--out", help="write the transcript here")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="seconds to wait for the boot report")
    args = parser.parse_args()

    if args.sim == bool(args.port):
        parser.error("give exactly one of --sim or --port")

    if args.sim:
        if not os.path.exists(args.sim_binary):
            raise SystemExit("%s is not built - run `make host` first" %
                             args.sim_binary)
        console = PipeConsole([args.sim_binary, "0", "console"])
        where = "the software-in-the-loop board"
    else:
        console = SerialConsole(args.port, args.baud, 0.2)
        where = args.port

    print("the bring-up checklist, against %s" % where, flush=True)

    runner = Runner(console, [])
    try:
        boot = runner.record(console.read_until(PROMPT, args.timeout))
        run_checks(runner, boot)
    finally:
        console.close()

    print("")
    print("verified:   %d" % len(runner.verified))
    print("not fitted: %d" % len(runner.absent))
    print("unverified: %d" % len(runner.unverified))
    print("failed:     %d" % len(runner.failed))

    if args.out:
        with open(args.out, "w") as handle:
            handle.write("AerialKit bench check\n")
            handle.write("  transport: %s\n" % where)
            handle.write("  when:      %s\n" %
                         datetime.datetime.now().isoformat(timespec="seconds"))
            handle.write("  verified:  %s\n" %
                         ", ".join(w for w, _ in runner.verified) or "nothing")
            handle.write("  not fitted: %s\n" %
                         ", ".join(w for w, _ in runner.absent) or "nothing")
            handle.write("  unverified: %s\n" %
                         ", ".join(w for w, _ in runner.unverified) or "nothing")
            if runner.failed:
                handle.write("  FAILED:    %s\n" %
                             "; ".join("%s (%s)" % (w, n)
                                       for w, n in runner.failed))
            handle.write("\n---- the session, as the console printed it ----\n\n")
            handle.write("".join(runner.transcript))
        print("transcript: %s" % args.out)

    return 1 if runner.failed else 0


if __name__ == "__main__":
    sys.exit(main())
