#!/usr/bin/env python3
"""The bench checklist through a *serial port*, before there is a board.

    python3 tools/bench_port_check.py [--out DIR]      # make test runs this

Why this exists. `tools/bench_check.py` has two ways to reach a console: a pipe
to the simulator (`--sim`, which `make bench-check` uses on this machine and
`make test` runs) and a real serial device (`--port /dev/ttyACM0`, which is the
one a person uses at the bench). The second had never been executed anywhere -
there is no board - and the first time a path runs is the worst time to find out
that it is wrong, which is what the rest of this project keeps saying about
builds and sessions.

So the board is a pty pair with the simulator on the far end, and the tool is
pointed at the device path of the other side:

    [ simulator ] <-> [ pty A ] <-> bridge <-> [ pty B ] <-> [ bench_check --port ]

Two ptys and a byte bridge, because a pty is a master/slave pair rather than a
wire: the simulator writing into its slave is read from A's *master*, and the
tool opening B's slave is talking to B's master. The bridge is what makes the
two halves one cable, and it is deliberately dumb - bytes across, nothing else.

Three cases, because the interesting one is not "it works":

* **the board boots while the tool waits** - the flow the tool was written for,
  and the one `--sim` exercises;
* **the board had already booted when the tool opened the port** - the usual
  bench case (plug the cable in, then type the command). This is where the
  check earned its keep: the tool used to fail the whole run here with "no
  prompt after the boot report", because the boot report went out before the
  process had the port open and neither a pty nor a USB CDC device replays it.
  It now asks a bare newline and says in the note which of the two cases the
  transcript is;
* **nothing on the other end** - a pty with no writer - which must still be a
  failure with words, or the check would pass on a cable nobody plugged in.

`pyserial` is what the tool's `--port` path uses, and it is installed here as
the distribution's `python3-serial`; without it this check says so and skips,
the same way the MAVLink checks skip without their oracle.
"""

import argparse
import os
import pty
import select
import subprocess
import sys
import tempfile
import termios
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.join(HERE, "bench_check.py")
SIM = os.environ.get("AK_SIM", "build-host/aerialkit-fw-sim")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def raw(fd):
    """A pty endpoint with nothing between bytes: no echo, no line discipline.
    pyserial does this to the tool's end of the cable; the bridge does it to
    both ends so that what crosses is what was written."""
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                     # iflag
    attrs[1] = 0                                     # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                                     # lflag
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class Cable:
    """Two ptys, one bridge, and a device path for the tool to open."""

    def __init__(self):
        self.a_master, a_slave = pty.openpty()
        self.b_master, b_slave = pty.openpty()
        for fd in (self.a_master, a_slave, self.b_master, b_slave):
            raw(fd)
        self.device = os.ttyname(b_slave)
        # The tool opens this path itself; this side is kept open so that the
        # pty does not disappear (a pty with no slave open reads as end of
        # file, which is a different failure from the one under test).
        self.hold = os.open(self.device, os.O_RDWR | os.O_NOCTTY)
        raw(self.hold)
        # The simulator gets the slave end of the other pty as its stdio.
        self.sim_slave = a_slave
        self.sim = None
        self.bridge = threading.Thread(target=self._bridge, daemon=True)
        self.bridge.start()

    def _bridge(self):
        while True:
            try:
                ready, _, _ = select.select([self.a_master, self.b_master],
                                            [], [], 0.2)
            except (OSError, ValueError):
                return
            for fd in ready:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    return
                if not data:
                    return
                try:
                    os.write(self.b_master if fd == self.a_master
                             else self.a_master, data)
                except OSError:
                    return

    def start_simulator(self):
        self.sim = subprocess.Popen([SIM, "0", "console"],
                                    stdin=self.sim_slave,
                                    stdout=self.sim_slave,
                                    stderr=subprocess.DEVNULL,
                                    close_fds=True)
        return self.sim

    def close(self):
        if self.sim is not None:
            self.sim.kill()
            self.sim.wait(timeout=10)
        os.close(self.hold)
        for fd in (self.a_master, self.b_master, self.sim_slave):
            try:
                os.close(fd)
            except OSError:
                pass


def run_tool(device, out, timeout):
    return subprocess.run([sys.executable, BENCH, "--port", device,
                           "--out", out, "--timeout", str(timeout)],
                          capture_output=True, text=True, timeout=600)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None,
                        help="directory for the transcripts (default: a "
                             "temporary one)")
    args = parser.parse_args()

    try:
        import serial                                          # noqa: F401
    except ImportError:
        print("the bench tool through a serial port: not checked here - no "
              "pyserial (apt install python3-serial)")
        return 0
    if not os.path.exists(SIM):
        print("the bench tool through a serial port: not checked here - no "
              "simulator at %s (make host)" % SIM)
        return 0

    scratch = None
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        where = args.out
    else:
        scratch = tempfile.TemporaryDirectory(prefix="ak-bench-port-")
        where = scratch.name

    print("the bench tool, through a serial port, against the simulator")

    # --- 1. the board boots while the tool waits -------------------------
    cable = Cable()
    tool = subprocess.Popen([sys.executable, BENCH, "--port", cable.device,
                             "--out", os.path.join(where, "bench-port-boot.log"),
                             "--timeout", "20"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)
    time.sleep(1.0)                       # the tool is at the port, waiting
    cable.start_simulator()               # ... and now the board comes up
    out, _ = tool.communicate(timeout=600)
    log = open(os.path.join(where, "bench-port-boot.log"),
               encoding="utf-8", errors="replace").read()
    expect("a board that boots while the tool waits passes the checklist",
           tool.returncode == 0 and "FAILED" not in out,
           " (rc %d)" % tool.returncode)
    expect("and the boot report is in the transcript",
           "product:" in log and "ak> " in log,
           " (%d bytes)" % len(log))
    cable.close()

    # --- 2. the board was already running when the port was opened -------
    cable = Cable()
    cable.start_simulator()
    time.sleep(1.0)
    tool = subprocess.run([sys.executable, BENCH, "--port", cable.device,
                           "--out", os.path.join(where, "bench-port-running.log"),
                           "--timeout", "6"],
                          capture_output=True, text=True, timeout=600)
    log = open(os.path.join(where, "bench-port-running.log"),
               encoding="utf-8", errors="replace").read()
    expect("a board that had already booted is a checklist run, not a failure",
           tool.returncode == 0 and "the console is listening" in tool.stdout \
           and "booted" in tool.stdout,
           " (rc %d, %s)" % (tool.returncode,
                             tool.stdout.splitlines()[1] if
                             len(tool.stdout.splitlines()) > 1 else ""))
    expect("and its transcript says the boot report is the missing part",
           "product:" not in log.split("----")[0],
           " (%d bytes)" % len(log))
    cable.close()

    # --- 3. nothing on the other end -------------------------------------
    cable = Cable()
    tool = subprocess.run([sys.executable, BENCH, "--port", cable.device,
                           "--out", os.path.join(where, "bench-port-dead.log"),
                           "--timeout", "2"],
                          capture_output=True, text=True, timeout=600)
    expect("a cable with nothing on the other end fails, with words",
           tool.returncode != 0 and
           "no prompt, before or after a bare newline" in tool.stdout,
           " (rc %d)" % tool.returncode)
    cable.close()

    if scratch is not None:
        scratch.cleanup()

    print("bench tool through a serial port: %s"
          % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
