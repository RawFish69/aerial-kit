#!/usr/bin/env python3
"""The protocol client's own command line, run the way the pages document it.

    python3 tools/akproto_cli_check.py [--out DIR]      # make test runs this

Why this exists. `tools/akproto.py` is the client a *person* uses - the pages
print its command lines (`docs/16-protocol.md` has eight of them, and
`23-first-flight.md` ends a flight with `log --source flash`), and every check
in this repository imports its `Client` class and never runs its `main()`. So
the thing that runs after the first flight - the client, over a cable, writing
the log to a file - was the one part of that path with nothing behind it but the
pages, and a documented command line that has never been executed is exactly
what this project keeps finding things in.

The board is the protocol simulator on the far end of a pty (the same harness
`tools/bench_port_check.py` uses for the bench tool), and each case is the
documented command line itself, as a subprocess:

    akproto.py --port <pty> hello | status | list | get N
    akproto.py --port <pty> log out.csv
    akproto.py --port <pty> --source flash log crash.csv
    akproto.py --host 127.0.0.1:<port> status
    akproto.py --host 127.0.0.1:<port> telemetry 5

The last one is the socket path a person uses when the board is an ESP32 on a
network, and it is a *different* transport in `open_client()` from the serial
one - so it is checked rather than assumed, through a TCP-to-pipe bridge to the
same simulator.
"""

import argparse
import os
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bench_port_check import Cable                          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CLIENT = os.path.join(HERE, "akproto.py")
SIM = os.environ.get("AK_SIM", "build-host/aerialkit-sim")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def run(argv, timeout=60):
    return subprocess.run([sys.executable, CLIENT] + argv, capture_output=True,
                          text=True, timeout=timeout)


def serial_cases(where):
    """Every `--port` line the pages show, against the simulator on a pty."""
    cable = Cable()
    cable.start_simulator()

    out = run(["--port", cable.device, "hello"])
    expect("`akproto.py --port ... hello` speaks to a board over a cable",
           out.returncode == 0 and "protocol 1," in out.stdout and
           "parameters" in out.stdout and "changed since saved" in out.stdout,
           " (%s)" % out.stdout.strip().replace("\n", " / ")[:90])

    out = run(["--port", cable.device, "status"])
    expect("`status` prints the state, the attitude and the motors",
           out.returncode == 0 and "state " in out.stdout and
           "attitude roll" in out.stdout and "motors " in out.stdout,
           " (%s)" % out.stdout.strip().replace("\n", " / ")[:90])

    out = run(["--port", cable.device, "list"])
    rows = [line for line in out.stdout.splitlines() if "  " in line]
    expect("`list` walks the table the board reports",
           out.returncode == 0 and len(rows) > 20 and
           any("airframe" in line for line in rows),
           " (%u rows)" % len(rows))

    out = run(["--port", cable.device, "get", "0"])
    expect("`get N` names the parameter rather than only its index",
           out.returncode == 0 and " = " in out.stdout,
           " (%s)" % out.stdout.strip())

    # The one the first-flight page ends with: the log that survives the
    # battery, written to a file.
    flight = os.path.join(where, "flight.csv")
    out = run(["--port", cable.device, "log", flight])
    header = ""
    rows = 0
    if os.path.exists(flight):
        with open(flight, encoding="utf-8", errors="replace") as handle:
            lines = handle.read().splitlines()
        # The file opens with the log's own comment lines (which log it is and
        # what the units are), then the column header, then the records - so the
        # header is the first line that is not a comment, and the rows are the
        # rest after it.
        body = [line for line in lines if line and not line.startswith("#")]
        header = body[0] if body else ""
        rows = max(0, len(body) - 1)
    expect("`log out.csv` writes the fast log, header and rows",
           out.returncode == 0 and "wrote" in out.stdout and rows > 10 and
           header.startswith("time_ms,"),
           " (%u rows, header %s)" % (rows, header[:40]))

    crash = os.path.join(where, "crash.csv")
    out = run(["--port", cable.device, "--source", "flash", "log", crash])
    flash_rows = 0
    if os.path.exists(crash):
        with open(crash, encoding="utf-8", errors="replace") as handle:
            body = [line for line in handle.read().splitlines()
                    if line and not line.startswith("#")]
        flash_rows = max(0, len(body) - 1)
    expect("`--source flash log crash.csv` pulls the log that survives a crash",
           out.returncode == 0 and "wrote" in out.stdout and flash_rows > 10,
           " (%u rows, %s%s)"
           % (flash_rows, out.stdout.strip(),
              out.stderr.strip()[:120] if out.returncode else ""))

    cable.close()


class SocketBridge:
    """A TCP listener with the simulator's pipes on the other side.

    The socket path is a different branch of `open_client()` - and the one a
    board on a network actually answers on - so it gets its own case rather
    than being assumed to work because the serial one does.

    **One connection per simulator, and the listener stays up.** The first
    version of this accepted a single connection: a client that reconnected -
    or a second command line - sat in the backlog unanswered, which looked
    exactly like a board that stopped answering. A board on a network serves
    whoever reconnects, so this does too: each accepted connection gets its own
    simulator, and both ends are torn down when that connection closes."""

    def __init__(self):
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.stop = False
        self.sims = []

    def serve(self):
        while not self.stop:
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            sim = subprocess.Popen([SIM], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE)
            self.sims.append(sim)
            done = threading.Event()

            def to_sim(conn=conn, sim=sim, done=done):
                while not self.stop and not done.is_set():
                    ready, _, _ = select.select([conn], [], [], 0.2)
                    if ready:
                        data = conn.recv(4096)
                        if not data:
                            done.set()
                            return
                        os.write(sim.stdin.fileno(), data)

            def from_sim(conn=conn, sim=sim, done=done):
                # os.read, not `sim.stdout.read`: a buffered reader pulls a whole
                # chunk out of the pipe on the first call, so gating the next
                # call on `select` waits for a file descriptor whose bytes are
                # already in Python's buffer - which is how the first version of
                # this bridge delivered one byte of a seven-byte reply and then
                # timed out.
                while not self.stop and not done.is_set():
                    ready, _, _ = select.select([sim.stdout], [], [], 0.2)
                    if ready:
                        data = os.read(sim.stdout.fileno(), 4096)
                        if not data:
                            done.set()
                            return
                        conn.sendall(data)

            for target in (to_sim, from_sim):
                threading.Thread(target=target, daemon=True).start()

    def close(self):
        self.stop = True
        self.server.close()
        for sim in self.sims:
            sim.kill()
            sim.wait(timeout=10)


def socket_case():
    bridge = SocketBridge()
    thread = threading.Thread(target=bridge.serve, daemon=True)
    thread.start()
    time.sleep(0.2)
    out = run(["--host", "127.0.0.1:%u" % bridge.port, "status"])
    expect("`akproto.py --host HOST:PORT status` speaks to a board on a network",
           out.returncode == 0 and "state " in out.stdout and
           "attitude roll" in out.stdout,
           " (%s)" % (out.stdout.strip().replace("\n", " / ")[:90] or
                      out.stderr.strip()[:90]))

    # The streaming line, `--host ... telemetry N`, is the one documented
    # command this check does *not* drive to a stream: the protocol simulator
    # behind this socket never pushes frames of its own (it answers a byte at a
    # time), and the firmware only streams on a *network* link - which on this
    # machine means the ESP32 under QEMU, where `make net-window-test` and
    # `scripts/esp32-proto.sh` drive the same `subscribe()`/`next_telemetry()`
    # pair through the window. What *is* worth pinning here is the refusal a
    # person gets when they point that line at a link that cannot push: it has
    # to say so and exit, not sit there looking like a board that went quiet.
    out = run(["--host", "127.0.0.1:%u" % bridge.port, "telemetry", "5"])
    expect("`--host ... telemetry N` says so when the link cannot stream",
           out.returncode == 1 and "will not stream" in out.stderr,
           " (rc %d, %s)" % (out.returncode,
                             out.stderr.strip().splitlines()[-1][:70]))
    bridge.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None,
                        help="directory for the CSV dumps (default: temporary)")
    args = parser.parse_args()

    if not os.path.exists(SIM):
        print("the protocol client's command line: not checked here - no "
              "simulator at %s (make host)" % SIM)
        return 0

    scratch = None
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        where = args.out
    else:
        scratch = tempfile.TemporaryDirectory(prefix="ak-proto-cli-")
        where = scratch.name

    print("the protocol client's command line, as the pages print it")
    serial_cases(where)
    socket_case()

    if scratch is not None:
        scratch.cleanup()
    print("protocol client's command line: %s"
          % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
