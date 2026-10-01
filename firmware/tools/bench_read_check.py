#!/usr/bin/env python3
"""The bench tool's read, against a console that prints a prompt nobody asked for.

    python3 tools/bench_read_check.py          # make test runs this

Why this exists. `tools/bench_check.py` frames every command by reading the port
until it sees the prompt, `"ak> "` - that being the only thing the firmware
prints that means "the last command has finished". The read was

    if needle in seen: break

with `seen` accumulating from the moment the read starts, so a prompt *already*
on the wire ended the read on its first byte. The checklist would ask for the
parameter table, get the leftover prompt from the command before, find no table
in it, and report "no table came back" - while the table itself arrived a moment
later and was thrown away unread. That is the flake this file is named after,
and it was never the board: every row was there when read directly.

Where a stray prompt comes from at a bench is the other half of the story. The
console and the binary config protocol share the port (docs/16-protocol.md), and
until 2026-09-28 a frame the firmware's parser abandoned over a bad length left
its tail arriving at the console - printable bytes into the line buffer, and a
CR or LF among them into a prompt no command had asked for. The firmware end is
fixed (`src/core/ak_console_link.c`); this is the script end, and it is fixed
too rather than only being made unnecessary, because a read that a stray prompt
can truncate is a read the next stray byte will truncate again.

So the far end here is deliberately not the simulator. It is a pty carrying a
board that behaves, except that it is willing to print a prompt of its own - and
the check drives `bench_check`'s own checks, not copies of them, so what is
being checked is what the bench actually runs.

Four cases in all. The first two are the ones the reported flake had, because a
stray prompt arrives at one of two moments and the two need different things:

* **a prompt already on the wire when the next command is sent**, which is the
  reported flake and the shape it really had: the stray bytes belong to whatever
  was on the wire *before*, and they are sitting in the buffer when the read
  starts. `Runner.command` draining the wire before it writes is what answers
  this, and the case is asserted on a short answer (`version`) as well as the
  long one - the table check has a second line of defence that would otherwise
  hide a missing drain;
* **a prompt that arrives inside an answer already being read**. `drain` cannot
  help, because the stray bytes land after the command was written. What answers
  it is the table check saying what a whole answer looks like: its header counts
  the rows, so the read carries on until they are there. It must not end up
  saying "no table came back" - that message blames the board for silence when
  the board answered - and where a row really was lost among the stray bytes it
  must say so with the numbers in it.

A **fourth** case is the one this file earned the hard way. The drain that
clears a stale prompt waits for the wire to go quiet, and a board with
something of its own to say never does: the software-in-the-loop board prints a
10 s heartbeat, and because its clock runs about 600x real time that line lands
every ~17 ms of wall time. The wait for `quiet` seconds of silence was
therefore a wait forever, and the whole checklist stalled behind it at its
first command. A fixture that strays cannot find that - it is silent between
commands by construction - so the case is a board that talks on its own
account, and it fails by *not finishing*, which is why it is given a thread and
a deadline.

The last case is the control: a board that strays not at all reads whole, so a
pass above is the read working rather than the fixture never being reached.

The stray-inside case is the slow one, and by design: a row really is lost, so
the table check reads on until its window closes before saying so. That is a
read window burning out, not a hang, and it is the reason this file takes half
a minute rather than a second.

`pyserial` is what the `--port` path uses; without it this says so and skips,
the same way the other port checks do.
"""

import argparse
import contextlib
import io
import os
import pty
import select
import sys
import termios
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bench_check                                              # noqa: E402

# What the board answers `params` with: a header the check parses the declared
# count out of, and rows it counts. Well past the check's own floor of 20.
ROWS = 92

BOOT = ("AerialKit F405  (aerialkit-f405, rev A)\r\n"
        "board:    AERIALKIT_F405\r\n"
        "ak> ")

VERSION = ("product:  AerialKit\r\n"
           "board:    AERIALKIT_F405\r\n"
           "rev:      test\r\n"
           "ak> ")


def table():
    return ("params (%u)\r\n" % ROWS
            + "".join("  p%02u        %u\n" % (i, i) for i in range(ROWS))
            + "ak> ")


failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name,
                         (" - " + detail) if detail else ""))
    if not condition:
        failures += 1


def raw(fd):
    """A pty endpoint with nothing between bytes: no echo, no line discipline.
    pyserial does this to the tool's end of the cable; this side has to match
    it, or what crosses is the terminal's edit of what was written."""
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                     # iflag
    attrs[1] = 0                                     # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                                     # lflag
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class Cable:
    """A pty, a board on the master end, and a device path on the other.

    The board answers lines. What makes it the board under test is the two
    things it is willing to do that a board should not: print a prompt on its
    own account between commands, and print one in the middle of an answer.
    """

    def __init__(self, stray_between=False, stray_inside=False, chatty=False):
        self.master, slave = pty.openpty()
        raw(self.master)
        raw(slave)
        self.device = os.ttyname(slave)
        # The tool opens this path itself; this side is held open so the pty
        # does not disappear (a pty with no slave open reads as end of file,
        # which is a different failure from the one under test).
        self.hold = os.open(self.device, os.O_RDWR | os.O_NOCTTY)
        raw(self.hold)
        self.stray_between = stray_between
        self.stray_inside = stray_inside
        self.chatty = chatty
        self.strayed = False
        self.stop = threading.Event()
        self.board = threading.Thread(target=self._board, daemon=True)
        self.board.start()

    def _say(self, text):
        os.write(self.master, text.encode())

    def _stray(self):
        """Four bytes no command asked for, which a read must not be framed by."""
        os.write(self.master, b"ak> ")
        self.strayed = True

    def _board(self):
        self._say(BOOT)
        seen = bytearray()
        beats = 0
        while not self.stop.is_set():
            if self.chatty:
                # Nothing of the check's is waiting to be read, so the board
                # says something of its own - the simulator's heartbeat, and
                # at the rate the simulator produces it: a clock about 600x
                # real time turns a 10 s period into ~17 ms of wall time.
                ready, _, _ = select.select([self.master], [], [], 0.05)
                if not ready:
                    beats += 1
                    self._say("alive: %u ms, %u loops\n"
                              % (beats * 10000, beats * 123456))
                    continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if not data:
                return
            seen += data
            while b"\r" in seen:
                line, _, rest = seen.partition(b"\r")
                seen = bytearray(rest)
                if self._answer(line.decode("utf-8", "replace").strip()):
                    return

    def _answer(self, line):
        """One command, answered the way the firmware would. False to stop."""
        if line == "params":
            header = "params (%u)\r\n" % ROWS
            rows = "".join("  p%02u        %u\n" % (i, i) for i in range(ROWS))
            if self.stray_inside:
                # The answer starts, and then a prompt nobody asked for lands
                # in the middle of it - the shape the firmware used to make out
                # of an abandoned frame's tail.
                self._say(header)
                time.sleep(0.05)
                self._say("ak> ")
                time.sleep(0.05)
                self._say(rows)
            else:
                self._say(header + rows)
            self._say("ak> ")
            return False
        if line == "version":
            self._say(VERSION)
            if self.stray_between and not self.strayed:
                # The command before this one is finished and answered. A
                # moment later the wire produces a prompt anyway - which is
                # what now sits in the buffer when the *next* read starts.
                time.sleep(0.05)
                self._stray()
            return False
        self._say("ak> ")
        return False

    def close(self):
        self.stop.set()
        os.close(self.hold)
        try:
            os.close(self.master)
        except OSError:
            pass


@contextlib.contextmanager
def session(stray_between=False, stray_inside=False, chatty=False):
    """A board, a port, and the boot report and first command already read.

    The first command matters: it is what the stray bytes land *after*, which
    is where they come from at a bench - whatever was on the wire before."""
    cable = Cable(stray_between=stray_between, stray_inside=stray_inside,
                  chatty=chatty)
    console = None
    try:
        console = bench_check.SerialConsole(cable.device, 115200, 0.2)
        runner = bench_check.Runner(console, [])
        boot = runner.record(console.read_until(bench_check.PROMPT, 5.0))
        if bench_check.PROMPT_TEXT not in boot:
            # Same recovery the tool itself makes for a board that had already
            # booted when the port was opened.
            runner.command("", timeout=3.0)
        runner.command("version", timeout=5.0)
        yield runner
    finally:
        if console is not None:
            console.close()
        cable.close()


def table_check(runner):
    """The check under test, and what it printed.

    Its output is caught rather than let loose, because one case here ends in it
    reporting a short table *on purpose*, and a `FAILED` line in this file's own
    output would read as this file failing."""
    verdict = io.StringIO()
    with contextlib.redirect_stdout(verdict):
        bench_check.check_parameter_table(runner)
    return " ".join(verdict.getvalue().split())


def case_stray_between():
    """The reported flake: a leftover prompt is on the wire when a read starts."""
    with session(stray_between=True, stray_inside=False) as runner:
        # The stray prompt arrives a little after the command before it was
        # answered; give it that moment, so it is in the buffer rather than
        # racing the next read.
        time.sleep(0.3)

        # A short answer first. The table check has a second line of defence -
        # `until`, which reads on until the rows the header promised are there
        # - and that would cover for a missing drain and let this case pass for
        # the wrong reason. Nothing covers for it here.
        answer = runner.command("version", timeout=5.0)
        expect("a prompt already on the wire: the next command reads its answer",
               "product:" in answer, answer.strip())

        said = table_check(runner)

    expect("a prompt already on the wire: the table is read whole",
           "the parameter table" in [w for w, _ in runner.verified], said)
    return runner


def case_stray_inside():
    """A prompt that arrives once the answer is already being read.

    Whatever is said about the table, it is not that the board said nothing. A
    row can genuinely be lost among stray bytes - they land wherever they land,
    including inside a row. What must not happen is the tool turning that into
    "no table came back", which is a claim about the board and a false one: the
    board answered, and the answer is in the transcript.
    """
    with session(stray_between=False, stray_inside=True) as runner:
        said = table_check(runner)

    notes = [n for _, n in runner.failed]
    whole = bool(runner.verified)
    expect("a prompt inside the answer: the board is not reported as silent",
           whole or all(n != "no table came back" for n in notes), said)
    expect("a prompt inside the answer: what is said carries the count",
           whole or any("declared parameters" in n for n in notes), said)
    return runner


def case_chatty():
    """A board that talks on its own account, so `drain` can never see quiet.

    This is the simulator's shape, and the other cases here could not have
    found it: they are silent between commands, so waiting for silence looks
    harmless.

    The whole case runs in a thread, not just the table check, and that is not
    tidiness: the regression is a hang, and the first place it hangs is
    `session`'s own first command, before any assertion could be reached. A
    check for a hang has to be able to outlive one, or it takes the run down
    with it - which is what this one did, silently, until it was given a
    deadline. The 20 s is far past the whole of a healthy case (~7 s).
    """
    outcome = []
    runners = []

    def run():
        try:
            with session(chatty=True) as runner:
                runners.append(runner)
                outcome.append(table_check(runner))
        except Exception as error:                  # the port cut out from under it
            outcome.append("the case ended: %s" % error)

    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    worker.join(20.0)
    stuck = worker.is_alive()
    said = outcome[0] if outcome else ""
    expect("a board that talks on its own account: the read still ends",
           not stuck and "the parameter table" in said,
           "still reading 20 s later - a wait for silence on a wire that "
           "never goes quiet" if stuck else said)
    return runners[0] if runners else None


def case_control():
    """A board that does none of this, so a pass above is the read working."""
    with session(stray_between=False, stray_inside=False) as runner:
        said = table_check(runner)

    expect("a board that strays not at all: the table is read whole",
           "the parameter table" in [w for w, _ in runner.verified], said)
    return runner


CASES = (("stray-between", case_stray_between),
         ("stray-inside", case_stray_inside),
         ("chatty", case_chatty),
         ("control", case_control))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None,
                        help="directory for a transcript of each case's wire "
                             "(default: none - the verdict lines are the check)")
    args = parser.parse_args()

    try:
        import serial                                          # noqa: F401
    except ImportError:
        print("the bench tool's read: not checked here - no pyserial "
              "(apt install python3-serial)")
        return 0

    if args.out:
        os.makedirs(args.out, exist_ok=True)

    print("the bench tool's read, against a console that strays")
    for name, case in CASES:
        runner = case()
        if args.out and runner is not None:
            # What the cable actually carried, for a case that fails: every
            # assertion above already prints the words it judged, but a read
            # that went wrong is easier to see in the whole of what arrived.
            path = os.path.join(args.out, "bench-read-%s.log" % name)
            with open(path, "w") as handle:
                handle.write("".join(runner.transcript))
    print("")
    print("the bench tool's read: %d failed" % failures)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
