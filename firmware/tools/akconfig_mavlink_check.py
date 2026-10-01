#!/usr/bin/env python3
"""The configurator's window against an ArduPilot vehicle.

    make proto-test        # which runs this with the rest of the clients

Like `akconfig_msp_check.py`, one protocol further on: the same window, built
against `tools/mavlink_fake_vehicle.py` - which is a vehicle made out of
pymavlink, the reference implementation - and read back through its widgets.

MAVLink is the one foreign protocol that gives this window a *parameter table*:
`PARAM_REQUEST_LIST` is standardized, so an ArduPilot or PX4 vehicle answers
with every parameter by name and value, and the window's own table shows them.
That is the difference this check is mostly about - and the rest of it is the
same two promises the MSP check makes: the state panes fill from the vehicle's
own frames, and the write buttons refuse in the vehicle's own name.
"""

import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from akconfig_window_check import ensure_display, pump              # noqa: E402

FAKE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "mavlink_fake_vehicle.py")
ORACLE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "upstream",
    "pymavlink-2.4.49"))

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def no_oracle():
    """What is missing here, or None - and it is a reason, not a bool.

    The vehicle this window talks to is *built on* pymavlink: the frames it
    sends are the reference's, not this repository's idea of them. So a
    checkout without `upstream/pymavlink-2.4.49/` cannot run this check at all,
    and the honest answer is the one `mavlink_check.py` gives for the same
    reason - a line saying so, and a zero - rather than a failure. It cost a
    landing rehearsal to notice: the first version ran the window anyway, and a
    vehicle that exits for want of its oracle made the check die on
    `window.table.item(rows[0], "values")` with an empty table, which took
    `make proto-test` down with it. The directory is ignored rather than
    tracked (pymavlink is 1.6 MB of pure Python and LGPL-3;
    `scripts/fetch-upstreams.sh pymavlink-2.4.49` downloads the wheel
    revisions.json pins, checks its sha256 and extracts it), so this is the
    state every fresh clone is in.

    That command is here because it did not used to work. The script ran
    `git clone` on the entry's PyPI page and died with `fatal: repository not
    found`, and this message pointed at "its README" - a README *inside* the
    ignored directory, so it could not exist in the checkout that needed it.
    """
    if not os.path.isdir(ORACLE):
        return ("no pymavlink at %s (run scripts/fetch-upstreams.sh "
                "pymavlink-2.4.49)" % ORACLE)
    sys.path.insert(0, ORACLE)
    try:
        from pymavlink.dialects.v20 import common       # noqa: F401
    except ImportError as why:
        return "pymavlink at %s will not import (%s)" % (ORACLE, why)
    return None


def main():
    problem = ensure_display()
    if problem is not None:
        print("the window against a MAVLink vehicle: not checked here - %s"
              % problem)
        return 0

    problem = no_oracle()
    if problem is not None:
        print("the window against a MAVLink vehicle: not checked here - %s"
              % problem)
        return 0

    import tkinter

    import akconfig
    import msp

    print("the configurator's window, against an ArduPilot vehicle")

    process = subprocess.Popen([sys.executable, FAKE], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE)

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
    kind, configurator = akconfig.detect(reader.read_some, transport.write)
    expect("the window's first questions find a vehicle that is neither ours "
           "nor MSP", kind == "mavlink" and configurator is not None,
           " (%s)" % kind)
    if kind != "mavlink" or configurator is None:
        # A vehicle that never answered has no table to read, and asking it for
        # one used to be a traceback rather than the failure already reported
        # above. The reason it is a report rather than an early return *with*
        # zero: the oracle is here, so something is wrong with the client or
        # with the vehicle, and that is exactly what this check is for.
        print("  (no MAVLink vehicle answered - the check stops here)")
        transport.close()
        print("configurator window, MAVLink vehicle: FAILED")
        return 1

    link = akconfig.Link(configurator)
    root = tkinter.Tk()
    window = akconfig.Window(root, configurator, link)

    # The table takes a moment: the window asks for the parameter list as it
    # connects, and that is forty frames from the vehicle.
    pump(root, lambda: window.items != [], 20.0)
    label = window.aircraft.cget("text")
    expect("and it names the autopilot and the system it found",
           "ArduPilot" in label and "system 1" in label, " (%s)" % label)

    rows = window.table.get_children()
    expect("the parameter table is filled from the vehicle's own list",
           len(rows) == 40 and "RATE_RLL_P" in
           str(window.table.item(rows[0], "values")),
           " (%u rows, first %s)" % (len(rows),
                                     window.table.item(rows[0], "values")))
    expect("through the same interface ours uses: picking a row fills the "
           "edit field", configurator.count == 40, " (%u)" % configurator.count)

    status = window.status.cget("text")
    expect("the aircraft pane carries the attitude, the position and the pack "
           "from MAVLink's own messages",
           "attitude" in status and "52.1234" in status and "12.6" in status,
           " (%s)" % " / ".join(status.split("\n")))
    expect("and the logs pane says a vehicle's logs are not this window's "
           "business yet",
           window.logs.cget("text").count("not on this board") == 3,
           " (%s)" % window.logs.cget("text").replace("\n", " / "))

    first = rows[0]
    window.table.selection_set(first)
    window.pick()
    window.value.delete(0, "end")
    window.value.insert(0, "1.000")
    window.set()
    pump(root, lambda: window.message.cget("text").startswith("this is"))
    expect("setting a parameter on somebody else's vehicle is refused, in "
           "words",
           "ArduPilot" in window.message.cget("text"),
           " (%s)" % window.message.cget("text"))
    window.save()
    pump(root, lambda: window.message.cget("text").startswith("not saved"))
    expect("and saving says the same thing",
           window.message.cget("text").startswith("not saved"),
           " (%s)" % window.message.cget("text"))

    window.rate.set("5")
    window.toggle_stream()
    pump(root, lambda: window.live.cget("text") != "", 10.0)
    live = window.live.cget("text")
    expect("and the live pane fills from the frames the vehicle streams",
           "roll" in live and "motors" in live,
           " (%s)" % live.replace("\n", " / "))
    window.toggle_stream()
    # A deadline, not the next line: the button posts the stop to the link
    # thread, and this check raced it once under load. See the MSP one.
    pump(root, lambda: configurator.stream_hz == 0, 5.0)
    expect("and it stops when it is asked to", configurator.stream_hz == 0,
          " (%u Hz)" % configurator.stream_hz)

    # --- and the other vehicle that speaks this protocol --------------------
    #
    # PX4 is the fourth of the five firmwares the owner named, and it is the
    # one this window can only reach through somebody else's implementation:
    # same protocol, same messages, different name - and the name is what the
    # refusals are made of ("this is ArduPilot ... does not write to it"), so
    # the pairing of heartbeat field to sentence is worth its own check rather
    # than being assumed from the ArduPilot one passing.
    process.kill()
    process = subprocess.Popen([sys.executable, FAKE, "--px4"],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    transport = Pipe()
    reader = msp.Deadline(transport)
    kind, configurator = akconfig.detect(reader.read_some, transport.write)
    expect("a PX4 vehicle is found the same way",
           kind == "mavlink" and configurator is not None, " (%s)" % kind)
    root2 = tkinter.Tk()
    window2 = akconfig.Window(root2, configurator, akconfig.Link(configurator))
    pump(root2, lambda: window2.aircraft.cget("text") != "connecting...")
    expect("and it is named as PX4, with the system it reports",
           "PX4" in window2.aircraft.cget("text") and
           "system 1" in window2.aircraft.cget("text"),
           " (%s)" % window2.aircraft.cget("text"))
    pump(root2, lambda: window2.items != [], 20.0)
    expect("and its parameter table fills from its own list",
           len(window2.table.get_children()) == 40,
           " (%u rows)" % len(window2.table.get_children()))
    rows = window2.table.get_children()
    window2.table.selection_set(rows[0])
    window2.pick()
    window2.value.insert(0, "1.000")
    window2.set()
    # Wait for the *answer*, not for any text: the message label's first words
    # are "connecting...", so "not empty" is satisfied before the link thread
    # has replied at all - which is what the first version of this check read.
    pump(root2, lambda: "PX4" in window2.message.cget("text"), 10.0)
    expect("and a write is refused in PX4's name rather than in ArduPilot's",
           "PX4" in window2.message.cget("text") and
           "ArduPilot" not in window2.message.cget("text"),
           " (%s)" % window2.message.cget("text").replace("\n", " / "))
    root2.destroy()
    transport.close()
    process.kill()

    root.destroy()
    transport.close()

    print("configurator window, MAVLink vehicle: %s"
          % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
