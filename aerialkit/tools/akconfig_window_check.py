#!/usr/bin/env python3
"""The configurator's *window*, built and driven on a machine with no screen.

    make window-test        # or: make proto-test, which runs this with it

`akconfig_check.py` drives the controller under the window and says in as many
words that "a window cannot be checked in a test". That is true of a person
looking at it and false of everything else: a Tk widget tree can be built,
filled, clicked and read back without anybody watching, which is what this
does. It is the difference between "the table is whole" and "the table is
whole *and the window that shows it says so*".

Tk needs a display, and the machine this firmware is built on is a Pi with no
X server and no monitor. So the check runs itself under Xvfb when there is no
DISPLAY and `xvfb-run` is installed, and skips with a line when there is
neither - a machine without Tk should not fail the firmware's own suite.

What it cannot do is decide whether the window is any *good*: whether the
columns are wide enough, whether the buttons are where a hand expects them.
That is still a person's, and this says which half it is holding.
"""

import os
import shutil
import sys
import time

SIM = os.environ.get("AK_SIM", "build-host/aerialkit-sim")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def skip(reason):
    print("the configurator's window: not checked here - %s" % reason)
    return 0


def ensure_display():
    """A display to build Tk widgets on, or a reason there will not be one.

    Returns None when the caller has one (a real X server, or the virtual one
    this re-execs into), and a string when the check should skip. The other
    window check imports this rather than growing a second copy of it.
    """
    try:
        import tkinter                                  # noqa: F401
    except ImportError:
        return "no Tk on this machine (python3-tk is the package)"

    if not os.environ.get("DISPLAY"):
        if not shutil.which("xvfb-run"):
            return "no DISPLAY and no xvfb-run to make one"
        # Re-exec under a virtual framebuffer. execvp, not a subprocess: the
        # checks below print to this process's stdout, and the exit status has
        # to be the check's, not a wrapper's.
        os.execvp("xvfb-run", ["xvfb-run", "-a", sys.executable] + sys.argv)
    return None


def pump(root, until, seconds=5.0):
    """Run Tk's loop by hand until `until()` is true or the clock runs out.

    The window posts its protocol calls on a thread and reads the replies back
    in an `after()` callback, so "let Tk do its work" is `update()` in a loop -
    `mainloop()` would never return to whoever is asserting.
    """
    deadline = time.time() + seconds
    while time.time() < deadline:
        root.update()
        if until():
            return True
        time.sleep(0.01)
    root.update()
    return until()


def main():
    problem = ensure_display()
    if problem is not None:
        return skip(problem)

    import tkinter

    # Everything above is about the machine; everything below is the firmware.
    from akconfig import Configurator, Link, Pipe, Window
    from akproto import Client

    print("the configurator's window, against the firmware's own protocol code")

    transport = Pipe([SIM])
    configurator = Configurator(Client(transport.read, transport.write))
    link = Link(configurator)

    root = tkinter.Tk()
    window = Window(root, configurator, link)

    # The window asks for the table as it is built, so "connected" is the
    # aircraft label saying what answered rather than the word "connecting".
    pump(root, lambda: window.items != [])
    expect("the window builds and the link comes up",
           "aerialkit" in window.aircraft.cget("text"),
           " (%s)" % window.aircraft.cget("text"))

    rows = window.table.get_children()
    expect("the table is filled from the aircraft's own parameter list",
           len(rows) == configurator.count == len(configurator.items),
           " (%u rows of %u)" % (len(rows), configurator.count))

    # The status and the logs are filled by the same reply, and they are the
    # two panes a pilot reads before a flight.
    status = window.status.cget("text")
    expect("the aircraft pane draws the state, the attitude and the motors",
           "state" in status and "attitude" in status and "motors" in status)
    logs = window.logs.cget("text")
    expect("and the three logs are counted, with a floor for the one this "
           "aircraft has no answer for",
           logs.count("records") == 3 or "not on this board" in logs,
           " (%s)" % " / ".join(logs.split("\n")))

    # Picking a row is what puts a value in the entry field, and the filter is
    # what makes a table of ninety parameters usable.
    first = rows[0]
    window.table.selection_set(first)
    window.pick()
    picked = window.value.get()
    expect("picking a row puts that parameter's value in the edit field",
           picked != "", " (%s = %s)" % (first, picked))

    window.filter.insert(0, "rate_kp_roll")
    window.fill_table()
    filtered = window.table.get_children()
    expect("the filter narrows the table to the names that match",
           0 < len(filtered) < len(rows), " (%u of %u)" % (len(filtered), len(rows)))
    window.filter.delete(0, "end")
    window.fill_table()

    # A value inside the range is taken, and the window says so in the board's
    # own words - which is the whole reason `param set` replies with a message.
    row = str([i for i, name, _v in configurator.items
               if name == "rate_kp_roll"][0])
    window.table.selection_set(row)
    window.pick()
    window.value.delete(0, "end")
    window.value.insert(0, "0.600")
    window.set()
    pump(root, lambda: window.message.cget("text") == "set")
    expect("a value in range is written and the window says so",
           window.message.cget("text") == "set" and
           "0.600" in str(window.table.item(row, "values")),
           " (%s)" % window.message.cget("text"))

    window.value.delete(0, "end")
    window.value.insert(0, "9.500")
    window.set()
    pump(root, lambda: window.message.cget("text").startswith("refused"))
    refused = window.message.cget("text")
    expect("a value out of range is refused, with the board's reason shown",
           "out of range" in refused, " (%s)" % refused)
    expect("and the table still shows the value the aircraft is actually on",
           "0.600" in str(window.table.item(row, "values")),
           " (%s)" % str(window.table.item(row, "values")))

    # Undo, then the board's own write: the two buttons whose meaning a person
    # has to be able to trust.
    window.revert()
    pump(root, lambda: window.message.cget("text").startswith("reverted"))
    expect("revert puts back the value the session started with",
           "0.250" in str(window.table.item(row, "values")),
           " (%s)" % str(window.table.item(row, "values")))

    window.save()
    pump(root, lambda: window.message.cget("text") == "saved")
    expect("save is the board's own write, and the changed count goes to zero",
           window.changed.cget("text").startswith("0 changed"),
           " (%s)" % window.changed.cget("text"))

    # And the logs, which is the thing a person opens a configurator *for*
    # after a flight: pull one off the aircraft and draw it. The plot is a Tk
    # canvas, so "did it draw" is a question about its items rather than about
    # anybody's eyes.
    window.log_source.set("flash")
    window.pull()
    pump(root, lambda: "drawn" in window.log_note.cget("text"))
    items = window.plot.find_all()
    expect("a log can be pulled and drawn in the window",
           len(items) >= 2 and "3 records" in window.log_note.cget("text"),
           " (%s, %u canvas items)" % (window.log_note.cget("text"), len(items)))
    expect("with one trace per attitude angle",
           sum(1 for item in items
               if window.plot.type(item) == "line") >= 4,
           " (%s)" % ", ".join(window.plot.type(i) for i in items))

    # A log this aircraft does not have says so, rather than drawing an empty
    # box that looks like a flight with no attitude in it.
    window.log_source.set("long")
    window.pull()
    pump(root, lambda: "no long log" in window.log_note.cget("text"))
    expect("a log the aircraft does not have is said, not drawn",
           "no long log" in window.log_note.cget("text") and
               len(window.plot.find_all()) == 0,
           " (%s)" % window.log_note.cget("text"))

    # And the live pane: a console link is a wire somebody types at, so asking
    # the window for a stream has to come back with the reason rather than a
    # rate, and the button has to go back to where it started.
    window.rate.set("10")
    window.toggle_stream()
    pump(root, lambda: window.message.cget("text").startswith("this link cannot"))
    expect("asking for a stream over a console link is refused, in the pane",
           "cannot stream" in window.message.cget("text") and
           window.live_button.cget("text") == "stream",
           " (%s)" % window.message.cget("text"))
    expect("and the live pane is left empty rather than showing numbers "
           "no frame carried",
           window.live.cget("text") == "")

    root.destroy()
    transport.close()

    print("configurator window: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
