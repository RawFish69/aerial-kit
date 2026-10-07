#!/usr/bin/env python3
"""The configurator's window against a board it does not own.

    make proto-test        # which runs this with the rest of the clients

`akconfig_window_check.py` drives the window against our own firmware.
This one points the same window at a Betaflight board
(`tools/msp_fake_board.py`) and checks the two things that make "compatible
with other firmwares" a fact rather than a wish:

* the window **finds it** - it asks its own protocol, is not answered, asks
  MSP, and names what answered; and
* the window **shows what it can and says what it cannot** - the state, the
  attitude, the pack and the motors from MSP's own frames, an empty parameter
  table with the reason in words, and buttons that refuse to write to somebody
  else's flight controller instead of sending bytes into it.

The board is a stand-in built from Betaflight's own layouts and constants (see
that file's docstring for which upstream file each number comes from), because
there is no Betaflight board on this machine. What that means for the evidence
is written down in `docs/27-configurator.md`: this proves the client and the
window, and the bench session - flash the F405 with Betaflight and point this
window at it - is what proves the board.
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from akconfig_window_check import ensure_display, pump              # noqa: E402

FAKE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "msp_fake_board.py")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def main():
    problem = ensure_display()
    if problem is not None:
        print("the window against a foreign board: not checked here - %s"
              % problem)
        return 0

    import tkinter
    from tkinter import ttk

    import akconfig
    import msp

    print("the configurator's window, against a board that is not ours")

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
    expect("the window's first question finds a board that is not ours",
           kind == "msp" and configurator is not None, " (%s)" % kind)

    link = akconfig.Link(configurator)
    root = tkinter.Tk()
    window = akconfig.Window(root, configurator, link)

    # The window asks for the table as it is built - which for this board is
    # empty - so "connected" is the title saying what answered.
    pump(root, lambda: window.aircraft.cget("text") != "connecting...")
    label = window.aircraft.cget("text")
    expect("and it names the firmware, its version and the board",
           "Betaflight" in label and "2026.6.1" in label and "S405" in label,
           " (%s)" % label)

    # The table is empty - MSP cannot be asked for a list - and the heading now
    # says what *can* be done and where the names came from: the window offers
    # this release's own names, read from its table in `upstream/`, because the
    # protocol has no request that lists them.
    heading = window.table_label.cget("text")
    # The name list is read from the release's own table in `upstream/`, which
    # is ignored rather than tracked (`scripts/fetch-upstreams.sh` recreates it),
    # so a checkout without it has no list - and that is a *case*, not a crash:
    # the window says which release it wanted and offers a plain entry. The
    # checks below are skipped with that line, the way the MAVLink ones skip
    # without pymavlink. (The first version of this file walked into the entry
    # with `cget("values")` and died with a TclError - found by the post-landing
    # rehearsal, whose tree is a checkout without `upstream/`.)
    have_names = "no betaflight-2026.6.1 checkout" not in heading
    if not have_names:
        print("  ----     the names: not checked here - %s" % heading)

    expect("the parameter table is empty, and the heading says what a person "
           "can do instead",
           len(window.table.get_children()) == 0 and
           (("a name at a time" in heading and "names" in heading)
            if have_names else "no name list here" in heading),
           " (%s)" % heading)
    if have_names:
        expect("and the names it offers are this release's own, from its own "
               "table",
               "upstream/betaflight-2026.6.1" in heading and "817" in heading,
               " (%s)" % heading)
        expect("and the box is a list of them rather than a blank entry",
               isinstance(window.filter, ttk.Combobox) and
               "failsafe_throttle" in list(window.filter.cget("values")) and
               len(window.filter.cget("values")) == 817,
               " (%s, %u offered)"
               % (type(window.filter).__name__,
                  len(window.filter.cget("values"))))

    status = window.status.cget("text")
    expect("the aircraft pane carries the state and the attitude from MSP's "
           "own frames",
           "state" in status and "attitude" in status and
           "roll 15" in status.replace("  ", " "),
           " (%s)" % " / ".join(status.split("\n")))
    expect("the pack is there, from the frame MSP carries it in",
           " V," in status and " mAh" in status,
           " (%s)" % status.split("\n")[3])
    expect("and the position is the one the module reported, not a zero from "
           "a field the client could not find",
           "52.1234567" in status, " (%s)" % status.split("\n")[2])
    # The other half of that rule, and the reason it is a rule: a firmware that
    # does *not* carry a field must not have it drawn as zero. Checked here
    # rather than through a board, because the board that answers everything is
    # the one this file can start.
    expect("a field the firmware does not carry is drawn as a question mark",
           akconfig._field({}, "lat", "%.7f") == "?" and
           akconfig._field({"lat": 0.0}, "lat", "%.7f") == "0.0000000",
           " (%s, %s)" % (akconfig._field({}, "lat", "%.7f"),
                          akconfig._field({"lat": 0.0}, "lat", "%.7f")))
    expect("the logs pane says each log is not on this board rather than "
           "showing counts this window did not read",
           window.logs.cget("text").count("not on this board") == 3,
           " (%s)" % window.logs.cget("text").replace("\n", " / "))

    # A *named* setting, which is the only way MSP exposes a parameter: there
    # is no list to ask for, so the filter box doubles as the question.
    window.filter.insert(0, "failsafe_throttle")
    window.fetch_one()
    pump(root, lambda: "failsafe_throttle" in window.message.cget("text"), 10.0)
    said = window.message.cget("text")
    # `pgn=8193`, which looks wrong and is right. This check said `pgn=21` until
    # 2026-10-02, and 21 is not a number any Betaflight board prints:
    # `PG_REGISTER_I` in `src/main/pg/pg.h` stores `.pgn = _pgn | (_version <<
    # 12)` and `src/main/flight/failsafe.c` registers `failsafeConfig` at
    # version 2, so `cliGetSettingInfoByName` prints `1 | (2 << 12)` = 8193.
    # The stand-in answered 21 -- its own invention -- and this check agreed
    # with it, which is the shape of a check that is testing the fixture rather
    # than the board. Correcting the stand-in to the release's own arithmetic
    # turned this red, which is what it was for.
    expect("typing a setting's name and pressing return shows what the board "
           "says about it",
           "failsafe_throttle = 1050" in said and "pgn=8193" in said and
           "max=2000" in said, " (%s)" % said.replace("\n", " / "))
    window.filter.delete(0, "end")
    window.fill_table()

    # And the same question asked the way the list invites - and with a name
    # the *release* has but this board does not, which is the one case a list
    # read from a released table cannot rule out (the table has entries a
    # particular target's build leaves out, and Betaflight's own configurator
    # has the same caveat). The answer has to be the board's own refusal, in
    # the board's own words: a window that showed nothing, or showed a value,
    # would be inventing a parameter.
    offered = list(window.filter.cget("values")) \
        if isinstance(window.filter, ttk.Combobox) else []
    if offered:
        chosen = next((name for name in offered
                       if name.startswith("gyro_lpf")), offered[0])
        window.filter.insert(0, chosen)
        window.fetch_one()
        pump(root, lambda: chosen in window.message.cget("text"), 10.0)
        refused = window.message.cget("text")
        expect("a name the release has but this board does not is refused in "
               "the board's own words rather than answered with a value",
               ("no setting called '%s'" % chosen) in refused,
               " (%s -> %s)" % (chosen, refused.replace("\n", " / ")))
        window.filter.delete(0, "end")
    else:
        print("  ----     a name off the list: not checked here - there is no "
              "list to pick one from")

    # The buttons that would write to somebody else's flight controller: two
    # of them are the difference between a viewer and a way to crash an
    # aircraft, and both refuse in the board's own name.
    window.value.insert(0, "1")
    window.set()
    pump(root, lambda: window.message.cget("text") != "")
    expect("setting a value is refused before a byte is sent, because there "
           "is no parameter to set",
           window.message.cget("text") == "pick a parameter first",
           " (%s)" % window.message.cget("text"))

    window.save()
    pump(root, lambda: window.message.cget("text").startswith("not saved"))
    expect("and saving, which needs nothing selected, is refused in the "
           "board's own name rather than reporting a number no board sent",
           window.message.cget("text").startswith("not saved") and
           "Betaflight" in window.message.cget("text"),
           " (%s)" % window.message.cget("text"))

    # And the live pane: MSP has no subscribe, so "streaming" here is polling -
    # and the pane has to fill with numbers the board really sent.
    window.rate.set("10")
    window.toggle_stream()
    pump(root, lambda: window.live.cget("text") != "", 10.0)
    live = window.live.cget("text")
    expect("asking for a stream on an MSP board polls it, and the live pane "
           "fills",
           "roll" in live and "motors" in live, " (%s)" % live.replace("\n", " / "))
    window.toggle_stream()
    # The button posts the stop to the link thread, so "it stopped" is a
    # question with a deadline rather than a fact to read in the next line -
    # which is how this check went red once under load, with the assertion
    # racing the thread it was asking about.
    pump(root, lambda: configurator.stream_hz == 0, 5.0)
    expect("and it stops when it is asked to, which for a poll is the thread "
           "going quiet", configurator.stream_hz == 0,
           " (%u Hz)" % configurator.stream_hz)

    # --- and the *other* firmware whose names are read the same way ---------
    #
    # INAV is the second variant with a name list here, and its names come out
    # of a YAML file rather than a C table - so the dispatch from what the
    # board says it is, to which release's table is read, is worth its own
    # check rather than being assumed from the Betaflight one passing.
    process.kill()
    process = subprocess.Popen([sys.executable, FAKE, "--inav"],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    transport = Pipe()
    reader = msp.Deadline(transport)
    kind, configurator = akconfig.detect(reader.read_some, transport.write)
    expect("an INAV board is found the same way",
           kind == "msp" and configurator is not None, " (%s)" % kind)
    root2 = tkinter.Tk()
    window2 = akconfig.Window(root2, configurator, akconfig.Link(configurator))
    pump(root2, lambda: window2.aircraft.cget("text") != "connecting...")
    expect("and it is named as INAV, with the version it reports",
           "INAV" in window2.aircraft.cget("text") and
           "9.1.0" in window2.aircraft.cget("text"),
           " (%s)" % window2.aircraft.cget("text"))
    # The same guard as above, for the same reason: INAV's names come from the
    # other ignored checkout.
    inav_names = list(window2.filter.cget("values")) \
        if isinstance(window2.filter, ttk.Combobox) else []
    if inav_names:
        expect("and the box offers INAV's own names, from INAV's own table",
               "upstream/inav-9.1.0" in window2.table_label.cget("text") and
               "nav_rth_altitude" in inav_names and len(inav_names) == 732,
               " (%u offered, %s)"
               % (len(inav_names), window2.table_label.cget("text")))
        window2.filter.insert(0, "nav_rth_altitude")
        window2.fetch_one()
        pump(root2, lambda: "nav_rth_altitude" in window2.message.cget("text"),
             10.0)
        # This check used to assert `"no setting called" in text`, and passed.
        # It was passing for the wrong reason twice over: the window sent
        # *Betaflight's* `MSP2_CLI_SETTING` (0x3010) to an INAV board, the
        # stand-in answered it in INAV mode as though the two firmwares shared
        # a settings protocol, and the sentence it matched on was the
        # *window's* wrapper around whatever came back. `nav_rth_altitude` is a
        # name INAV has, so "no setting called" was never the right answer.
        # What is true is that this window does not read INAV's settings
        # protocol at all, and the thing to check is that it says so itself,
        # in its own voice, instead of asking an INAV board a Betaflight
        # question.
        expect("and a name the INAV box offers is refused in the window's own "
               "voice rather than by asking an INAV board Betaflight's "
               "question",
               window2.message.cget("text").startswith("not read:") and
               "0x3010" not in window2.message.cget("text") and
               "parameter groups" in window2.message.cget("text"),
               " (%s)" % window2.message.cget("text").replace("\n", " / "))
        # And the box says so before anything is typed into it, because a box
        # that invites a person to press return has to say when return cannot
        # work.
        expect("and the box above it says the same thing rather than inviting "
               "a press it cannot answer",
               "Betaflight board only" in window2.table_label.cget("text"),
               " (%s)" % window2.table_label.cget("text"))
    else:
        print("  ----     the INAV names: not checked here - %s"
              % window2.table_label.cget("text"))
    root2.destroy()
    transport.close()
    process.kill()

    root.destroy()
    transport.close()

    print("configurator window, foreign board: %s"
          % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
