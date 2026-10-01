#!/usr/bin/env python3
"""AerialKit's configurator: a window onto the parameter table and the aircraft.

    python3 tools/akconfig.py --sim                 # the built simulator
    python3 tools/akconfig.py --port /dev/ttyACM0   # a board's USB console
    python3 tools/akconfig.py --host 10.0.2.15:5555 # the ESP32's network

This is the client the parameter table and the config protocol were built for
(docs/16-protocol.md): it lists every parameter by name and index, sets one
through the same range check the console uses - and shows the board's own words
when it refuses, which is what the `param set` reply carries it for - saves the
table to the board, and shows what the aircraft is doing while it is connected:
state and attitude, the fix, the motor outputs, the three logs' counts, and a
telemetry stream at a rate a person chooses.

**Where it runs, and where it does not.** Anywhere with Python 3 and Tk: the
laptop that builds the firmware, or the Pi beside the aircraft. It is a *tool*,
not a service - nothing starts it, nothing listens for it, and it is deliberately
not on the NAS's web console: flight control out of the stack that serves files
is a decision this project has already made and reversed once
(docs/06-console.md, docs/01-plan.md M7).

**What it cannot do.** It cannot flash: that is `dfu` on the console and
`dfu-util`, because writing flash from a window is the one operation that can
leave an aircraft unable to talk about it. It cannot arm or move an output
either - the protocol has no such command, and the console's own rule is that
nothing arms an aircraft but a receiver.

The half that talks to the aircraft is `Configurator`, which has no Tk in it and
is driven headless by `tools/akconfig_check.py`; the window is a view of that.
"""

import argparse
import os
import queue
import struct
import sys
import threading
import time

from akproto import (Client, HELLO, LOG_FIELDS, LOG_INFO, LOG_SOURCES,
                     PARAM_GET, PARAM_SAVE, PARAM_SET, STATUS, TELEMETRY_MAX_HZ,
                     c_string, fetch_log, parse_status, parse_telemetry,
                     next_telemetry, select_log, set_reply, subscribe)


# --- the transport ---------------------------------------------------------
#
# Three ways to reach a firmware, and they are the same three the other tools
# have: a pipe to the simulator, a socket to the ESP32, and a serial port to a
# board's console. A pipe is what the checks use; a socket needs nothing; the
# serial port is opened with the standard library so that a laptop with a USB
# cable needs no package installed.

class Pipe:
    """A child process on its stdin and stdout - the simulator, usually."""

    def __init__(self, argv):
        import subprocess
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE)

    def read(self, count):
        return self.process.stdout.read(count)

    def write(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def close(self):
        self.process.kill()


class Socket:
    def __init__(self, spec):
        import socket
        host, _, port = spec.rpartition(":")
        self.connection = socket.create_connection((host or "127.0.0.1",
                                                    int(port)), 5)
        self.connection.settimeout(5)

    def read(self, count):
        return self.connection.recv(count)

    def write(self, data):
        self.connection.sendall(data)

    def close(self):
        self.connection.close()


class Serial:
    """A TTY at 115200 8N1, raw, in `os` and `termios` rather than pyserial.

    The protocol shares the console's UART, so this is the same door the console
    is on - which means a board that is printing a banner has to be quiet before
    a frame goes out. The firmware's console says nothing unless something is
    typed, so in practice the port is idle.
    """

    def __init__(self, path):
        import termios
        import tty
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        settings = termios.tcgetattr(self.fd)
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, settings)

    def read(self, count):
        while True:
            try:
                data = os.read(self.fd, count)
            except BlockingIOError:
                data = b""
            if data:
                return data
            time.sleep(0.002)

    def write(self, data):
        os.write(self.fd, data)

    def close(self):
        os.close(self.fd)


# --- the half with no window in it ----------------------------------------

class Configurator:
    """The parameter table and the aircraft's numbers, over one connection.

    Every method here is a synchronous request-and-reply, which is what makes it
    testable without a window: the window runs it on a thread and posts the
    results to the event loop, and the checks call it directly.
    """

    def __init__(self, client):
        self.client = client
        self.product = ""
        self.protocol = 0
        self.count = 0
        self.changed = 0
        self.items = []          # [(index, name, value-as-text)]
        self.originals = {}      # index -> the text as it was when last read
        self.stream_hz = 0

    def connect(self):
        """Hello, and remember what the aircraft says it is."""
        reply = self.client.request(HELLO)
        self.protocol = reply[0]
        self.product = c_string(reply, 1)
        at = 2 + len(self.product)
        self.count = struct.unpack_from("<H", reply, at)[0]
        self.changed = int(c_string(reply, at + 2))
        return self

    def parameters(self):
        """The whole table, one parameter at a time.

        There is no "send me the table" command on purpose: the protocol's
        parameter calls are the console's, and the console's `params` walks the
        table the same way. Eighty-one round trips is about a third of a second
        on a 115200 UART and nothing on a socket, which is why this is allowed
        to be the simple thing.
        """
        self.items = []
        for index in range(self.count):
            payload = self.client.request(PARAM_GET, bytes([index]))
            if payload[0] != 0:
                continue
            name = c_string(payload, 1)
            value = c_string(payload, 2 + len(name))
            self.items.append((index, name, value))
            self.originals.setdefault(index, value)
        return self.items

    def set(self, index, text):
        """Set one parameter. Returns `(status, message)` - the message is the
        table's own, which is the point of the reply growing a string."""
        return set_reply(self.client.request(PARAM_SET,
                                             bytes([index]) + text.encode()))

    def revert(self):
        """Put back the values this session first read.

        It is a *client-side* revert made of real writes, because a `set`
        applies to the running aircraft immediately and the protocol has no
        "undo": the only other way back to the saved record is a reboot, and a
        window that offers "revert" should not be offering a reboot. What it
        does not do is put back a value somebody changed from the console while
        this window was open - it puts back what this window read.
        """
        restored = 0
        for index, _name, value in list(self.items):
            if self.originals.get(index) not in (None, value):
                status, _ = self.set(index, self.originals[index])
                if status == 0:
                    restored += 1
        # Read the table back: a revert is a set of writes like any other, and
        # the caller - the window - shows what this holds. Without this the
        # table went on showing the values that had just been undone, which is
        # the one thing a revert must not do. The window check caught it.
        if restored:
            self.parameters()
        return restored

    def save(self):
        """Write the table to the board, and treat what is now on the board as
        what a revert goes back to - the alternative is a window whose "revert"
        undoes a save."""
        status = self.client.request(PARAM_SAVE)[0]
        if status == 0:
            self.parameters()
            self.originals = {index: value for index, _name, value in self.items}
        return status

    def status(self):
        return parse_status(self.client.request(STATUS))

    def log_counts(self):
        """How many records each log holds, and `None` for one the aircraft does
        not have - which is a different answer from zero."""
        return {name: select_log(self.client, source)
                for name, source in LOG_SOURCES.items()}

    def pull_log(self, source):
        """Every record of one log, as rows of numbers - or `None` when the
        aircraft does not have that log at all, which is the same distinction
        `log_counts` makes.

        It reads through the same `fetch_log()` the command-line tool uses, so
        a log is a log whichever way it left the aircraft: the CSV header, the
        checksum-skipping and the record layout are that function's business,
        not this one's. What comes back is what the window draws.
        """
        text = []
        try:
            fetch_log(self.client, text.append, source)
        except IOError:
            return None

        rows = []
        for line in "".join(text).splitlines():
            if not line or line.startswith("#"):
                continue
            if line.split(",") == LOG_FIELDS:
                continue  # the CSV's own header row, which names the columns
            rows.append([float(value) for value in line.split(",")])
        return rows

    def start_stream(self, hz):
        """Ask for a stream and remember the rate that will actually be sent -
        which is 0 on a link that cannot push, and the window says so rather
        than waiting for frames that are never coming."""
        self.stream_hz = subscribe(self.client, min(hz, TELEMETRY_MAX_HZ))
        return self.stream_hz

    def stop_stream(self):
        self.stream_hz = subscribe(self.client, 0)
        return self.stream_hz

    def next_telemetry(self):
        """Blocks until a telemetry frame arrives - the window's thread calls
        this in a loop while a stream is running."""
        return next_telemetry(self.client)

    def describe(self):
        """One line for the window's title bar, and the same words whichever
        firmware answered - see `MspConfigurator.describe()`."""
        return ("AerialKit %s, protocol %u, %u parameters"
                % (self.product, self.protocol, self.count))

    def parameters_note(self):
        """What to say when the table has no rows. A board of ours that
        reports none is a board with an empty table; a foreign board is a
        different sentence, and the window shows whichever it is."""
        return ("this board reports no parameters" if self.count == 0 else "")


class MspConfigurator:
    """A board that is not ours, read through MSP - read-only, on purpose.

    Betaflight and INAV speak this, which is four of the five firmwares the
    owner named (`docs/27-configurator.md`), and the missing half is the
    interesting part: this class *reads* and never writes. Parameter writes to
    somebody else's firmware are how a tool crashes an aircraft, so the panes
    that would write say what they cannot do instead.

    It answers the same questions `Configurator` does - `status()`,
    `log_counts()`, `next_telemetry()` - so the window shows an MSP board
    through the widgets it already has rather than growing a second window.
    What MSP does not carry is `None`, never a zero: "this firmware does not
    say" and "the value is zero" are different answers and the window prints
    them differently.
    """

    def __init__(self, msp):
        self.msp = msp
        self.identity = None
        self.product = ""
        self.protocol = 0
        self.count = 0            # parameters this window can read: none, yet
        self.changed = 0
        self.items = []
        self.originals = {}
        self.stream_hz = 0

    def connect(self):
        from msp import describe, identify
        self.identity = identify(self.msp)
        self.product = describe(self.identity)
        self.protocol = self.identity["api"]["protocol"]
        # The two facts the name list is *for*: which firmware this is (the
        # variant string) and which release of it (the version string). Both
        # come from the board, because the names a board answers to are the
        # names its own release has.
        self.variant = (self.identity.get("variant") or "").strip()
        self.version = (self.identity.get("version") or {}).get("text", "").strip()
        # The variant's own parameter registry is what a *writer* would need,
        # and this class does not read it: `count` is the number of parameters
        # this window can show, which is none of Betaflight's two hundred.
        self.count = 0
        return self

    def setting_names(self):
        """`(names, source)` - the names this firmware *release* has, and where
        they were read from, or `([], reason)`.

        The one thing this class can offer that the protocol cannot: MSP has no
        "list the settings" request (see `docs/27-configurator.md`), so the
        window's filter box is where a person types a name - and without a list
        they have to know it by heart. Betaflight's own configurator carries a
        list per release for exactly this reason. Ours is read from the pinned
        checkout of the release the board says it is.

        The second element is the *reason* when there is no list: a version
        this workspace has no checkout of is a different sentence from a
        firmware with no reader here, and the window shows whichever it is.
        """
        import msp_settings
        firmware = msp_settings.firmware_for(self.variant)
        if firmware is None:
            return [], ("%s has no name list here: this window reads names for "
                        "Betaflight and INAV, whose own configurators carry "
                        "one (see msp_settings.py)" % (self.variant or "this "
                                                       "variant"))
        found, source = msp_settings.names(firmware, self.version)
        if found:
            return found, source
        return [], source

    def parameters(self):
        """Nothing, and the window says why: MSP's parameter model is the
        firmware's own registry (`MSP_SETTING_INFO`/`MSP_SETTING` for
        Betaflight, its parameter groups for INAV), which is the piece of work
        `docs/27-configurator.md` records as not done."""
        self.items = []
        return self.items

    def status(self):
        from msp import state
        # `alt_m` and the GPS fields are in the firmware's own status dict
        # shape; the ones this protocol does not carry arrive as None and the
        # window prints them as unknown rather than as zero.
        reading = state(self.msp)
        reading.setdefault("alt_m", None)
        return reading

    def log_counts(self):
        """MSP has logs of its own and none of them is a log this window can
        read yet; the pane says "not on this board" for each, which is the same
        answer it gives for a log ours does not have."""
        return {}

    def pull_log(self, source):
        return None

    def set(self, index, text):
        from msp import short_name
        return (1, "this is a %s board: this window reads it and does not "
                   "write to it (docs/27-configurator.md)"
                   % short_name(self.identity))

    def revert(self):
        return 0

    def save(self):
        from msp import short_name
        return ("not saved: this window reads a %s board and does not write to "
                "it (docs/27-configurator.md)" % short_name(self.identity))

    def start_stream(self, hz):
        """MSP has no subscribe: a client polls. The window's thread calls
        `next_telemetry()` in a loop, so "streaming" here is "polling", and the
        rate is what the window asked for rather than a rate the board agreed
        to."""
        self.stream_hz = max(1, min(int(hz), 50))
        return self.stream_hz

    def stop_stream(self):
        self.stream_hz = 0
        return 0

    def next_telemetry(self):
        return self.status()

    def describe(self):
        return self.product

    def parameters_note(self):
        names, source = self.setting_names()
        if names:
            return ("a name at a time: MSP cannot be asked for a list, so the "
                    "box offers the %u names read from %s - pick one and press "
                    "return for the board's own answer"
                    % (len(names), source))
        return ("this board's parameters live in its own registry (MSP's "
                "settings, or INAV's parameter groups), which this window does "
                "not read yet - and there is no name list here either: %s "
                "(docs/27-configurator.md)" % source)

    def setting(self, name):
        """One setting by name, in the board's own words.

        The only way MSP exposes a parameter: there is no list to ask for (see
        `docs/27-configurator.md`), so the window's filter box asks about the
        one a person names, and the answer is the board's sentence - "name =
        value" - followed by what the board says the setting *is*.
        """
        from msp import Error, ProtocolError
        detail = ""
        try:
            detail = self.msp.setting_info(name)
        except (Error, ProtocolError):
            detail = ""
        try:
            value = self.msp.setting(name)
        except (Error, ProtocolError) as problem:
            # A name list read from a release's own table cannot know what a
            # particular target's build left out - Betaflight's table wraps
            # some entries in `#if defined(USE_...)` - so "this release has
            # that name and this board does not" is an ordinary answer. It is
            # the board's answer, and it is quoted with the name that was asked
            # for rather than as the command id the refusal came back under.
            raise Error("no setting called '%s' on this board (%s)"
                        % (name, problem))
        return "%s\n%s" % (value, detail) if detail else value


# --- which firmware is on the other end ------------------------------------

class WithTimeout:
    """A reader with a deadline, for detection and for everything after it.

    `b""` means nothing arrived in time, which is what `akproto`'s client
    already reads as "the other end stopped talking" - so a board that has gone
    quiet raises there instead of hanging a window. That is the same shape the
    MSP client wants, so both protocols sit on this one reader and never fight
    over the stream.
    """

    def __init__(self, read_some, timeout):
        self.read_some = read_some
        self.timeout = timeout

    def read(self, count):
        return self.read_some(count, self.timeout)


class MavlinkConfigurator:
    """An ArduPilot or PX4 vehicle, read through MAVLink - read-only, on purpose.

    The third of the window's three protocols (`docs/27-configurator.md`), and
    the one that gives a foreign board a *parameter table*: MAVLink's parameter
    protocol is standardized, so `PARAM_REQUEST_LIST` makes a vehicle send
    every parameter by name, value and type, and the window's own table shows
    them. Writes are refused in words, like the MSP side, and for the same
    reason - `PARAM_SET` against somebody else's flight controller is how a
    tool crashes an aircraft.

    What MAVLink does not have is anything this window already calls a log: a
    vehicle's logs are a file-transfer protocol of its own (MAVLink FTP, or the
    ArduPilot log download a ground station speaks), so the logs pane says
    "not on this board" for all three, honestly.
    """

    def __init__(self, mav):
        self.mav = mav
        self.identity = None
        self.product = ""
        self.protocol = 0
        self.count = 0
        self.changed = 0
        self.items = []
        self.originals = {}
        self.stream_hz = 0

    def connect(self):
        from mavlink import describe
        self.identity = self.mav.identify(3.0)
        self.product = describe(self.identity)
        # Every message the window shows, asked for once: a vehicle streams
        # what its ground station asked for, and nothing else.
        for name in ("ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT",
                     "SYS_STATUS", "VFR_HUD", "RC_CHANNELS",
                     "SERVO_OUTPUT_RAW"):
            try:
                self.mav.request(name, 5.0)
            except Exception:                            # noqa: BLE001
                pass
        return self

    def parameters(self):
        """The vehicle's own parameter list, read the standard way."""
        listing = self.mav.parameters(timeout=20.0)
        self.items = [(index, name, "%.4f" % value)
                      for index, (name, value, _kind) in enumerate(listing["items"])]
        self.count = listing["count"] if listing["count"] is not None \
            else len(self.items)
        for index, _name, value in self.items:
            self.originals.setdefault(index, value)
        return self.items

    def status(self):
        from mavlink import state
        return state(self.mav)

    def log_counts(self):
        return {}

    def pull_log(self, source):
        return None

    def set(self, index, text):
        from mavlink import describe
        return (1, "this is %s: this window reads it and does not write to it "
                   "(docs/27-configurator.md)" % describe(self.identity))

    def revert(self):
        return 0

    def save(self):
        return ("not saved: this window reads a MAVLink vehicle and does not "
                "write to it (docs/27-configurator.md)")

    def start_stream(self, hz):
        """MAVLink *does* stream - a vehicle pushes what it was asked for - so
        this asks the vehicle for a faster rate and polls its own reader on the
        window's thread."""
        from mavlink import Timeout
        rate = max(1, min(int(hz), 50))
        for name in ("ATTITUDE", "GLOBAL_POSITION_INT", "SYS_STATUS"):
            try:
                self.mav.request(name, rate)
            except Timeout:
                pass
        self.stream_hz = rate
        return self.stream_hz

    def stop_stream(self):
        self.stream_hz = 0
        return 0

    def next_telemetry(self):
        self.mav.wait("ATTITUDE", 2.0)
        return self.status()

    def describe(self):
        return self.product

    def parameters_note(self):
        return ("this vehicle's parameters are read through MAVLink; writing "
                "them is refused (docs/27-configurator.md)")


def detect(read_some, write, timeout=0.6):
    """Which firmware is on the other end of this stream?

        ("aerialkit", Configurator)   ("msp", MspConfigurator)   ("unknown", None)

    **Ours first**, because a hello is the only frame a board can be asked for
    before anything is known about it - and because the probe for the next
    protocol down sends bytes our console would have to swallow. Then MSP,
    which is what Betaflight and INAV answer.

    ArduPilot and PX4 are MAVLink: a third protocol, a third file's worth of
    work, and the reason this returns "unknown" rather than guessing. A board
    that answers neither is not a failure of this function - it is a fact about
    the board, and the caller says it in words.
    """
    ours = Configurator(Client(WithTimeout(read_some, timeout).read, write))
    try:
        ours.connect()
        return "aerialkit", ours
    except Exception:                                    # noqa: BLE001
        # Anything at all: detection's whole job is "if this is not ours, move
        # on", and a foreign board is allowed to answer with anything.
        pass

    from msp import Msp, Timeout as MspTimeout

    client = Msp(read_some, write, timeout=1.0)
    try:
        return "msp", MspConfigurator(client).connect()
    except (MspTimeout, IOError, OSError):
        pass

    # And then MAVLink, which is the one protocol that does not have to be
    # asked: a vehicle announces itself with a heartbeat, and it was almost
    # certainly announcing itself while the two questions above were being
    # asked. A ground station sends its own heartbeat first, because a vehicle
    # only streams to a system it has heard from.
    from mavlink import Mavlink, Timeout as MavlinkTimeout

    mav = Mavlink(read_some, write, timeout=1.0)
    try:
        mav.send_heartbeat()
        return "mavlink", MavlinkConfigurator(mav).connect()
    except (MavlinkTimeout, IOError, OSError, ValueError):
        return "unknown", None


# --- the window ------------------------------------------------------------

class Link:
    """One thread owns the connection; the window owns the widgets.

    A Tk callback that blocks on a read freezes the window for as long as the
    other end takes to answer, and a socket that has gone quiet takes five
    seconds to say so - so every request goes onto a queue and every answer
    comes back on one. The telemetry stream is the same thread, reading frames
    rather than waiting for replies, because a stream *is* frames arriving
    unasked.
    """

    def __init__(self, configurator):
        self.configurator = configurator
        self.requests = queue.Queue()
        self.replies = queue.Queue()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def post(self, fn, token=None):
        self.requests.put((fn, token))

    def _run(self):
        while not self.stop.is_set():
            try:
                fn, token = self.requests.get(timeout=0.05)
            except queue.Empty:
                if self.configurator.stream_hz:
                    self._stream_once()
                continue
            try:
                self.replies.put((token, fn(), None))
            except Exception as problem:            # noqa: BLE001 - shown, not raised
                self.replies.put((token, None, problem))

    def _stream_once(self):
        try:
            frame = self.configurator.next_telemetry()
            self.replies.put(("telemetry", frame, None))
        except Exception as problem:                # noqa: BLE001
            self.replies.put(("telemetry", None, problem))
            self.stop.set()


class Window:
    """The configurator's face: the table on the left, the aircraft on the right."""

    def __init__(self, root, configurator, link):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.root = root
        self.configurator = configurator
        self.link = link
        self.items = []

        root.title("AerialKit configurator")
        root.geometry("980x560")

        bar = ttk.Frame(root)
        bar.pack(fill="x", padx=6, pady=4)
        self.aircraft = ttk.Label(bar, text="connecting...")
        self.aircraft.pack(side="left")
        self.changed = ttk.Label(bar, text="")
        self.changed.pack(side="right")

        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=6)

        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(body)
        right.pack(side="right", fill="both", expand=False, padx=(8, 0))

        self.table_label = ttk.Label(left, text="parameters")
        self.table_label.pack(anchor="w")
        # The box is a list when this board's firmware has one here. MSP cannot
        # be asked for its settings' names - there is no request for that (see
        # `docs/27-configurator.md`) - so typing one from memory is all the
        # protocol allows, unless the window knows the release's own list,
        # which for Betaflight and INAV it does (tools/msp_settings.py). A
        # combobox is an entry with a list attached, so filtering and the
        # by-name ask behave the same on a board whose names are known and on
        # one whose are not; only the offering differs.
        offered = []
        if hasattr(configurator, "setting_names"):
            offered = configurator.setting_names()[0]
        self.filter = (ttk.Combobox(left, values=offered) if offered
                       else ttk.Entry(left))
        self.filter.pack(fill="x")
        self.filter.bind("<KeyRelease>", lambda _event: self.fill_table())
        # And return, for the firmwares whose parameters are read *by name*
        # rather than listed - see `fetch_one()`.
        self.filter.bind("<Return>", lambda _event: self.fetch_one())

        self.table = ttk.Treeview(left, columns=("index", "name", "value"),
                                  show="headings", height=18)
        for column, width in (("index", 50), ("name", 220), ("value", 160)):
            self.table.heading(column, text=column)
            self.table.column(column, width=width, anchor="w")
        self.table.pack(fill="both", expand=True)
        self.table.bind("<<TreeviewSelect>>", lambda _event: self.pick())

        edit = ttk.Frame(left)
        edit.pack(fill="x", pady=4)
        self.value = ttk.Entry(edit, width=24)
        self.value.pack(side="left")
        self.value.bind("<Return>", lambda _event: self.set())
        for label, fn in (("set", self.set), ("revert", self.revert),
                          ("refresh", self.refresh), ("save", self.save)):
            ttk.Button(edit, text=label, command=fn).pack(side="left", padx=2)
        self.message = ttk.Label(left, text="", foreground="#444")
        self.message.pack(anchor="w")

        ttk.Label(right, text="the aircraft").pack(anchor="w")
        self.status = ttk.Label(right, text="", justify="left", width=44)
        self.status.pack(anchor="w")
        self.logs = ttk.Label(right, text="", justify="left", width=44)
        self.logs.pack(anchor="w", pady=(6, 0))

        live = ttk.Frame(right)
        live.pack(anchor="w", pady=(8, 0))
        self.rate = ttk.Combobox(live, width=4, values=("5", "10", "20", "50"))
        self.rate.set("10")
        self.rate.pack(side="left")
        ttk.Label(live, text="Hz").pack(side="left")
        self.live_button = ttk.Button(live, text="stream", command=self.toggle_stream)
        self.live_button.pack(side="left", padx=4)
        self.live = ttk.Label(right, text="", justify="left", width=44)
        self.live.pack(anchor="w", pady=(6, 0))

        # The logs: a count above, and below them the one thing a log is for -
        # a plot of what the aircraft did. `pull` reads every record over the
        # same protocol the command-line tool uses and draws the attitude
        # traces; the count comes back with it, so a size on screen and a trace
        # on screen cannot disagree.
        pull = ttk.Frame(right)
        pull.pack(anchor="w", pady=(8, 0))
        self.log_source = ttk.Combobox(pull, width=5,
                                       values=tuple(LOG_SOURCES.keys()))
        self.log_source.set("flash")
        self.log_source.pack(side="left")
        self.pull_button = ttk.Button(pull, text="pull log", command=self.pull)
        self.pull_button.pack(side="left", padx=4)
        self.log_note = ttk.Label(right, text="no log pulled", width=44,
                                  justify="left")
        self.log_note.pack(anchor="w")
        self.plot = tk.Canvas(right, width=360, height=120,
                              background="#101418", highlightthickness=0)
        self.plot.pack(anchor="w", pady=(2, 0))

        self.link.start()
        self.root.after(50, self.drain)
        self.connect()

    # -- plumbing between the thread and the widgets ------------------------

    def connect(self):
        def hello():
            # `self.configurator`, not the `configurator` that `main()` happens
            # to hold: this closure runs on the link's thread, where that local
            # is not in scope at all - which is what the window check found the
            # first time anybody ran the window.
            self.configurator.connect()
            self.configurator.parameters()
            return (self.configurator.status(), self.configurator.log_counts())

        self.link.post(hello, token="connect")
        self.message.configure(text="connecting...")

    def drain(self):
        try:
            while True:
                token, result, problem = self.link.replies.get_nowait()
                self.deliver(token, result, problem)
        except queue.Empty:
            pass
        self.root.after(50, self.drain)

    def deliver(self, token, result, problem):
        if problem is not None:
            self.message.configure(text="%s" % problem)
            return
        if token == "connect":
            status, logs = result
            # `describe()` rather than a format string here: two firmwares can
            # answer this window now, and how each of them wants to be named is
            # its own business.
            self.aircraft.configure(text=self.configurator.describe())
            self.items = list(self.configurator.items)
            self.fill_table()
            self.fill_status(status)
            self.fill_logs(logs)
        elif token == "set-ack" and isinstance(result, tuple):
            status, message = result
            self.items = list(self.configurator.items)
            self.fill_table()
            if status == 0:
                self.message.configure(text="set")
            else:
                self.message.configure(text="refused: %s" % (message or "no reason given"))
        elif token == "set-ack":
            self.items = list(self.configurator.items)
            self.fill_table()
            self.message.configure(text="reverted %s value(s)" % result)
        elif token == "save-ack":
            if isinstance(result, str):
                # A backend that cannot write says so in its own words; ints
                # are the status byte of a board that can.
                self.message.configure(text=result)
            else:
                self.message.configure(text="saved" if result == 0
                                       else "the board refused the write (%s)"
                                            % result)
        elif token == "refresh":
            self.items = list(self.configurator.items)
            self.fill_table()
            self.fill_status(result[0])
            self.fill_logs(result[1])
        elif token == "setting":
            # The board's own answer about one named setting - the MSP
            # firmwares' only door onto their parameters. See `fetch_one()`.
            self.message.configure(text=result or "no answer")
        elif token == "status":
            self.fill_status(result)
        elif token == "telemetry":
            self.fill_live(result)
        elif token == "stream":
            if result:
                self.message.configure(text="streaming at %u Hz" % result)
            else:
                self.live_button.configure(text="stream")
                self.message.configure(
                    text="this link cannot stream: a console is a wire somebody "
                         "types at, and the network link (an ESP32's port) is "
                         "where a stream lives")
        elif token == "stream-stopped":
            self.message.configure(text="stream stopped")
        elif token == "log":
            self.draw_log(result[0], result[1])
        self.changed.configure(
            text="%u changed since the last save" % self.configurator.changed)

    def fill_table(self):
        wanted = self.filter.get().lower()
        # The row a person is editing, kept across a refill: setting a value
        # reads the table back and refills it, and a window that forgot which
        # parameter it was on would answer the next "set" with "pick a
        # parameter first".
        selected = self.table.selection()
        self.table.delete(*self.table.get_children())
        for index, name, value in self.items:
            if wanted and wanted not in name.lower():
                continue
            self.table.insert("", "end", iid=str(index),
                              values=(index, name, value))
        keep = [iid for iid in selected if self.table.exists(iid)]
        if keep:
            self.table.selection_set(keep)
        # An empty table is two different things - a board with nothing to
        # configure, and a board whose parameters this window does not read -
        # and the heading says which one it is looking at.
        note = self.configurator.parameters_note() if not self.items else ""
        self.table_label.configure(text="parameters" if not note
                                   else "parameters (%s)" % note)

    def fill_status(self, status):
        self.status.configure(text=(
            "state      %s   link %s   fix %s (%s sats)\n"
            "attitude   roll %s  pitch %s  yaw %s\n"
            "position   %s, %s\n"
            "power      %s V, %s mAh\n"
            "motors     %s"
            % (_field(status, "flight_state", "%u"),
               _field(status, "link_live", "%u"),
               _field(status, "gps_fix_type", "%u"),
               _field(status, "gps_satellites", "%u"),
               _field(status, "roll_deg", "%.1f"),
               _field(status, "pitch_deg", "%.1f"),
               _field(status, "yaw_deg", "%.1f"),
               _field(status, "lat", "%.7f"),
               _field(status, "lon", "%.7f"),
               _field(status, "vbat_v", "%.1f"),
               _field(status, "mah", "%u"),
               ",".join(str(m) for m in status.get("motors") or []) or "?")))

    def fill_logs(self, logs):
        def show(name):
            count = logs.get(name)
            return "not on this board" if count is None else "%u records" % count

        self.logs.configure(text="logs       fast %s\n           long %s\n"
                                 "           flash %s"
                                 % (show("fast"), show("long"), show("flash")))

    def fill_live(self, frame):
        self.live.configure(text=(
            "live       roll %s  pitch %s  yaw %s\n"
            "           motors %s"
            % (_field(frame, "roll_deg", "%.1f"),
               _field(frame, "pitch_deg", "%.1f"),
               _field(frame, "yaw_deg", "%.1f"),
               ",".join(str(m) for m in frame.get("motors") or []) or "?")))

    # -- the buttons --------------------------------------------------------

    def fetch_one(self):
        """Ask the board about the name in the filter box.

        Two of the three firmwares answer a setting *by name* and have no way
        to list them: MSP's `MSP2_CLI_SETTING` takes a name, and the reference
        configurator ships the list of names itself. So the filter box doubles
        as the way to ask about one - type a name, press return, and the
        board's own sentence is shown - while the table stays empty with the
        reason in its heading. A board that does not answer named settings
        (ours, and a MAVLink vehicle, whose parameters are all in the table
        already) does nothing here.
        """
        name = self.filter.get().strip()
        if not name or not hasattr(self.configurator, "setting"):
            return
        self.link.post(lambda: self.configurator.setting(name), token="setting")

    def pick(self):
        selection = self.table.selection()
        if selection:
            index = int(selection[0])
            for item_index, _name, value in self.items:
                if item_index == index:
                    self.value.delete(0, "end")
                    self.value.insert(0, value)

    def set(self):
        selection = self.table.selection()
        if not selection:
            self.message.configure(text="pick a parameter first")
            return
        index = int(selection[0])
        text = self.value.get()

        def do_set():
            status, message = self.configurator.set(index, text)
            # Read it back either way: a refused set leaves the old value, and
            # the table should show what the aircraft actually has.
            self.configurator.parameters()
            return (status, message)

        self.link.post(do_set, token="set-ack")

    def revert(self):
        self.link.post(self.configurator.revert, token="set-ack")

    def save(self):
        self.link.post(self.configurator.save, token="save-ack")

    def refresh(self):
        def reload():
            self.configurator.parameters()
            return (self.configurator.status(), self.configurator.log_counts())

        self.link.post(reload, token="refresh")

    def pull(self):
        """Read one log off the aircraft and draw it. On the link's thread, like
        every other protocol call here: a flash log is a few hundred round
        trips, and doing that on the widgets' thread would freeze the window
        for exactly as long as the pull takes."""
        source = LOG_SOURCES.get(self.log_source.get(), LOG_SOURCES["flash"])
        # Both read *here*, on the thread Tk owns. The closure below runs on
        # the link's, and a Tk call from there raises "main thread is not in
        # main loop" - which is how this line was written the first time, and
        # how the window check caught it a second time.
        name = self.log_source.get()
        self.log_note.configure(text="pulling %s..." % name)

        def read():
            return (self.configurator.pull_log(source), name)

        self.link.post(read, token="log")

    def draw_log(self, rows, name):
        """The attitude traces, against time, in the pane's own units.

        Roll, pitch and yaw are what a bench or a first flight is read for, and
        they share a unit (degrees, and the record carries tenths of one), so
        one scale is honest for all three. A log with fewer than two records
        has no trace to draw and says so rather than dividing by zero.
        """
        self.plot.delete("all")
        if rows is None:
            self.log_note.configure(
                text="no %s log on this aircraft" % name)
            return
        if len(rows) < 2:
            self.log_note.configure(
                text="%s log: %u record%s - too short to draw"
                     % (name, len(rows), "" if len(rows) == 1 else "s"))
            return

        width = int(self.plot.cget("width"))
        height = int(self.plot.cget("height"))
        margin = 12
        first = rows[0][0]
        last = rows[-1][0]
        span_ms = max(1.0, last - first)
        traces = (("roll", LOG_FIELDS.index("roll"), "#e06c75"),
                  ("pitch", LOG_FIELDS.index("pitch"), "#98c379"),
                  ("yaw", LOG_FIELDS.index("yaw"), "#61afef"))

        values = [row[index] / 10.0 for row in rows for _n, index, _c in traces]
        low = min(values)
        high = max(values)
        if high - low < 1.0:  # a level aircraft is still worth a scale
            middle = (high + low) / 2.0
            low = middle - 0.5
            high = middle + 0.5

        def x_at(row):
            return margin + (width - 2 * margin) * (row[0] - first) / span_ms

        def y_at(row, index):
            return (height - margin) - (height - 2 * margin) * \
                ((row[index] / 10.0 - low) / (high - low))

        self.plot.create_line(margin, height - margin, width - margin,
                              height - margin, fill="#3b4048")
        for label, index, colour in traces:
            points = []
            for row in rows:
                points.extend((x_at(row), y_at(row, index)))
            self.plot.create_line(*points, fill=colour, width=2)
        self.plot.create_text(margin, 4, anchor="nw", fill="#9aa0a6",
                              text="%s: roll/pitch/yaw, %.1f..%.1f deg over "
                                   "%.1f s" % (name, low, high, span_ms / 1000.0))
        self.log_note.configure(
            text="%s log: %u records, roll/pitch/yaw drawn"
                 % (name, len(rows)))

    def toggle_stream(self):
        if self.configurator.stream_hz:
            self.link.post(self.stop_stream, token="stream-stopped")
            self.live_button.configure(text="stream")
        else:
            # Read the widget *here*, on the thread Tk owns: the call below runs
            # on the link's, and a Tk call from there raises "main thread is not
            # in main loop" - which is what this did until the window check
            # asked it for a stream.
            hz = int(self.rate.get())
            self.link.post(lambda: self.configurator.start_stream(hz),
                           token="stream")
            self.live_button.configure(text="stop")

    def stop_stream(self):
        """On the link thread, like every other protocol call here: the reply to
        a stop arrives while the last frames of the stream are still coming, and
        that race is what `request_while_streaming` exists for."""
        self.configurator.stop_stream()
        return "stopped"


def _field(status, key, spec):
    """One field of a status pane: the number, or the word for one the firmware
    does not carry.

    Two firmwares answer this window now, and they do not carry the same
    fields. A missing one is shown as `?` rather than as a zero, because "this
    board does not say" and "this board says zero" are different answers - and
    a zero latitude is a place in the Atlantic.
    """
    value = status.get(key)
    return "?" if value is None else (spec % value)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", nargs="?", const="",
                        help="run the simulator and talk to it (default path: "
                             "$AK_SIM or build-host/aerialkit-sim)")
    parser.add_argument("--host", help="HOST:PORT of a firmware on a network")
    parser.add_argument("--port", help="serial port of a board's console")
    args = parser.parse_args()

    if args.sim is not None:
        path = args.sim or os.environ.get("AK_SIM", "build-host/aerialkit-sim")
        transport = Pipe([path])
    elif args.host:
        transport = Socket(args.host)
    elif args.port:
        transport = Serial(args.port)
    else:
        parser.error("say where the firmware is: --sim, --host or --port")

    # Which firmware is on the other end, before any window is built: our own
    # protocol first, then MSP (Betaflight and INAV), and "unknown" - which the
    # window then says in words rather than showing an empty table.
    from msp import Deadline

    reader = Deadline(transport)
    kind, configurator = detect(reader.read_some, transport.write)
    if configurator is None:
        print("no answer: neither AerialKit's protocol nor MSP (Betaflight, "
              "INAV) - ArduPilot and PX4 speak MAVLink, which this window "
              "does not read yet", file=sys.stderr)
        transport.close()
        return 1
    if kind != "aerialkit":
        print("connected to %s" % configurator.describe())

    link = Link(configurator)

    import tkinter
    root = tkinter.Tk()
    Window(root, configurator, link)
    try:
        root.mainloop()
    finally:
        transport.close()


if __name__ == "__main__":
    sys.exit(main())
