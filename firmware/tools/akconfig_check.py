#!/usr/bin/env python3
"""The configurator's half that talks to the aircraft, checked without a window.

    make proto-test          # this is one of the checks it runs

`tools/akconfig.py` is a window and a controller, and the controller is where
everything that can be wrong lives: which command it sends, what it does with
the reply, what it shows a person when the board refuses a value, and whether a
"revert" puts back what was read. A window cannot be checked in a test, and this
is the reason the file is split the way it is.

The other end is the real simulator - the same `aerialkit-sim` the protocol
check and the companion's check drive - so the frames are the firmware's own.
"""

import os
import subprocess
import sys

from akconfig import Configurator
from akproto import Client, HELLO, LOG_FIELDS, c_string
from akproto_check import CAPTURED_TELEMETRY

SIM = os.environ.get("AK_SIM", "build-host/aerialkit-sim")

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def unsaved(client):
    """How many parameters the aircraft says have changed since the last save:
    hello carries it, and the window puts it beside the table."""
    reply = client.request(HELLO)
    product = c_string(reply, 1)
    # version, the product string and its terminator, the count as two bytes,
    # and then this as text - the same offsets the console's own reader uses.
    return int(c_string(reply, 4 + len(product)))


def main():
    process = subprocess.Popen([SIM], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    client = Client(process.stdout.read, lambda data: (
        process.stdin.write(data), process.stdin.flush()))
    configurator = Configurator(client)

    print("the configurator, against the firmware's own protocol code")

    configurator.connect()
    expect("it says hello and learns what the aircraft is",
           configurator.product != "" and configurator.protocol == 1,
           " (%s, protocol %u)" % (configurator.product, configurator.protocol))
    expect("with a parameter count to walk", configurator.count > 0,
           " (%u parameters)" % configurator.count)

    items = configurator.parameters()
    expect("and the table comes back whole, by index and by name",
           len(items) == configurator.count and
           all(name for _index, name, _value in items))

    first_index, first_name, first_value = items[0]

    # A value in range: taken, and read back as the aircraft's own text.
    status, message = configurator.set(first_index, "0.5")
    expect("a value in range is accepted", status == 0,
           " (%s, %s)" % (first_name, message or "no message"))
    read_back = dict((name, value) for _i, name, value in
                     configurator.parameters())[first_name]
    expect("and reads back as what was set", float(read_back) == 0.5,
           " (%s = %s)" % (first_name, read_back))

    # Out of range: refused, with the board's own words about it - which is the
    # half of the reply this tool asked the protocol to grow.
    status, message = configurator.set(first_index, "99")
    expect("a value out of range is refused", status == 2)
    expect("with the board's own reason, which is what a window shows",
           "out of" in message, " (%r)" % message)

    # And the value is still the one that was accepted: a refusal changes
    # nothing, which is the property a configurator is trusted for.
    read_back = dict((name, value) for _i, name, value in
                     configurator.parameters())[first_name]
    expect("and a refused value leaves the old one alone",
           float(read_back) == 0.5)

    # Revert: the session's first value goes back, as real writes.
    restored = configurator.revert()
    read_back = dict((name, value) for _i, name, value in
                     configurator.parameters())[first_name]
    expect("revert writes back the value the session started with",
           restored >= 1 and float(read_back) == float(first_value),
           " (%s back to %s)" % (first_name, read_back))

    # Saving is the board's write, and the count of unsaved changes is what the
    # window puts beside the table.
    configurator.set(first_index, "0.5")
    expect("saving is accepted by the board", configurator.save() == 0)
    expect("and after a save there is nothing unsaved", unsaved(client) == 0)

    # The aircraft's own numbers, which the window shows while it is connected.
    status = configurator.status()
    expect("the state and the attitude come back",
           "flight_state" in status and "roll_deg" in status)
    expect("and the motor outputs with them", len(status["motors"]) == 4,
           " (%s)" % ",".join(str(m) for m in status["motors"]))

    counts = configurator.log_counts()
    expect("every log the aircraft has is counted",
           counts.get("fast") is not None and counts.get("flash") is not None,
           " (%s)" % ", ".join("%s=%s" % (k, v) for k, v in sorted(counts.items())))

    # And pulled, which is what a log is for: the whole ring, record by record,
    # through the same reader the command-line tool uses. The count and the
    # rows are two answers to the same question, and a window that drew one
    # while showing the other would be lying about one of them.
    rows = configurator.pull_log(2)  # the flash log, the one that survives
    expect("a log can be pulled off the aircraft, record by record",
           rows is not None and len(rows) == counts.get("flash"),
           " (%s records pulled, counted %s)"
           % (None if rows is None else len(rows), counts.get("flash")))
    expect("and every record carries the fields the plot reads",
           rows is not None and len(rows) > 0 and
               all(len(row) == len(LOG_FIELDS) for row in rows),
           " (%u fields)" % (len(rows[0]) if rows else 0))
    expect("and its timestamps move forward, which is what a plot draws",
           rows is not None and len(rows) > 1 and
               all(rows[i][0] <= rows[i + 1][0] for i in range(len(rows) - 1)))
    expect("a log this aircraft does not have is `None`, not an empty plot",
           configurator.pull_log(1) is None)  # the long ring is absent here

    # The stream, and the answer that matters as much as the rate: the console
    # link this check is running over *cannot* push frames, and it says so with
    # a zero rather than agreeing to a rate no frame will arrive at. A client
    # that waited on the old answer waited for ever.
    rate = configurator.start_stream(20)
    expect("a console link is told it cannot stream, rather than given a rate",
           rate == 0)

    # Which leaves the live pane's *reader* to be checked the way the protocol
    # check checks its own: against a frame a target actually sent. This one is
    # from the ESP32 under QEMU (docs/evidence/esp32-network.txt), fed back as if
    # it had just arrived.
    captured = bytes.fromhex(CAPTURED_TELEMETRY)
    stream = iter(captured)
    fed = Configurator(Client(lambda _n: bytes([next(stream)]), lambda _d: None))
    frame = fed.next_telemetry()
    expect("and a captured frame parses into the numbers the live pane shows",
           frame["uptime_ms"] == 4300 and len(frame["motors"]) == 4,
           " (uptime %u ms, motors %s)"
           % (frame["uptime_ms"], ",".join(str(m) for m in frame["motors"])))

    process.kill()
    print("configurator end to end: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
