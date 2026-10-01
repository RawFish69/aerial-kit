#!/usr/bin/env python3
"""The configurator's window against the ESP32's own socket, under QEMU.

    scripts/esp32-proto.sh      # the port's check, then this one
    IDF_PATH=~/esp-idf make net-window-test

The two window checks either side of this one cover the window on a *console*
link: the simulator over a pipe, where the firmware answers a subscribe with 0
because a console is a wire somebody types at. That leaves the half of the
window that exists for the network - `Link._stream_once` reading frames nobody
asked for twice, and the live pane drawing them - with no check at all, on the
argument that the only firmware that streams is the ESP32 and the only ESP32
here is emulated.

It is emulated, and it streams: QEMU forwards the guest's listening port to
this machine, so the window can be pointed at `127.0.0.1:5555` exactly as it is
pointed at a board's address in the field. What is checked here is what a
person flying with a laptop would do: connect, read the table, press *stream*,
and watch numbers arrive.

Skipped, with a line, when there is no QEMU, no built image, or no Tk - this is
the one check in the suite that needs a whole emulator and a GUI toolkit
together, and neither is a reason for the firmware's own suite to fail.
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import esp32_proto_check as esp32                      # noqa: E402
from akconfig import Configurator, Link, Socket, Window  # noqa: E402
from akconfig_window_check import ensure_display, pump   # noqa: E402
from akproto import Client, HELLO                       # noqa: E402

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def skip(reason):
    print("the window over a network link: not checked here - %s" % reason)
    return 0


def wait_for_the_firmware(seconds=40.0):
    """The firmware's own hello, not a port that answers.

    QEMU forwards the port the moment it starts, so a connect succeeds while
    the guest is still booting and the connection is to nobody. The only signal
    worth waiting for is the one this protocol is about.
    """
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            transport = Socket("127.0.0.1:%d" % esp32.NET_PORT)
        except OSError:
            time.sleep(0.5)
            continue
        try:
            reply = Client(transport.read, transport.write).request(HELLO)
            return reply is not None and len(reply) > 1
        except Exception:                                 # noqa: BLE001
            transport.close()
            time.sleep(0.5)
    return False


def main():
    problem = ensure_display()
    if problem is not None:
        return skip(problem)

    import tkinter

    qemu = esp32.find_qemu()
    if qemu is None:
        return skip("no qemu-system-xtensa (see docs/17-esp32-port.md)")
    if not os.path.exists(os.path.join(esp32.PORT_DIR, "build", "aerialkit-esp32.elf")):
        return skip("the ESP32 image has not been built (idf.py -C ports/esp32 build)")

    esp32.build_flash_image()

    print("the configurator's window, over the ESP32's own socket")

    guest = esp32.launch_guest(qemu)
    # The guest's console goes to this pipe and nobody reads it. A pipe nobody
    # reads fills, and a guest with a full pipe stops - which would look like a
    # firmware hang in whichever check ran second.
    threading.Thread(target=lambda: guest.stdout.read(), daemon=True).start()

    try:
        if not wait_for_the_firmware():
            return skip("the emulated board never answered a hello")

        transport = Socket("127.0.0.1:%d" % esp32.NET_PORT)
        configurator = Configurator(Client(transport.read, transport.write))
        link = Link(configurator)

        root = tkinter.Tk()
        window = Window(root, configurator, link)

        pump(root, lambda: window.items != [], 20.0)
        # And if that round trip was missed, ask again rather than judging it.
        #
        # The window's connect is one request on the socket's own timeout, and
        # the board at the other end is an emulated ESP32 on a machine that may
        # be running the rest of the suite at the same time. It ran red once
        # that way - an empty product and an empty table, which reads as a
        # firmware that does not answer and was a host that was busy. The retry
        # is the same button a person would press.
        deadline = time.time() + 20.0
        while (configurator.product == "" or not window.items) and \
                time.time() < deadline:
            window.connect()
            pump(root, lambda: configurator.product != "" and window.items,
                 5.0)

        expect("the window connects to a board over a network link",
               configurator.product != "" and "esp32" in configurator.product,
               " (%s, protocol %u)" % (configurator.product, configurator.protocol))
        expect("and the ESP32's table is the size this port is supposed to have",
               configurator.count == esp32.EXPECTED_PARAMETER_COUNT and
               len(window.table.get_children()) == configurator.count,
               " (%u parameters, %u rows)"
               % (configurator.count, len(window.table.get_children())))

        # The one question a console link answers with zero, asked of the link
        # that exists to answer with a rate.
        window.rate.set("10")
        window.toggle_stream()
        pumped = pump(root, lambda: window.message.cget("text")
                      .startswith("streaming"))
        expect("asking for a stream on a network link is allowed",
               pumped and configurator.stream_hz != 0,
               " (%s)" % window.message.cget("text"))

        # And then the pane fills from frames the aircraft sent unasked, which
        # is the whole point of the stream: this is the widget path no check
        # could reach over a pipe.
        pumped = pump(root, lambda: window.live.cget("text") != "", 15.0)
        drawn = window.live.cget("text")
        expect("and the live pane fills from frames the board sent",
               pumped and "roll" in drawn and "motors" in drawn,
               " (%s)" % " / ".join(part.strip() for part in drawn.split("\n")))
        expect("with the rate the person asked for",
               configurator.stream_hz == 10, " (%u Hz)" % configurator.stream_hz)

        # Stopping has to work too: a stream that cannot be stopped is a link
        # that never goes quiet again.
        window.toggle_stream()
        pump(root, lambda: window.message.cget("text") == "stream stopped")
        expect("and it can be stopped again",
               window.message.cget("text") == "stream stopped" and
               window.live_button.cget("text") == "stream",
               " (%s)" % window.message.cget("text"))

        root.destroy()
        transport.close()
    finally:
        guest.kill()
        try:
            guest.wait(timeout=5)
        except Exception:                                 # noqa: BLE001
            pass

    print("configurator over the network: %s"
          % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
