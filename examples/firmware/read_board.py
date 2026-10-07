#!/usr/bin/env python3
"""Read AK firmware identity and attitude over USB or TCP; never writes parameters."""
from __future__ import annotations
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "firmware" / "tools"))
import akproto

@contextmanager
def connection(port: str | None, host: str | None):
    if port:
        import serial
        link = serial.Serial(port, 115200, timeout=2, write_timeout=2)
        try:
            yield akproto.Client(link.read, link.write)
        finally:
            akproto.leave_port_as_found(link)
            link.close()
    else:
        address, sep, number = (host or "").rpartition(":")
        if not sep or not address:
            raise ValueError("Use --host ADDRESS:PORT for the firmware TCP config endpoint")
        with socket.create_connection((address, int(number)), timeout=3) as link:
            link.settimeout(3)
            yield akproto.Client(link.recv, link.sendall)

def read_once(client):
    """The same protocol parsers used by the firmware's Python tools."""
    return akproto.parse_status(client.request(akproto.STATUS))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="USB serial device, e.g. /dev/ttyACM0 or COM5")
    source.add_argument("--host", help="ESP32 firmware TCP config endpoint, ADDRESS:PORT")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100000 or args.interval < 0.02:
        parser.error("samples must be 1..100000 and interval at least 0.02 seconds")
    try:
        with connection(args.port, args.host) as client:
            identity = akproto.parse_hello(client.request(akproto.HELLO))
            print(json.dumps({"identity": identity}), file=sys.stderr)
            for index in range(args.samples):
                print(json.dumps({"sample": index, **read_once(client)}), flush=True)
                if index + 1 < args.samples:
                    time.sleep(args.interval)
    except (OSError, ValueError, ImportError) as error:
        parser.exit(1, f"Connection failed: {error}\n")

if __name__ == "__main__":
    main()
