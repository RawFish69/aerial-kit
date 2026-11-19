#!/usr/bin/env python3
"""Read MSP identity from a Betaflight/INAV board."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms import msp


def demo() -> dict:
    payload = b"\x00\x01\x00"
    frame = (
        msp.FRAME_REPLY
        + bytes((len(payload), msp.MSP_API_VERSION))
        + payload
        + bytes((msp.checksum(msp.MSP_API_VERSION, payload),))
    )
    command, decoded = msp.parse(frame)
    return {"command": command, "payload": decoded.hex()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()
    if args.demo:
        print(demo())
        return 0
    if not args.port:
        parser.error("--port is required without --demo")
    port = msp.Port(args.port)
    try:
        client = msp.Msp(port.read, port.write)
        print(msp.describe(msp.identify(client)))
    finally:
        port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
