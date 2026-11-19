#!/usr/bin/env python3
"""Print one set of CRSF RC channels, or run a hardware-free demo."""

from __future__ import annotations

import argparse
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms.crsf import (
    CrsfLink,
    CrsfParser,
    SerialTransport,
    make_rc_channels_frame,
    unpack_channels,
)


def demo() -> list[int]:
    raw = make_rc_channels_frame(tuple(range(992, 1008)))
    frame = CrsfParser().feed(raw)[0]
    return list(unpack_channels(frame.payload))


def live(port: str) -> list[int]:
    link = CrsfLink(SerialTransport(port, baudrate=400_000))
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            for frame in link.poll(512, 0.2):
                if frame.frame_type == 0x16 and len(frame.payload) == 22:
                    return list(unpack_channels(frame.payload))
        raise TimeoutError("no CRSF RC frame arrived")
    finally:
        link.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()
    channels = demo() if args.demo else live(args.port)
    print(" ".join(map(str, channels)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
