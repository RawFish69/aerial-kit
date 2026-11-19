#!/usr/bin/env python3
"""Read Aerial Kit identity over USB or a CRSF radio."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms.akp import AkClient
from aerial_kit.comms.akp.codec import parse_hello
from aerial_kit.comms.crsf import CRSF_BAUD, CrsfAkpTunnel, CrsfLink
from aerial_kit.comms.transport import SerialTransport


DEMO_HELLO = bytes.fromhex(
    "aa5501811c0161657269616c6b69742d66343035002e003000"
    "c1010000920638cfd0a3"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--radio")
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()

    if args.demo:
        hello = parse_hello(DEMO_HELLO[5:-2])
    else:
        port = args.radio or args.port
        if not port:
            parser.error("--port or --radio is required without --demo")
        if args.radio:
            link = CrsfLink(
                SerialTransport(port, baudrate=CRSF_BAUD, timeout=0.05)
            )
            client = AkClient(CrsfAkpTunnel(link))
        else:
            client = AkClient(SerialTransport(port, baudrate=115_200))
        try:
            hello = client.hello()
        finally:
            client.close()
    print(
        f"{hello['product']} version={hello['version']} "
        f"parameters={hello['count']} changed={hello['changed']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
