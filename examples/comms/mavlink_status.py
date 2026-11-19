#!/usr/bin/env python3
"""Identify a PX4/ArduPilot vehicle, or decode a demo heartbeat."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms.mavlink import codec
from aerial_kit.comms.mavlink.client import MavlinkVehicle
from aerial_kit.comms.transport import TcpTransport, UdpTransport


def demo() -> tuple[str, dict]:
    raw = codec.build(
        "HEARTBEAT",
        {
            "type": 2,
            "autopilot": 3,
            "base_mode": 0,
            "custom_mode": 4,
            "system_status": 4,
            "mavlink_version": 3,
        },
        system=1,
        component=1,
    )
    return codec.parse(raw)[:2]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host")
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()

    if args.demo:
        name, fields = demo()
        print(f"{name} autopilot={fields['autopilot']} type={fields['type']}")
        return 0
    if not args.host or ":" not in args.host:
        parser.error("--host host:port is required without --demo")
    host_value = args.host.removeprefix("udp://").removeprefix("tcp://")
    host, raw_port = host_value.rsplit(":", 1)
    transport = (
        UdpTransport.connect(host, int(raw_port))
        if args.host.startswith("udp://")
        else TcpTransport.connect(host.removeprefix("tcp://"), int(raw_port))
    )
    vehicle = MavlinkVehicle(transport)
    try:
        print(vehicle.identify())
    finally:
        vehicle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
