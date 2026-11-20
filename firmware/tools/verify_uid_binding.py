#!/usr/bin/env python3
"""Read, set and verify matching ExpressLRS UIDs on two native-USB endpoints.

This is the hardware acceptance driver for the Rev A3 pair. It performs no
mutation unless both ``--write`` and ``--confirm`` are present. The optional RF
bench starts only after both endpoints report the same non-zero UID.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from aerial_kit.comms import (  # noqa: E402
    CRSF_ADDRESS_CRSF_TRANSMITTER,
    CRSF_ADDRESS_ELRS_LUA,
    CRSF_ADDRESS_RECEIVER,
    CRSF_BAUD,
    CRSF_RX_FC_BAUD,
    CrsfLink,
    CrsfParameterClient,
    CrsfProtocolError,
    SerialTransport,
    binding_uid,
    format_uid,
    run_two_board_bench,
)


def uid_client(path: str, role: str, timeout: float) -> tuple[CrsfLink, CrsfParameterClient]:
    tx = role == "tx"
    link = CrsfLink(
        SerialTransport(
            path,
            baudrate=CRSF_BAUD if tx else CRSF_RX_FC_BAUD,
            timeout=timeout,
        )
    )
    return link, CrsfParameterClient(
        link,
        destination=(
            CRSF_ADDRESS_CRSF_TRANSMITTER
            if tx
            else CRSF_ADDRESS_RECEIVER
        ),
        origin=CRSF_ADDRESS_ELRS_LUA,
        timeout=timeout,
    )


def read_uid(path: str, role: str, timeout: float) -> bytes:
    link, client = uid_client(path, role, timeout)
    try:
        return client.read_uid(attempts=2, timeout=max(4.0, timeout))
    finally:
        link.close()


def write_phrase(path: str, role: str, phrase: str, timeout: float) -> tuple[bytes, bytes]:
    link, client = uid_client(path, role, timeout)
    try:
        before = client.read_uid(attempts=2, timeout=max(4.0, timeout))
        client.write_binding_phrase(phrase, allow_write=True)
    finally:
        link.close()
    time.sleep(1.0)
    after = read_uid(path, role, timeout)
    return before, after


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read/set the same ExpressLRS UID on TX and RX over native USB"
    )
    parser.add_argument("--tx", required=True, help="TX native-USB CRSF path")
    parser.add_argument("--rx", required=True, help="RX native-USB CRSF path")
    parser.add_argument("--phrase", help="shared binding phrase")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument(
        "--write",
        action="store_true",
        help="allow a binding-phrase write (also requires --confirm)",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="confirm that --tx and --rx are the intended physical endpoints",
    )
    parser.add_argument(
        "--rf-duration",
        type=float,
        default=0.0,
        help="after UID match, run this many seconds of two-board RC/link-stat testing",
    )
    parser.add_argument("--json", help="write machine-readable evidence to this path or -")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.tx == args.rx:
        raise ValueError("--tx and --rx must be distinct USB endpoints")
    if args.write and (not args.confirm or args.phrase is None):
        raise PermissionError("binding writes require --phrase, --write and --confirm")
    if args.rf_duration < 0:
        raise ValueError("--rf-duration must not be negative")

    result: dict[str, object] = {
        "tx_path": args.tx,
        "rx_path": args.rx,
        "mutated": bool(args.write),
    }
    tx_before = read_uid(args.tx, "tx", args.timeout)
    rx_before = read_uid(args.rx, "rx", args.timeout)
    print(f"initial tx_uid={format_uid(tx_before)}")
    print(f"initial rx_uid={format_uid(rx_before)}")
    result["initial_tx_uid"] = format_uid(tx_before)
    result["initial_rx_uid"] = format_uid(rx_before)

    if args.write:
        expected = binding_uid(args.phrase or "")
        tx_before, tx_after = write_phrase(
            args.tx, "tx", args.phrase or "", args.timeout
        )
        rx_before, rx_after = write_phrase(
            args.rx, "rx", args.phrase or "", args.timeout
        )
        if tx_after != expected or rx_after != expected:
            raise CrsfProtocolError(
                "binding readback mismatch: "
                f"expected {format_uid(expected)}, "
                f"TX {format_uid(tx_after)}, RX {format_uid(rx_after)}"
            )
        result.update(
            {
                "expected_uid": format_uid(expected),
                "tx_uid": format_uid(tx_after),
                "rx_uid": format_uid(rx_after),
                "tx_before_uid": format_uid(tx_before),
                "rx_before_uid": format_uid(rx_before),
            }
        )
        print(f"verified tx_uid={format_uid(tx_after)}")
        print(f"verified rx_uid={format_uid(rx_after)}")
    else:
        result.update(
            {
                "tx_uid": format_uid(tx_before),
                "rx_uid": format_uid(rx_before),
            }
        )

    tx_uid = bytes.fromhex(str(result["tx_uid"]).replace(":", ""))
    rx_uid = bytes.fromhex(str(result["rx_uid"]).replace(":", ""))
    if tx_uid != rx_uid:
        raise CrsfProtocolError("TX and RX UIDs do not match; RF test refused")
    if tx_uid == bytes(6):
        raise CrsfProtocolError("TX and RX are both unbound; RF test refused")

    if args.rf_duration > 0:
        if not args.write or not args.confirm:
            raise PermissionError("RF bench requires --write and --confirm")
        tx_transport = SerialTransport(
            args.tx, baudrate=CRSF_BAUD, timeout=0.05
        )
        rx_transport = SerialTransport(
            args.rx, baudrate=CRSF_RX_FC_BAUD, timeout=0.05
        )
        try:
            bench = run_two_board_bench(
                tx_transport,
                rx_transport,
                tx_path=args.tx,
                rx_path=args.rx,
                duration=args.rf_duration,
            )
        finally:
            tx_transport.close()
            rx_transport.close()
        result["rf_bench"] = bench.to_dict()
        print(f"rf_overall={bench.overall}")
        print(f"rx_rc_capture={bench.requirements.get('rx_rc_capture')}")
        print(f"link_statistics={bench.requirements.get('link_statistics')}")

    if args.json:
        text = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.json == "-":
            sys.stdout.write(text)
        else:
            Path(args.json).write_text(text, encoding="utf-8")
    return 0 if "rf_bench" not in result or result["rf_bench"]["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
