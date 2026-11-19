#!/usr/bin/env python3
"""Validate every shared protocol vector without using protocol encoders."""

from __future__ import annotations

import json
import struct
from pathlib import Path

HERE = Path(__file__).resolve().parent
VECTORS = HERE / "vectors"


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def load(name: str) -> dict[str, object]:
    return json.loads((VECTORS / name).read_text())


def check_crsf(document: dict[str, object]) -> None:
    assert document["schema_version"] == 1
    assert document["protocol"] == "crsf"
    names: set[str] = set()
    for case in document["cases"]:
        assert case["name"] not in names
        names.add(case["name"])
        raw = bytes.fromhex(case["raw_hex"])
        payload = bytes.fromhex(case["payload_hex"])
        assert raw[0] == case["address"]
        assert raw[2] == case["frame_type"]
        assert raw[1] == len(raw) - 2
        assert raw[3:-1] == payload
        assert raw[-1] == crc8_dvb_s2(raw[2:-1])


def check_akp(document: dict[str, object]) -> None:
    assert document["schema_version"] == 1
    assert document["protocol"] == "akp"
    names: set[str] = set()
    for case in document["cases"]:
        assert case["name"] not in names
        names.add(case["name"])
        raw = bytes.fromhex(case["raw_hex"])
        payload = bytes.fromhex(case["payload_hex"])
        assert raw[:2] == b"\xAA\x55"
        assert raw[2] == 1
        assert raw[3] == case["command"]
        assert raw[4] == len(payload)
        assert raw[5:-2] == payload
        assert struct.unpack("<H", raw[-2:])[0] == crc16_ccitt_false(raw[2:-2])
        assert bool(case["command"] & 0x80) == bool(case["response"])


def check_mavlink(document: dict[str, object]) -> None:
    assert document["schema_version"] == 1
    assert document["protocol"] == "mavlink"
    names: set[str] = set()
    for case in document["cases"]:
        assert case["name"] not in names
        names.add(case["name"])
        raw = bytes.fromhex(case["raw_hex"])
        assert raw[0] == 0xFD
        assert raw[1] == len(raw) - 12
        assert raw[4] == case["sequence"]
        assert raw[5] == case["system_id"]
        assert raw[6] == case["component_id"]
        assert int.from_bytes(raw[7:10], "little") == case["message_id"]
        assert raw[10:-2].hex() == case["payload_hex"]
        assert struct.unpack("<H", raw[-2:])[0] == crc16_x25(
            raw[1:-2] + bytes((case["crc_extra"],))
        )


def reassemble(segments: list[bytes]) -> bytes:
    by_sequence = {}
    total = None
    for segment in segments:
        assert segment[0] == 0xA1
        assert total is None or segment[3] == total
        total = segment[3]
        sequence = segment[2] & 0x3F
        assert sequence not in by_sequence
        by_sequence[sequence] = segment[4:]
    assert by_sequence
    assert 0 in by_sequence
    assert max(by_sequence) == len(by_sequence) - 1
    output = b"".join(by_sequence[index] for index in range(len(by_sequence)))
    assert len(output) == total
    return output


def check_tunnel(document: dict[str, object]) -> None:
    assert document["schema_version"] == 1
    assert document["protocol"] == "akp-over-crsf"
    for case in document["cases"]:
        segments = [bytes.fromhex(item) for item in case["segments_hex"]]
        frames = [bytes.fromhex(item) for item in case["crsf_frames_hex"]]
        assert len(segments) == len(frames)
        assert len(case["crsf_frame_types"]) == len(frames)
        for segment, raw, frame_type in zip(
                segments, frames, case["crsf_frame_types"]):
            assert raw[1] == len(raw) - 2
            assert raw[2] == frame_type
            assert raw[-1] == crc8_dvb_s2(raw[2:-1])
            assert segment in raw
        if "expected_error" in case:
            selected = [segments[index] for index in case["delivery_order"]]
            try:
                reassemble(selected)
            except AssertionError:
                continue
            raise AssertionError(f"{case['name']}: loss unexpectedly reassembled")
        selected = [segments[index] for index in case["delivery_order"]]
        assert reassemble(selected).hex() == case["reassembled_hex"]


def main() -> int:
    check_crsf(load("crsf.json"))
    check_akp(load("akp.json"))
    check_mavlink(load("mavlink.json"))
    check_tunnel(load("akp-over-crsf.json"))
    print("protocol vectors: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
