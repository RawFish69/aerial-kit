#!/usr/bin/env python3
"""Generate the shared protocol vectors from their authoritative definitions.

CRSF layout and constants come from ExpressLRS 4.1.0 and the firmware encoder.
AKP comes from firmware/src/core/ak_proto.* and firmware/docs/16-protocol.md.
MAVLink frames are packed by pymavlink 2.4.49.

MAVLink generation needs the optional dependency:

    python -m pip install pymavlink==2.4.49
    python protocols/generate_vectors.py
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "protocols" / "vectors"

CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA

CRSF_TYPE_GPS = 0x02
CRSF_TYPE_BATTERY = 0x08
CRSF_TYPE_BARO = 0x09
CRSF_TYPE_LINK_STATISTICS = 0x14
CRSF_TYPE_RC_CHANNELS = 0x16
CRSF_TYPE_ATTITUDE = 0x1E
CRSF_TYPE_FLIGHT_MODE = 0x21
CRSF_TYPE_DEVICE_PING = 0x28
CRSF_TYPE_DEVICE_INFO = 0x29
CRSF_TYPE_PARAMETER_ENTRY = 0x2B
CRSF_TYPE_PARAMETER_READ = 0x2C
CRSF_TYPE_PARAMETER_WRITE = 0x2D
CRSF_TYPE_COMMAND = 0x32
CRSF_TYPE_MSP_REQUEST = 0x7A
CRSF_TYPE_MSP_RESPONSE = 0x7B
CRSF_TYPE_MSP_WRITE = 0x7C

AKP_SYNC1 = 0xAA
AKP_SYNC2 = 0x55
AKP_VERSION = 1
AKP_RESPONSE_BIT = 0x80

TUNNEL_MAGIC = 0xA1
TUNNEL_FIRST = 0x40
TUNNEL_LAST = 0x80
TUNNEL_SEQUENCE_MASK = 0x3F


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crsf_frame(address: int, frame_type: int, payload: bytes) -> bytes:
    body = bytes((frame_type,)) + payload
    return bytes((address, len(body) + 1)) + body + bytes((crc8_dvb_s2(body),))


def pack_channels(channels: tuple[int, ...]) -> bytes:
    if len(channels) != 16:
        raise ValueError("CRSF needs sixteen channels")
    output = bytearray(22)
    accumulator = 0
    bits = 0
    offset = 0
    for channel in channels:
        accumulator |= channel << bits
        bits += 11
        while bits >= 8:
            output[offset] = accumulator & 0xFF
            offset += 1
            accumulator >>= 8
            bits -= 8
    return bytes(output)


def crsf_case(
    name: str,
    address: int,
    frame_type: int,
    payload: bytes,
    decoded: dict[str, object],
) -> dict[str, object]:
    raw = crsf_frame(address, frame_type, payload)
    return {
        "name": name,
        "address": address,
        "frame_type": frame_type,
        "payload_hex": payload.hex(),
        "raw_hex": raw.hex(),
        "decoded": decoded,
    }


def crsf_vectors() -> dict[str, object]:
    centre = (992,) * 16
    extremes = (172, 1811, 172, 1811, 992, 992, 992, 992,
                172, 1811, 172, 1811, 992, 992, 992, 992)
    battery = struct.pack(">HH", 168, 123) + (1500).to_bytes(3, "big") + bytes((88,))
    gps = struct.pack(
        ">iiHHHB",
        473_977_000,
        85_440_000,
        360,
        9000,
        1450,
        12,
    )
    attitude = struct.pack(">hhh", 1000, -5000, 12000)
    baro = struct.pack(">Hh", 11_234, -250)
    link = bytes((90, 92, 88, 7, 1, 2, 3, 80, 70, 4))
    device_name = b"aerialkit-f405\0"
    device_info = bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                         CRSF_ADDRESS_FLIGHT_CONTROLLER))
    device_info += device_name + bytes(12) + bytes((0, 1))

    return {
        "schema_version": 1,
        "protocol": "crsf",
        "source": {
            "layout": "ExpressLRS 4.1.0 src/include/crsf_protocol.h",
            "encoder": "firmware/src/core/flight/ak_crsf.c and ak_crsf_telemetry.c",
        },
        "cases": [
            crsf_case("rc_channels_centre", 0xC8, CRSF_TYPE_RC_CHANNELS,
                      pack_channels(centre),
                      {"channels": list(centre), "units": "raw CRSF counts"}),
            crsf_case("rc_channels_extremes", 0xEA, CRSF_TYPE_RC_CHANNELS,
                      pack_channels(extremes),
                      {"channels": list(extremes), "units": "raw CRSF counts"}),
            crsf_case("attitude", 0xEA, CRSF_TYPE_ATTITUDE, attitude,
                      {"pitch_rad": 0.10, "roll_rad": -0.50, "yaw_rad": 1.20}),
            crsf_case("battery", 0xEA, CRSF_TYPE_BATTERY, battery,
                      {"voltage_v": 16.8, "current_a": 12.3,
                       "capacity_mah": 1500, "remaining_percent": 88}),
            crsf_case("gps", 0xEA, CRSF_TYPE_GPS, gps,
                      {"latitude_deg": 47.3977, "longitude_deg": 8.544,
                       "groundspeed_mps": 10.0, "heading_deg": 90.0,
                       "altitude_m": 450.0, "satellites": 12}),
            crsf_case("barometer", 0xEA, CRSF_TYPE_BARO, baro,
                      {"altitude_m": 123.4, "vertical_speed_mps": -2.5}),
            crsf_case("link_statistics", 0xEA, CRSF_TYPE_LINK_STATISTICS, link,
                      {"uplink_rssi_1_dbm": -90, "uplink_rssi_2_dbm": -92,
                       "uplink_link_quality": 88, "uplink_snr_db": 7,
                       "active_antenna": 1, "rf_mode": 2,
                       "uplink_tx_power": 3, "downlink_rssi_dbm": -80,
                       "downlink_link_quality": 70,
                       "downlink_snr_db": 4}),
            crsf_case("flight_mode", 0xEA, CRSF_TYPE_FLIGHT_MODE, b"ANGLE\0",
                      {"mode": "ANGLE"}),
            crsf_case("device_ping",
                      0xC8, CRSF_TYPE_DEVICE_PING,
                      bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_FLIGHT_CONTROLLER, 0, 0)),
                      {"destination": "radio transmitter", "origin": "flight controller",
                       "note": "extended CRSF address header"}),
            crsf_case("device_info",
                      0xC8, CRSF_TYPE_DEVICE_INFO, device_info,
                      {"destination": "radio transmitter", "origin": "flight controller",
                       "name": "aerialkit-f405", "parameter_count": 0,
                       "parameter_version": 1}),
            crsf_case("parameter_settings_entry",
                      0xEA, CRSF_TYPE_PARAMETER_ENTRY,
                      bytes((CRSF_ADDRESS_FLIGHT_CONTROLLER,
                             CRSF_ADDRESS_RADIO_TRANSMITTER, 0, 1, 0x0B,
                             0x01, 0x00, 0x00, 0x00, 0x00)),
                      {"note": "representative extended parameter-frame envelope",
                       "field": 1, "type": "uint8"},
                      ),
            crsf_case("parameter_read",
                      0xC8, CRSF_TYPE_PARAMETER_READ,
                      bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_FLIGHT_CONTROLLER, 1, 0)),
                      {"field": 1, "chunk": 0}),
            crsf_case("parameter_write",
                      0xC8, CRSF_TYPE_PARAMETER_WRITE,
                      bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_FLIGHT_CONTROLLER, 1, 2)),
                      {"field": 1, "value": 2, "chunks": 1,
                       "write": True}),
            crsf_case("command",
                      0xC8, CRSF_TYPE_COMMAND,
                      bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_FLIGHT_CONTROLLER,
                             0x10, 0x01)),
                      {"destination": "radio transmitter",
                       "origin": "flight controller",
                       "subsystem": "receiver", "bind": True,
                       "write": True}),
            crsf_case("msp_request",
                      0xC8, CRSF_TYPE_MSP_REQUEST,
                      bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_FLIGHT_CONTROLLER, 0, 0, 0, 1, 0, 1)),
                      {"note": "candidate AKP tunnel envelope", "direction": "uplink"}),
            crsf_case("msp_response",
                      0xEA, CRSF_TYPE_MSP_RESPONSE,
                      bytes((CRSF_ADDRESS_FLIGHT_CONTROLLER,
                             CRSF_ADDRESS_RADIO_TRANSMITTER, 0, 0, 0, 1, 0, 1)),
                      {"note": "candidate AKP tunnel envelope", "direction": "downlink"}),
            crsf_case("msp_write",
                      0xC8, CRSF_TYPE_MSP_WRITE,
                      bytes((CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_FLIGHT_CONTROLLER, 0, 0, 0, 1, 0, 1)),
                      {"note": "candidate AKP tunnel envelope", "direction": "uplink",
                       "write": True}),
        ],
    }


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def akp_frame(command: int, payload: bytes = b"") -> bytes:
    body = bytes((AKP_VERSION, command, len(payload))) + payload
    return bytes((AKP_SYNC1, AKP_SYNC2)) + body + struct.pack("<H", crc16_ccitt_false(body))


COMMANDS: dict[int, tuple[str, bytes, dict[str, object], bool]] = {
    0x01: ("hello", b"", {}, False),
    0x02: ("param_get", bytes((0,)), {"index": 0}, False),
    0x03: ("param_set", bytes((0,)) + b"0.250\0", {"index": 0, "value": "0.250"}, True),
    0x04: ("param_save", b"", {}, True),
    0x05: ("status", b"", {}, False),
    0x06: ("log_info", b"", {}, False),
    0x07: ("log_get", struct.pack("<H", 0), {"index": 0}, False),
    0x08: ("telemetry", bytes((10,)), {"rate_hz": 10}, False),
    0x09: ("log_source", bytes((0,)), {"source": 0}, False),
    0x0A: ("param_info", bytes((0,)), {"first": 0}, False),
    0x0B: ("param_help", bytes((0,)) + struct.pack("<H", 0),
           {"index": 0, "offset": 0}, False),
    0x0C: ("param_default", bytes((1, 0)), {"mode": "one", "index": 0}, True),
    0x0D: ("rc_channels", b"", {}, False),
    0x0E: ("sensor_info", bytes((0,)), {"topic": "imu"}, False),
    0x0F: ("output_info", b"", {}, False),
    0x10: ("output_test", bytes((0, 0, 0, 5)),
           {"op": "hold", "kind": "motor", "index": 0, "percent": 5}, True),
    0x11: ("log_stream", bytes((0,)) + struct.pack("<HH", 0, 4) + bytes((10,)),
           {"source": 0, "start": 0, "count": 4, "rate_hz": 10}, False),
    0x12: ("preflight", bytes((0,)) + struct.pack("<H", 0),
           {"index": 0, "offset": 0}, False),
    0x13: ("calibrate", bytes((0,)), {"verb": "status"}, True),
    0x14: ("mission", bytes((0,)), {"verb": "status"}, False),
    0x15: ("perf", b"", {}, False),
    0x16: ("motor_telemetry", b"", {}, False),
}


def fixture_frames() -> dict[str, dict[str, str]]:
    path = ROOT / "apps" / "configurator" / "tests" / "fixtures" / "aerialkit.json"
    data = json.loads(path.read_text())
    return data["frames"]


def akp_vectors() -> dict[str, object]:
    frames = fixture_frames()
    cases = []
    for command, (name, payload, decoded, write) in COMMANDS.items():
        cases.append({
            "name": f"request_{name}",
            "command": command,
            "response": False,
            "raw_hex": akp_frame(command, payload).hex(),
            "payload_hex": payload.hex(),
            "decoded": decoded,
            "write": write,
        })

    for name, key in (
        ("hello", "hello"),
        ("status", "status"),
        ("param_get", "param_get_0"),
        ("param_get_refused", "param_get_past_end"),
        ("param_set_refused", "param_set_refused"),
        ("param_save", "param_save"),
        ("telemetry_refused", "telemetry_rate"),
    ):
        item = frames[key]
        cases.append({
            "name": f"captured_{name}",
            "command": int(item["request"][6:8], 16),
            "response": False,
            "raw_hex": item["request"],
            "payload_hex": item["request"][10:-4],
            "decoded": {"meaning": item["note"]},
            "write": key in {"param_set_refused", "param_save"},
        })
        cases.append({
            "name": f"captured_{name}_reply",
            "command": int(item["reply"][6:8], 16),
            "response": True,
            "raw_hex": item["reply"],
            "payload_hex": item["reply"][10:-4],
            "decoded": {"meaning": item["note"]},
            "write": False,
        })

    return {
        "schema_version": 1,
        "protocol": "akp",
        "source": {
            "wire": "firmware/src/core/ak_proto.h",
            "semantics": "firmware/docs/16-protocol.md",
            "captured_fixtures": "apps/configurator/tests/fixtures/aerialkit.json",
        },
        "cases": cases,
    }


def tunnel_segments(payload: bytes, session: int, chunk_size: int) -> list[bytes]:
    chunks = [payload[index:index + chunk_size]
              for index in range(0, len(payload), chunk_size)]
    segments = []
    for sequence, chunk in enumerate(chunks):
        flags = sequence & TUNNEL_SEQUENCE_MASK
        if sequence == 0:
            flags |= TUNNEL_FIRST
        if sequence == len(chunks) - 1:
            flags |= TUNNEL_LAST
        segments.append(bytes((TUNNEL_MAGIC, session, flags, len(payload))) + chunk)
    return segments


def tunnel_vectors() -> dict[str, object]:
    akp = akp_frame(0x01)
    session = 7
    up = tunnel_segments(akp, session, 4)
    down = tunnel_segments(akp, session, 32)
    up_frames = [
        crsf_frame(0xC8, CRSF_TYPE_MSP_REQUEST,
                   bytes((0xEA, 0xC8)) + segment).hex()
        for segment in up
    ]
    down_frames = [
        crsf_frame(0xEA, CRSF_TYPE_MSP_RESPONSE,
                   bytes((0xC8, 0xEA)) + segment).hex()
        for segment in down
    ]
    return {
        "schema_version": 1,
        "protocol": "akp-over-crsf",
        "source": {
            "crsf": "ExpressLRS 4.1.0 src/include/crsf_protocol.h MSP frame types",
            "akp": "firmware/src/core/ak_proto.h",
        },
        "tunnel": {
            "magic": TUNNEL_MAGIC,
            "session_bytes": 1,
            "seq_flags": {
                "sequence_mask": TUNNEL_SEQUENCE_MASK,
                "first": TUNNEL_FIRST,
                "last": TUNNEL_LAST,
            },
            "total_length_bytes": 1,
            "header_fields": ["magic", "session", "seq_flags", "total_length"],
            "uplink_max_chunk_bytes": 4,
            "downlink_max_chunk_bytes": 54,
            "forwarding_status": "candidate; verify TX/RX MSP forwarding before Phase 5 lands",
        },
        "cases": [
            {
                "name": "hello_uplink_reassembled_out_of_order",
                "session": session,
                "akp_hex": akp.hex(),
                "segments_hex": [segment.hex() for segment in up],
                "crsf_frames_hex": up_frames,
                "crsf_frame_types": [CRSF_TYPE_MSP_REQUEST] * len(up_frames),
                "delivery_order": [1, 0],
                "reassembled_hex": akp.hex(),
            },
            {
                "name": "hello_downlink_single_frame",
                "session": session,
                "akp_hex": akp.hex(),
                "segments_hex": [segment.hex() for segment in down],
                "crsf_frames_hex": down_frames,
                "crsf_frame_types": [CRSF_TYPE_MSP_RESPONSE] * len(down_frames),
                "delivery_order": [0],
                "reassembled_hex": akp.hex(),
            },
            {
                "name": "lost_first_segment",
                "session": session,
                "akp_hex": akp.hex(),
                "segments_hex": [segment.hex() for segment in up],
                "crsf_frames_hex": up_frames,
                "crsf_frame_types": [CRSF_TYPE_MSP_REQUEST] * len(up_frames),
                "delivery_order": [1],
                "expected_error": "missing-first-segment",
            },
        ],
    }


def mavlink_case(mav_module, name: str, values: dict[str, object]) -> dict[str, object]:
    message_cls = getattr(mav_module, f"MAVLink_{name.lower()}_message")
    message = message_cls(**values)
    mav = mav_module.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.seq = 0
    raw = message.pack(mav)
    crc_extra = mav_module.mavlink_map[message_cls.id].crc_extra
    return {
        "name": name.lower(),
        "message": name,
        "message_id": message_cls.id,
        "crc_extra": crc_extra,
        "sequence": 0,
        "system_id": 1,
        "component_id": 1,
        "payload_hex": bytes(message.get_msgbuf()[10:-2]).hex(),
        "raw_hex": raw.hex(),
        "decoded": values,
    }


def mavlink_vectors() -> dict[str, object]:
    try:
        from pymavlink.dialects.v20 import common as mav_module
    except ImportError as exc:
        raise SystemExit(
            "MAVLink vectors need pymavlink==2.4.49; install the mavlink extra"
        ) from exc

    cases = [
        mavlink_case(mav_module, "HEARTBEAT", {
            "type": 2, "autopilot": 12, "base_mode": 81,
            "custom_mode": 0, "system_status": 4, "mavlink_version": 3,
        }),
        mavlink_case(mav_module, "ATTITUDE", {
            "time_boot_ms": 1234, "roll": 0.1, "pitch": -0.2,
            "yaw": 1.3, "rollspeed": 0.01, "pitchspeed": -0.02,
            "yawspeed": 0.03,
        }),
        mavlink_case(mav_module, "GLOBAL_POSITION_INT", {
            "time_boot_ms": 1234, "lat": 47_397_700, "lon": 8_544_000,
            "alt": 450_000, "relative_alt": 12_000, "vx": 300,
            "vy": -100, "vz": 20, "hdg": 9000,
        }),
        mavlink_case(mav_module, "GPS_RAW_INT", {
            "time_usec": 1_700_000_000_000_000, "fix_type": 3,
            "lat": 47_397_700, "lon": 8_544_000, "alt": 450_000,
            "eph": 90, "epv": 120, "vel": 1000, "cog": 9000,
            "satellites_visible": 14,
        }),
        mavlink_case(mav_module, "PARAM_REQUEST_LIST", {
            "target_system": 1, "target_component": 1,
        }),
        mavlink_case(mav_module, "PARAM_VALUE", {
            "param_id": b"rate_kp_roll", "param_value": 0.25,
            "param_type": 9, "param_count": 46, "param_index": 0,
        }),
        mavlink_case(mav_module, "COMMAND_LONG", {
            "param1": 0.0, "param2": 0.0, "param3": 0.0,
            "param4": 0.0, "param5": 0.0, "param6": 0.0, "param7": 0.0,
            "target_system": 1, "target_component": 1, "command": 400,
            "confirmation": 0,
        }),
        mavlink_case(mav_module, "STATUSTEXT", {
            "severity": 6, "text": b"preflight passed",
        }),
        mavlink_case(mav_module, "SET_POSITION_TARGET_LOCAL_NED", {
            "time_boot_ms": 1234, "target_system": 1,
            "target_component": 1, "coordinate_frame": 1,
            "type_mask": 0x0FF8, "x": 1.0, "y": 2.0, "z": -3.0,
            "vx": 0.1, "vy": 0.2, "vz": -0.3,
            "afx": 0.0, "afy": 0.0, "afz": 0.0,
            "yaw": 0.0, "yaw_rate": 0.0,
        }),
    ]
    return {
        "schema_version": 1,
        "protocol": "mavlink",
        "source": {
            "implementation": "pymavlink 2.4.49",
            "dialect": "v20 common",
        },
        "cases": cases,
    }


def write_json(path: Path, data: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(jsonable(data), indent=2, sort_keys=True) + "\n")


def jsonable(value):
    if isinstance(value, bytes):
        try:
            text = value.rstrip(b"\0").decode("ascii")
        except UnicodeDecodeError:
            return {"hex": value.hex()}
        return text if text.isprintable() else {"hex": value.hex()}
    if isinstance(value, dict):
        return {key: jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    builders: tuple[tuple[str, Callable[[], dict[str, object]]], ...] = (
        ("crsf.json", crsf_vectors),
        ("akp.json", akp_vectors),
        ("mavlink.json", mavlink_vectors),
        ("akp-over-crsf.json", tunnel_vectors),
    )
    for filename, builder in builders:
        path = args.output / filename
        write_json(path, builder())
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
