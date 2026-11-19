from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from aerial_kit.comms.akp import (
    AkClient,
    AkProtocolError,
    AkWriteRefused,
    CALIBRATE_GYRO,
    MISSION_START,
    OUTPUT_TEST_HOLD,
    build,
)
from aerial_kit.comms.akp import codec
from aerial_kit.comms.transport import MemoryTransport


class RoutingTransport(MemoryTransport):
    def __init__(self, responses: dict[int, bytes]) -> None:
        super().__init__(timeout=0)
        self.responses = responses
        self.requests: list[tuple[int, bytes]] = []

    def _write(self, data: bytes) -> int:
        request = data
        assert request[:2] == b"\xAA\x55"
        command = request[3]
        payload = request[5:-2]
        self.requests.append((command, payload))
        response = self.responses.get(command)
        if response is not None:
            self.feed(build(command | 0x80, response))
        return len(data)


def status_payload() -> bytes:
    return (
        bytes((1, 1, 3, 12))
        + struct.pack("<hhh", 100, -200, 300)
        + struct.pack("<ii", 37_000_000, -122_000_000)
        + bytes((10, 20, 30, 40))
    )


def parameter_entry(name: str = "rate_kp_roll") -> bytes:
    return (
        name.encode()
        + b"\0"
        + bytes((codec.PARAM_FLOAT, 1, 3, 0))
        + b"0.000\0"
        + b"3.000\0"
        + b"0.250\0"
    )


def test_client_reads_identity_status_and_parameter_values() -> None:
    hello = (
        bytes((1,))
        + b"aerialkit-f405\0"
        + (46).to_bytes(2, "little")
        + b"2 changed\0"
        + (0x2FFF).to_bytes(4, "little")
        + (0x12345678).to_bytes(4, "little")
    )
    responses = {
        codec.HELLO: hello,
        codec.STATUS: status_payload(),
        codec.PARAM_GET: b"\0" + b"rate_kp_roll\0" + b"0.250\0",
    }
    client = AkClient(RoutingTransport(responses))
    identity = client.hello()
    assert identity.product == "aerialkit-f405"
    assert identity.parameter_count == 46
    assert "PARAM_INFO" in identity.feature_names
    status = client.status()
    assert status.roll_deg == 10.0
    assert status.motors == (10, 20, 30, 40)
    parameter = client.get_parameter(0)
    assert parameter.name == "rate_kp_roll"
    assert parameter.value == "0.250"


class ReplyTransport(MemoryTransport):
    def __init__(
        self,
        reply_for: dict[int, bytes],
        *,
        extra_after_write: list[bytes] | None = None,
    ) -> None:
        super().__init__(timeout=0)
        self.reply_for = reply_for
        self.requests: list[tuple[int, bytes]] = []
        self.extra_after_write = list(extra_after_write or [])

    def _write(self, data: bytes) -> int:
        command = data[3]
        payload = data[5:-2]
        self.requests.append((command, payload))
        if command in self.reply_for:
            self.feed(build(command | 0x80, self.reply_for[command]))
        if self.extra_after_write:
            self.feed(self.extra_after_write.pop(0))
        return len(data)


def test_all_command_families_are_typed_and_reachable() -> None:
    calibration = (
        bytes((0, CALIBRATE_GYRO, 0, 0xFF, 0))
        + (0).to_bytes(4, "little")
        + (0).to_bytes(4, "little")
        + b"".join((0).to_bytes(4, "little", signed=True) for _ in range(6))
    )
    mission = (
        bytes((0, MISSION_START, 1, 1, 1, 0, 7))
        + (0).to_bytes(2, "little")
        + (1).to_bytes(2, "little")
        + (0).to_bytes(2, "little")
        + (10_000).to_bytes(4, "little", signed=True)
    )
    perf = bytearray(59)
    perf[0] = 0
    struct.pack_into("<I", perf, 1, 1000)
    struct.pack_into("<I", perf, 5, 990)
    struct.pack_into("<H", perf, 9, 1000)
    struct.pack_into("<III", perf, 11, 1000, 999, 1100)
    struct.pack_into("<I", perf, 23, 9)
    struct.pack_into("<HHH", perf, 27, 5, 10, 40)
    struct.pack_into("<I", perf, 33, 3)
    struct.pack_into("<HHHHH", perf, 37, 10, 20, 30, 40, 50)
    struct.pack_into("<HHHHH", perf, 47, 100, 200, 300, 400, 500)
    struct.pack_into("<H", perf, 57, 123)
    replies = {
        codec.PARAM_SET: b"\0ok\0",
        codec.PARAM_SAVE: b"\0saved\0",
        codec.PARAM_DEFAULT: b"\0reset\0",
        codec.LOG_INFO: b"\0" + (3).to_bytes(2, "little"),
        codec.LOG_GET: b"\0" + bytes(87),
        codec.LOG_SOURCE: b"\0\x00" + (4).to_bytes(2, "little"),
        codec.PARAM_INFO: b"\0\x00\x01" + parameter_entry(),
        codec.PARAM_HELP: b"\0\x00\x00\x00\x05\x00\x05help!",
        codec.RC_CHANNELS: b"\x01",
        codec.SENSOR_INFO: b"\0\x00\x00",
        codec.OUTPUT_INFO: b"\0\x01\x01\x00\x0f" + bytes((0, 0, 0, 0, 0, 0, 0)),
        codec.OUTPUT_TEST: bytes((0, OUTPUT_TEST_HOLD, 0, 0, 10)) + (100).to_bytes(2, "little"),
        codec.LOG_STREAM: b"\0\x00\x00\x00\x01\x00\x0a",
        codec.PREFLIGHT: b"\x02",
        codec.MOTOR_TELEMETRY: b"\x01",
        codec.MISSION: mission,
        codec.CALIBRATE: calibration,
        codec.PERF: perf,
    }
    transport = ReplyTransport(replies)
    client = AkClient(transport)
    assert client.set_parameter(0, "1.0", allow_write=True).ok
    assert client.save_parameters(allow_write=True).ok
    assert client.default_parameters(index=0, allow_write=True).ok
    assert client.log_info().count == 3
    assert len(client.log_record(0).values) > 10
    assert client.log_source(0).count == 4
    assert client.parameter_info(0, 1)[0].name == "rate_kp_roll"
    assert client.parameter_help(0) == "help!"
    assert client.rc_channels().status_name == "no receiver input"
    assert not client.sensor_info(0).present
    assert client.output_info().count == 1
    assert client.output_test(allow_write=True).level_pct == 10
    assert client.start_log_stream(0, 0, 1, 10) == (0, 0, 1, 10)
    assert client.preflight() == ()
    assert client.motor_telemetry().status_name == "no motor telemetry"
    assert client.mission(MISSION_START, allow_write=True).active
    assert client.calibrate(CALIBRATE_GYRO, allow_write=True).verb == CALIBRATE_GYRO
    assert client.performance().loops == 1000


def test_mutations_are_refused_before_writing() -> None:
    transport = MemoryTransport()
    client = AkClient(transport)
    for operation in (
        lambda: client.set_parameter(0, "1"),
        lambda: client.save_parameters(),
        lambda: client.default_parameters(),
        lambda: client.output_test(),
        lambda: client.mission(MISSION_START),
        lambda: client.calibrate(CALIBRATE_GYRO),
    ):
        with pytest.raises(AkWriteRefused):
            operation()
    assert transport.drain_tx() == b""


def test_telemetry_subscription_and_stream_frame_are_typed() -> None:
    telemetry = struct.pack("<i", 1234) + status_payload()
    transport = ReplyTransport(
        {codec.TELEMETRY: b"\x0a"},
        extra_after_write=[build(codec.TELEMETRY, telemetry)],
    )
    client = AkClient(transport)
    assert client.telemetry_subscribe(10) == 10
    frame = client.telemetry_next()
    assert frame.uptime_ms == 1234
    assert frame.status.gps_satellites == 12


def test_protocol_error_is_typed() -> None:
    transport = ReplyTransport({codec.HELLO: b"\x01bad"})
    client = AkClient(transport, timeout=0.01)
    with pytest.raises((AkProtocolError, OSError)):
        client.hello()


def test_configurator_capture_is_consumed_by_the_typed_client() -> None:
    fixture = json.loads(
        (Path(__file__).parent / "apps/configurator/tests/fixtures/aerialkit.json").read_text()
    )["frames"]

    def payload(name: str) -> bytes:
        return bytes.fromhex(fixture[name]["reply"])[5:-2]

    client = AkClient(
        RoutingTransport(
            {
                codec.HELLO: payload("hello"),
                codec.STATUS: payload("status"),
                codec.PARAM_GET: payload("param_get_0"),
            }
        )
    )
    assert client.hello().product == "aerialkit-f405"
    assert client.status().flight_state == 1
    assert client.get_parameter(0).value == "0.250"
