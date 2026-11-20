from __future__ import annotations

import struct
import threading
import time

import pytest

from aerial_kit.comms import (
    BackendArmSequencer,
    CrsfChannelMap,
    CrsfDeviceInfo,
    CrsfChunkLoss,
    CrsfFrame,
    CrsfLink,
    CrsfParameterClient,
    CrsfParser,
    CrsfRadio,
    CrsfWriteRefused,
    LinkStatisticsTelemetry,
    MemoryTransport,
    RcSticks,
    make_frame,
    binding_uid,
    format_uid,
    parse_uid,
    decode_uid_response,
    encode_binding_phrase_write,
    encode_uid_request,
    encode_uid_write,
    uid_from_frame_bytes,
    loopback_pair,
    parse_device_info,
    parse_parameter_record,
    unpack_channels,
)


def test_arm_sequencer_holds_low_then_passes_sticks() -> None:
    sequencer = BackendArmSequencer(arm_hold_sec=1.0, throttle_min=0.0)
    sticks = RcSticks(0.2, -0.3, 0.4, 0.6)
    held, enabled = sequencer.update(True, sticks, 10.0)
    assert enabled and held == RcSticks(0.0, 0.0, 0.0, 0.0)
    assert sequencer.update(True, sticks, 10.9)[0].throttle == 0.0
    assert sequencer.update(True, sticks, 11.0) == (sticks, True)
    assert sequencer.update(False, sticks, 12.0) == (
        RcSticks(0.0, 0.0, 0.0, 0.0),
        False,
    )


def test_channel_map_round_trip() -> None:
    channel_map = CrsfChannelMap()
    sticks = RcSticks(0.5, -0.25, 0.1, 0.75)
    channels = channel_map.encode(sticks, enabled=True)
    decoded = channel_map.decode_sticks(channels)
    assert decoded.arm_request
    assert decoded.angle_mode
    assert decoded.sticks.roll == pytest.approx(sticks.roll, abs=0.002)
    assert decoded.sticks.pitch == pytest.approx(sticks.pitch, abs=0.002)
    assert decoded.sticks.yaw == pytest.approx(sticks.yaw, abs=0.002)
    assert decoded.sticks.throttle == pytest.approx(sticks.throttle, abs=0.002)


def test_radio_sends_arm_sequence_and_fixed_rate_frames() -> None:
    transport = MemoryTransport()
    radio = CrsfRadio(
        CrsfLink(transport),
        send_rate_hz=100.0,
        arm_hold_sec=0.05,
    )
    radio.set_sticks(RcSticks(0.25, 0.0, 0.0, 0.5))
    radio.arm()
    radio.start()
    time.sleep(0.12)
    radio.stop()
    raw = transport.drain_tx()
    frames = CrsfParser().feed(raw)
    assert len(frames) >= 8
    first = unpack_channels(frames[0].payload)
    last = unpack_channels(frames[-1].payload)
    assert first[2] == 172
    assert last[2] > first[2]
    radio.close()


def test_radio_telemetry_callback_and_link_quality() -> None:
    transport = MemoryTransport()
    radio = CrsfRadio(CrsfLink(transport))
    received: list[object] = []
    radio.add_telemetry_callback(received.append)
    payload = bytes((90, 92, 88, 7, 1, 2, 3, 80, 70, 4))
    radio._dispatch(CrsfFrame(0xEA, 0x14, payload))
    assert isinstance(received[0], LinkStatisticsTelemetry)
    assert radio.link_quality is not None
    assert radio.link_quality.uplink_link_quality == 88
    radio.close()


def _device_info_frame() -> bytes:
    payload = bytes((0xEA, 0xEE)) + b"RadioMaster RP3\0"
    payload += 0x454C5253.to_bytes(4, "big")
    payload += (0).to_bytes(4, "big")
    payload += 0x00030000.to_bytes(4, "big")
    payload += bytes((7, 0))
    return make_frame(0x29, payload)


def test_device_info_matches_expresslrs_layout() -> None:
    frame = CrsfParser().feed(_device_info_frame())[0]
    info = parse_device_info(frame)
    assert info == CrsfDeviceInfo(
        destination=0xEA,
        origin=0xEE,
        name="RadioMaster RP3",
        serial=0x454C5253,
        hardware_version=0,
        software_version=0x00030000,
        field_count=7,
        parameter_version=0,
    )


def test_device_ping_returns_typed_info() -> None:
    transport = MemoryTransport()
    client = CrsfParameterClient(
        CrsfLink(transport), destination=0xEE, origin=0xEA, timeout=0.2
    )
    original_write = transport.write

    def write_and_answer(data: bytes) -> int:
        result = original_write(data)
        request = CrsfParser().feed(transport.drain_tx())[0]
        assert request.frame_type == 0x28
        transport.feed(_device_info_frame())
        return result

    transport.write = write_and_answer  # type: ignore[method-assign]
    info = client.ping()
    assert info.name == "RadioMaster RP3"
    assert info.origin == 0xEE


def _entry(
    field_id: int,
    parent: int,
    type_id: int,
    name: str,
    value: bytes,
) -> bytes:
    return (
        bytes((field_id, 0, parent, type_id))
        + name.encode()
        + b"\0"
        + value
    )


def test_parameter_record_parsers_cover_numeric_selection_folder_and_command() -> None:
    numeric = parse_parameter_record(
        _entry(2, 0, 3, "power", struct.pack(">hhhh", 100, 0, 1000, 250) + b"mW\0")
    )
    assert numeric.value == 100
    assert numeric.maximum == 1000
    assert numeric.units == "mW"

    selection = parse_parameter_record(
        _entry(3, 0, 9, "rate", b"50Hz;150Hz;250Hz\0" + bytes((1, 0, 2, 1)) + b"\0")
    )
    assert selection.options == ("50Hz", "150Hz", "250Hz")
    assert selection.value == 1

    folder = parse_parameter_record(_entry(4, 0, 11, "TX", bytes((2, 3, 0xFF))))
    assert folder.children == (2, 3)

    command = parse_parameter_record(_entry(5, 0, 13, "Bind", b"\x01\x14Bound\0"))
    assert command.step == 1
    assert command.timeout_ms == 200
    assert command.info == "Bound"


def _chunk_entry(field_id: int, record: bytes, chunk_size: int) -> list[bytes]:
    body = record[2:]
    chunks = [
        body[index:index + chunk_size]
        for index in range(0, len(body), chunk_size)
    ]
    output = []
    for index, chunk in enumerate(chunks):
        remaining = len(chunks) - index - 1
        output.append(bytes((field_id, remaining)) + chunk)
    return output


def _responder(field_id: int, record: bytes) -> tuple[MemoryTransport, MemoryTransport, threading.Event]:
    left, right = loopback_pair(timeout=0.01)
    stop = threading.Event()
    chunks = _chunk_entry(field_id, record, 6)

    def run() -> None:
        parser = CrsfParser()
        while not stop.is_set():
            frames = parser.feed(right.read(512, timeout=0.01))
            for frame in frames:
                if frame.frame_type != 0x2C:
                    continue
                requested = frame.payload[3]
                if requested >= len(chunks):
                    continue
                response = make_frame(
                    0x2B,
                    bytes((0xEA, 0xEE)) + chunks[requested],
                )
                right.write(response)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    stop._thread = thread  # type: ignore[attr-defined]
    return left, right, stop


def test_parameter_chunks_reassemble_without_radio_framework() -> None:
    record = _entry(
        6,
        0,
        10,
        "version",
        b"a-long-parameter-value-that-needs-more-than-one-frame\0",
    )
    left, right, stop = _responder(6, record)
    client = CrsfParameterClient(
        CrsfLink(left), destination=0xEE, origin=0xEA, timeout=0.2
    )
    try:
        parameter = client.read_parameter(6)
        assert parameter.name == "version"
        assert parameter.value == "a-long-parameter-value-that-needs-more-than-one-frame"
    finally:
        stop.set()
        stop._thread.join(1.0)  # type: ignore[attr-defined]
        left.close()
        right.close()


def test_parameter_chunks_reassemble_when_responses_arrive_out_of_order() -> None:
    record = _entry(
        9,
        0,
        10,
        "version",
        b"out-of-order-chunk-data-that-spans-three-frames\0",
    )
    chunks = _chunk_entry(9, record, (len(record) + 2) // 3)
    assert len(chunks) == 3
    transport = MemoryTransport()
    parser = CrsfParser()
    responses = iter((chunks[0], chunks[2], chunks[1]))
    original_write = transport.write

    def answer_requests(data: bytes) -> int:
        result = original_write(data)
        for request in parser.feed(transport.drain_tx()):
            if request.frame_type == 0x2C and request.payload[2] == 9:
                transport.feed(
                    make_frame(0x2B, bytes((0xEA, 0xEE)) + next(responses))
                )
        return result

    transport.write = answer_requests  # type: ignore[method-assign]
    client = CrsfParameterClient(
        CrsfLink(transport), destination=0xEE, origin=0xEA, timeout=0.05
    )
    parameter = client.read_parameter(9)
    assert parameter.value == "out-of-order-chunk-data-that-spans-three-frames"


def test_parameter_chunk_loss_is_reported_with_its_index() -> None:
    record = _entry(
        10,
        0,
        10,
        "version",
        b"this-record-has-a-lost-middle-chunk\0",
    )
    chunks = _chunk_entry(10, record, (len(record) + 3) // 4)
    assert len(chunks) >= 3
    transport = MemoryTransport()
    parser = CrsfParser()
    original_write = transport.write

    def answer_without_middle_chunk(data: bytes) -> int:
        result = original_write(data)
        for request in parser.feed(transport.drain_tx()):
            if request.frame_type != 0x2C or request.payload[2] != 10:
                continue
            requested = request.payload[3]
            if requested == 1:
                continue
            transport.feed(
                make_frame(0x2B, bytes((0xEA, 0xEE)) + chunks[requested])
            )
        return result

    transport.write = answer_without_middle_chunk  # type: ignore[method-assign]
    client = CrsfParameterClient(
        CrsfLink(transport), destination=0xEE, origin=0xEA, timeout=0.01
    )
    with pytest.raises(
        CrsfChunkLoss, match=rf"missing chunk 1 of {len(chunks)}"
    ):
        client.read_parameter(10)


def test_parameter_tree_walks_folder_children() -> None:
    records = {
        0: _entry(0, 0, 11, "Radio", b"\x01\x02\xFF"),
        1: _entry(1, 0, 0, "power", bytes((3, 0, 8, 3)) + b"\0"),
        2: _entry(2, 0, 9, "rate", b"50Hz;150Hz\0" + bytes((1, 0, 1, 1)) + b"\0"),
    }
    left, right = loopback_pair(timeout=0.01)
    stop = threading.Event()

    def run() -> None:
        parser = CrsfParser()
        while not stop.is_set():
            for frame in parser.feed(right.read(512, timeout=0.01)):
                if frame.frame_type != 0x2C:
                    continue
                field_id = frame.payload[2]
                chunk = _chunk_entry(field_id, records[field_id], 64)[frame.payload[3]]
                right.write(make_frame(0x2B, bytes((0xEA, 0xEE)) + chunk))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    client = CrsfParameterClient(
        CrsfLink(left), destination=0xEE, origin=0xEA, timeout=0.2
    )
    try:
        tree = client.read_parameter_tree(root=0)
        assert set(tree) == {0, 1, 2}
        assert tree[0].children == (1, 2)
        assert tree[1].value == 3
        assert tree[2].options[1] == "150Hz"
    finally:
        stop.set()
        thread.join(1.0)
        left.close()
        right.close()


def test_parameter_writes_and_bind_are_refused_without_opt_in() -> None:
    transport = MemoryTransport()
    client = CrsfParameterClient(CrsfLink(transport), destination=0xEE, origin=0xEA)
    with pytest.raises(CrsfWriteRefused):
        client.write_parameter(1, 1)
    with pytest.raises(CrsfWriteRefused):
        client.bind()
    with pytest.raises(CrsfWriteRefused):
        client.write_uid(b"\x01\x02\x03\x04\x05\x06")
    with pytest.raises(CrsfWriteRefused):
        client.write_binding_phrase("test")
    assert transport.drain_tx() == b""


def test_uid_request_matches_the_hardware_rx_frame() -> None:
    frame = encode_uid_request(0xEC)
    assert frame.encode().hex() == "c80c7aecEF50002D000100001a85".lower()


def test_uid_response_decodes_the_hardware_rx_payload() -> None:
    raw = bytes.fromhex("c80f7befec30072d000000000000002ad1")
    assert uid_from_frame_bytes(raw) == bytes(6)
    assert format_uid(uid_from_frame_bytes(raw)) == "00:00:00:00:00:00"
    frame = CrsfParser().feed(raw)[0]
    assert decode_uid_response(frame) == bytes(6)


def test_uid_and_phrase_writes_have_the_expresslrs_msp_layout() -> None:
    uid_frame = encode_uid_write(0xEE, parse_uid("AA:BB:CC:DD:EE:FF"))
    assert uid_frame.frame_type == 0x7C
    assert uid_frame.encode().hex() == "c80f7ceeef30072d00aabbccddeeff3b65"

    phrase_frame = encode_binding_phrase_write(0xEE, "test")
    assert phrase_frame.frame_type == 0x7C
    assert phrase_frame.encode().hex() == "c80d7ceeef30052d01746573743fcc"
    assert binding_uid("test").hex() == "4f04fd822155"


def test_uid_read_uses_origin_lua_and_accepts_hardware_response() -> None:
    transport = MemoryTransport()
    client = CrsfParameterClient(
        CrsfLink(transport), destination=0xEC, origin=0xEF, timeout=0.05
    )
    original_write = transport.write

    def answer_uid(data: bytes) -> int:
        result = original_write(data)
        for request in CrsfParser().feed(transport.drain_tx()):
            assert request.frame_type == 0x7A
            transport.feed(
                bytes.fromhex("c80f7befec30072d000000000000002ad1")
            )
        return result

    transport.write = answer_uid  # type: ignore[method-assign]
    assert client.read_uid(attempts=1, timeout=0.05) == bytes(6)


def test_bind_with_opt_in_writes_the_expresslrs_command_frame() -> None:
    transport = MemoryTransport()
    client = CrsfParameterClient(CrsfLink(transport), destination=0xEE, origin=0xEA)
    client.bind(allow_write=True)
    frame = CrsfParser().feed(transport.drain_tx())[0]
    assert frame.frame_type == 0x32
    assert frame.payload == bytes((0xEE, 0xEA, 0x10, 0x01))


def test_parameter_write_requires_opt_in_and_encodes_big_endian_value() -> None:
    transport = MemoryTransport()
    client = CrsfParameterClient(CrsfLink(transport), destination=0xEE, origin=0xEA)
    current = 100
    parser = CrsfParser()

    def record() -> list[bytes]:
        body = _entry(
            8,
            0,
            2,
            "power",
            struct.pack(">HHHH", current, 0, 1000, 250) + b"mW\0",
        )
        return _chunk_entry(8, body, 64)

    original_write = transport.write

    def write_and_answer(data: bytes) -> int:
        nonlocal current
        result = original_write(data)
        for request in parser.feed(transport.drain_tx()):
            if request.frame_type == 0x2C:
                transport.feed(make_frame(0x2B, bytes((0xEA, 0xEE)) + record()[0]))
            elif request.frame_type == 0x2D:
                current = struct.unpack_from(">H", request.payload, 3)[0]
                transport.feed(make_frame(0x2B, bytes((0xEA, 0xEE)) + record()[0]))
        return result

    transport.write = write_and_answer  # type: ignore[method-assign]
    updated = client.write_parameter(8, 500, allow_write=True)
    assert updated is not None
    assert updated.value == 500
