from __future__ import annotations

import os
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from aerial_kit.comms import (
    AKP_TUNNEL_DEFAULT_CHUNK_SIZE,
    AKP_TUNNEL_FIRST,
    AKP_TUNNEL_LAST,
    AKP_TUNNEL_MAGIC,
    AKP_TUNNEL_SEQUENCE_MASK,
    AkpTunnelDuplicateConflict,
    AkpTunnelError,
    AkpTunnelIncomplete,
    AkpTunnelLengthMismatch,
    AkpTunnelMetadataConflict,
    AkpTunnelReassembler,
    AkpTunnelSegment,
    AkpTunnelSequenceGap,
    AkpTunnelSessionConflict,
    split_akp_frame,
)
from aerial_kit.comms.crsf.codec import CrsfLink, CrsfParser
from aerial_kit.comms.crsf.codec import CrsfFrame, crc8_dvb_s2
from aerial_kit.comms.crsf.msp_tunnel import (
    AKP_MSP_FUNCTION,
    AKP_MSP_REQUEST_CHUNK,
    AKP_MSP_RESPONSE_CHUNK,
    CrsfAkpTunnel,
    decode_msp_request,
    decode_msp_response,
    encode_msp_request,
    encode_msp_response,
)
from aerial_kit.comms.transport import MemoryTransport
from aerial_kit.comms.akp.client import AkClient
from aerial_kit.comms.akp import codec as akp_codec
from aerial_kit.comms.akp.codec import HELLO

ROOT = Path(__file__).resolve().parents[1]


def reassemble(
    payload: bytes,
    segments: tuple[AkpTunnelSegment, ...],
) -> bytes:
    receiver = AkpTunnelReassembler()
    result = None
    for segment in segments:
        result = receiver.feed(segment)
    assert result == payload
    assert not receiver.active
    return result


def test_split_and_reassemble_in_order() -> None:
    payload = bytes(range(23))
    segments = split_akp_frame(payload, session=7)
    assert len(segments) == 8
    assert [segment.sequence for segment in segments] == list(range(8))
    assert segments[0].first and segments[0].seq_flags & AKP_TUNNEL_FIRST
    assert segments[-1].last and segments[-1].seq_flags & AKP_TUNNEL_LAST
    assert all(
        segment.session == 7 and segment.total_length == len(payload)
        for segment in segments
    )
    assert reassemble(payload, segments) == payload


def test_default_chunk_size_matches_uplink_msp_budget() -> None:
    segments = split_akp_frame(b"abcdefghij", session=3)
    assert AKP_TUNNEL_DEFAULT_CHUNK_SIZE == 3
    assert [segment.chunk for segment in segments] == [
        b"abc",
        b"def",
        b"ghi",
        b"j",
    ]


def test_empty_payload_is_one_first_and_last_segment() -> None:
    segments = split_akp_frame(b"", session=0)
    assert len(segments) == 1
    assert segments[0].first and segments[0].last
    assert segments[0].sequence == 0
    assert segments[0].chunk == b""
    assert AkpTunnelReassembler().feed(segments[0]) == b""


def test_encoded_header_is_stable_and_decodes_back() -> None:
    segment = AkpTunnelSegment(
        session=0x42,
        sequence=3,
        first=False,
        last=True,
        total_length=0x1234,
        chunk=b"data",
    )
    encoded = segment.encode()
    assert encoded == bytes.fromhex("a142831234") + b"data"
    assert AkpTunnelSegment.decode(encoded) == segment
    assert segment.seq_flags & AKP_TUNNEL_SEQUENCE_MASK == 3


def test_reassembler_accepts_bytes_and_reordered_segments() -> None:
    payload = b"reordered AKP payload"
    segments = split_akp_frame(payload, session=11, chunk_size=5)
    receiver = AkpTunnelReassembler()
    result = None
    for segment in reversed(segments):
        result = receiver.feed(segment.encode())
    assert result == payload
    assert not receiver.active


def test_partial_stream_is_reported_incomplete_until_all_segments_arrive() -> None:
    segments = split_akp_frame(b"partial payload", session=1, chunk_size=4)
    receiver = AkpTunnelReassembler()
    assert receiver.feed(segments[0]) is None
    assert receiver.feed(segments[1]) is None
    assert receiver.feed(segments[2]) is None
    assert receiver.active
    assert receiver.received_sequences == (0, 1, 2)


def test_identical_duplicate_is_deduplicated() -> None:
    segments = split_akp_frame(b"duplicate", session=2, chunk_size=4)
    receiver = AkpTunnelReassembler()
    assert receiver.feed(segments[0]) is None
    assert receiver.feed(segments[0]) is None
    for segment in segments[1:]:
        result = receiver.feed(segment)
    assert result == b"duplicate"


def test_conflicting_duplicate_is_rejected() -> None:
    receiver = AkpTunnelReassembler()
    first = AkpTunnelSegment(1, 0, True, False, 8, b"abcd")
    receiver.feed(first)
    with pytest.raises(AkpTunnelDuplicateConflict):
        receiver.feed(AkpTunnelSegment(1, 0, True, False, 8, b"ABCD"))


def test_session_conflict_is_rejected_while_transfer_is_active() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 4, b"ab"))
    with pytest.raises(AkpTunnelSessionConflict):
        receiver.feed(AkpTunnelSegment(2, 1, False, True, 2, b"cd"))


def test_total_length_conflict_is_rejected() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 8, b"ab"))
    with pytest.raises(AkpTunnelMetadataConflict):
        receiver.feed(AkpTunnelSegment(1, 1, False, True, 9, b"cd"))


def test_missing_first_segment_is_named_precisely() -> None:
    receiver = AkpTunnelReassembler()
    assert receiver.feed(AkpTunnelSegment(1, 1, False, True, 4, b"cd")) is None
    with pytest.raises(AkpTunnelIncomplete, match="first"):
        receiver.finish()


def test_missing_final_segment_is_named_precisely() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 8, b"ab"))
    with pytest.raises(AkpTunnelIncomplete, match="final"):
        receiver.finish()


def test_gap_after_final_is_named_precisely() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 9, b"aa"))
    receiver.feed(AkpTunnelSegment(1, 2, False, True, 9, b"cc"))
    with pytest.raises(AkpTunnelSequenceGap, match="1"):
        receiver.finish()


def test_two_different_final_segments_are_rejected() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 6, b"ab"))
    receiver.feed(AkpTunnelSegment(1, 2, False, True, 6, b"ef"))
    with pytest.raises(AkpTunnelMetadataConflict, match="final"):
        receiver.feed(AkpTunnelSegment(1, 3, False, True, 6, b"gh"))


def test_segment_after_final_is_rejected() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 6, b"ab"))
    receiver.feed(AkpTunnelSegment(1, 2, False, True, 6, b"ef"))
    with pytest.raises(AkpTunnelMetadataConflict, match="follows final"):
        receiver.feed(AkpTunnelSegment(1, 3, False, False, 6, b"gh"))


def test_length_mismatch_is_rejected() -> None:
    receiver = AkpTunnelReassembler()
    receiver.feed(AkpTunnelSegment(1, 0, True, False, 8, b"ab"))
    with pytest.raises(AkpTunnelLengthMismatch):
        receiver.feed(AkpTunnelSegment(1, 1, False, True, 8, b"cd"))


@pytest.mark.parametrize("payload", [
    b"",
    b"\xA1",
    b"\x00\x01\x02",
    b"\xff" * 257,
])
def test_malformed_headers_are_typed(payload: bytes) -> None:
    with pytest.raises(AkpTunnelError):
        AkpTunnelSegment.decode(payload)


def test_bad_magic_is_typed() -> None:
    with pytest.raises(AkpTunnelError, match="magic"):
        AkpTunnelSegment.decode(bytes.fromhex("a042c00004") + b"data")


def test_first_flag_must_be_sequence_zero() -> None:
    with pytest.raises(AkpTunnelError, match="sequence zero"):
        AkpTunnelSegment(1, 1, True, False, 4, b"ab")


def test_sequence_budget_and_payload_limits_are_enforced() -> None:
    with pytest.raises(AkpTunnelError, match="sequence"):
        split_akp_frame(b"x" * 65, session=1, chunk_size=1)
    with pytest.raises(AkpTunnelError, match="limit"):
        split_akp_frame(b"x" * 65_536, session=1, chunk_size=1024)
    with pytest.raises(AkpTunnelMetadataConflict, match="receiver limit"):
        AkpTunnelReassembler(max_payload=10).feed(
            AkpTunnelSegment(1, 0, True, True, 11, b"x" * 11)
        )


def test_invalid_sender_configuration_is_typed() -> None:
    with pytest.raises(AkpTunnelError, match="positive"):
        split_akp_frame(b"x", session=1, chunk_size=0)
    with pytest.raises(AkpTunnelError, match="session"):
        split_akp_frame(b"x", session=256)


def test_large_payload_reassembles_at_limit() -> None:
    payload = bytes(range(256)) * 255
    assert len(payload) == 65_280
    segments = split_akp_frame(payload, session=9, chunk_size=1022)
    assert len(segments) <= 64
    assert reassemble(payload, segments) == payload


def test_msp_envelopes_round_trip_tunnel_bytes() -> None:
    payload = b"\xa1\x07\x40\x04\x00abcd"
    request = encode_msp_request(payload)
    assert request.frame_type == 0x7A
    assert request.payload[0] == 0xC8
    assert request.payload[2] == 0x50
    assert decode_msp_request(request) == payload

    response = encode_msp_response(payload)
    assert response.frame_type == 0x7B
    assert response.payload[0] == 0xEA
    assert response.payload[2] == 0x30
    assert decode_msp_response(response) == payload
    assert AKP_MSP_FUNCTION == 0x4B

    write_body = bytes((len(payload) + 1, AKP_MSP_FUNCTION)) + payload
    write = CrsfFrame(
        0xC8,
        0x7C,
        bytes((0xC8, 0xEA, 0x30))
        + write_body
        + bytes((crc8_dvb_s2(write_body),)),
    )
    assert decode_msp_request(write) == payload


def test_msp_envelope_rejects_wrong_function_and_bad_crc() -> None:
    request = encode_msp_request(b"payload", function=0x4C)
    with pytest.raises(Exception, match="not AKP"):
        decode_msp_request(request)
    response = encode_msp_response(b"payload")
    payload = bytearray(response.payload)
    payload[-1] ^= 0x01
    with pytest.raises(Exception, match="CRC"):
        decode_msp_response(
            type(response)(
                response.address,
                response.frame_type,
                bytes(payload),
            )
        )


class FakeRadio:
    def __init__(self) -> None:
        self.transport = MemoryTransport()
        self.link = CrsfLink(self.transport)
        self.callbacks = []

    def add_frame_callback(self, callback) -> None:
        self.callbacks.append(callback)

    def remove_frame_callback(self, callback) -> None:
        self.callbacks.remove(callback)

    def emit(self, frame) -> None:
        for callback in tuple(self.callbacks):
            callback(frame)


class AkpResponderTransport(MemoryTransport):
    def __init__(self, radio: FakeRadio) -> None:
        super().__init__()
        self.radio = radio
        self.parser = CrsfParser()
        self.reassembler = AkpTunnelReassembler()

    def _write(self, data: bytes) -> int:
        for frame in self.parser.feed(data):
            if frame.frame_type not in (0x7A, 0x7C):
                continue
            segment = AkpTunnelSegment.decode(decode_msp_request(frame))
            request = self.reassembler.feed(segment)
            if request is None:
                continue
            payload = (
                b"\x01aerialkit-test\x00"
                + b"\x01\x00"
                + b"0\x00"
                + (0).to_bytes(4, "little")
                + (0).to_bytes(4, "little")
            )
            body = bytes((1, HELLO | 0x80, len(payload))) + payload
            response = (
                b"\xaa\x55"
                + body
                + struct.pack("<H", akp_codec.crc16(body))
            )
            for out in split_akp_frame(
                response,
                session=segment.session,
                chunk_size=AKP_MSP_RESPONSE_CHUNK,
            ):
                self.radio.emit(encode_msp_response(out.encode()))
        return len(data)


def test_crsf_akp_tunnel_sends_segmented_requests_and_reassembles_responses() -> None:
    radio = FakeRadio()
    tunnel = CrsfAkpTunnel(radio, send_interval_s=0)
    request = b"AKP request payload " * 8
    tunnel.write(request)
    frames = CrsfParser().feed(radio.transport.drain_tx())
    assert len(frames) == (len(request) + AKP_MSP_REQUEST_CHUNK - 1) // AKP_MSP_REQUEST_CHUNK
    sender = AkpTunnelReassembler()
    assembled = None
    for frame in frames:
        segment = decode_msp_request(frame)
        assembled = sender.feed(segment)
    assert assembled == request
    assert AKP_MSP_REQUEST_CHUNK == 3
    assert len(frames) >= len(request) // AKP_MSP_REQUEST_CHUNK

    response = b"AKP response payload " * 9
    session = decode_msp_request(frames[0])[1]
    segments = split_akp_frame(response, session=session, chunk_size=AKP_MSP_RESPONSE_CHUNK)
    for segment in reversed(segments):
        radio.emit(encode_msp_response(segment.encode()))
    assert tunnel.read(len(response), timeout=0.1) == response
    assert tunnel.received_frames == 1


def test_ak_client_hello_over_the_crsf_tunnel() -> None:
    radio = FakeRadio()
    transport = AkpResponderTransport(radio)
    radio.transport = transport
    radio.link = CrsfLink(transport)
    tunnel = CrsfAkpTunnel(radio, send_interval_s=0)
    client = AkClient(tunnel, timeout=0.2)
    hello = client.hello()
    assert hello.product == "aerialkit-test"
    assert hello.parameter_count == 1
    client.close()


def test_akp_radio_route_against_host_built_firmware(tmp_path: Path) -> None:
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no C compiler")
    firmware = ROOT / "firmware"
    output = tmp_path / "akp-radio"
    command = [
        compiler,
        "-std=c11",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{firmware / 'src/core'}",
        f"-I{firmware / 'src/core/flight'}",
        str(firmware / "tools/host/test_akp_radio.c"),
        str(firmware / "src/core/flight/ak_crsf.c"),
        str(firmware / "src/core/ak_crsf_tunnel.c"),
        str(firmware / "src/core/ak_proto.c"),
        str(firmware / "src/core/ak_params.c"),
        str(firmware / "src/core/ak_text.c"),
        "-o",
        str(output),
    ]
    subprocess.run(command, check=True, cwd=ROOT, env=os.environ.copy())
    result = subprocess.run(
        [str(output)], check=True, cwd=ROOT, capture_output=True, text=True
    )
    assert "akp-radio: ok" in result.stdout
