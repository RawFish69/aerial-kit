from __future__ import annotations

import struct

from aerial_kit.comms.akp import codec
from aerial_kit.comms.crsf import (
    AKP_MSP_RESPONSE_CHUNK,
    AkpTunnelReassembler,
    decode_msp_response,
    encode_msp_request,
    split_akp_frame,
)
from hw_bridge.akp_sim import AkpSimDevice


def exchange(command: int, payload: bytes = b"") -> tuple[str, bytes]:
    request = codec.build(command, payload)
    device = AkpSimDevice()
    replies = []
    for segment in split_akp_frame(request, session=9, chunk_size=3):
        replies.extend(device.feed(encode_msp_request(segment.encode())))
    assert replies
    receiver = AkpTunnelReassembler()
    response = None
    for frame in replies:
        response = receiver.feed(decode_msp_response(frame))
    assert response is not None
    assert response[:2] == bytes((codec.SYNC1, codec.SYNC2))
    length = response[4]
    assert len(response) == length + 7
    assert struct.unpack_from("<H", response, 5 + length)[0] == codec.crc16(
        response[2:5 + length]
    )
    return chr(response[3]), response[5:5 + length]


def test_akp_sim_hello_status_and_parameters() -> None:
    command, payload = exchange(codec.HELLO)
    hello = codec.parse_hello(payload)
    assert command == chr(codec.HELLO | codec.RESPONSE_BIT)
    assert hello["product"] == "aerialkit-sim-radio"
    assert hello["count"] == 3

    command, payload = exchange(codec.STATUS)
    assert command == chr(codec.STATUS | codec.RESPONSE_BIT)
    status = codec.parse_status(payload)
    assert status["link_live"] == 1
    assert status["gps_satellites"] == 12

    command, payload = exchange(codec.PARAM_GET, b"\x00")
    assert command == chr(codec.PARAM_GET | codec.RESPONSE_BIT)
    assert payload.startswith(b"\x00rate_kp_roll\x00")


def test_akp_sim_rejects_unknown_read_without_writing() -> None:
    command, payload = exchange(codec.MISSION, b"\x09")
    assert command == chr(codec.MISSION | codec.RESPONSE_BIT)
    assert payload == b"\x7f"


def test_new_first_segment_preempts_a_stale_session() -> None:
    device = AkpSimDevice()
    old = split_akp_frame(codec.build(codec.HELLO), session=1, chunk_size=3)
    new = split_akp_frame(codec.build(codec.HELLO), session=2, chunk_size=3)
    assert device.feed(encode_msp_request(old[0].encode())) == ()
    replies = []
    for segment in new:
        replies.extend(device.feed(encode_msp_request(segment.encode())))
    assert len(replies) == 1
    assert decode_msp_response(replies[0])[0] == 0xA1
