"""Pure AKP-over-CRSF endpoint logic for the ELRS simulator."""

from __future__ import annotations

import struct

from aerial_kit.comms.akp import codec
from aerial_kit.comms.crsf import (
    AKP_MSP_RESPONSE_CHUNK,
    AKP_TUNNEL_MAX_PAYLOAD,
    AkpTunnelError,
    AkpTunnelSegment,
    AkpTunnelReassembler,
    CrsfFrame,
    decode_msp_request,
    encode_msp_response,
    split_akp_frame,
)


class AkpDemoBoard:
    """Small read-only AKP board used behind the simulated ELRS link."""

    parameters = (
        ("rate_kp_roll", "0.250"),
        ("rate_ki_roll", "1.500"),
        ("arm_channel", "6"),
    )

    def _response(self, command: int, payload: bytes) -> bytes:
        body = bytes((codec.VERSION, command | codec.RESPONSE_BIT, len(payload))) + payload
        return (
            bytes((codec.SYNC1, codec.SYNC2))
            + body
            + struct.pack("<H", codec.crc16(body))
        )

    def handle(self, frame: bytes) -> bytes:
        if (
            len(frame) < 7
            or frame[:2] != bytes((codec.SYNC1, codec.SYNC2))
            or frame[2] != codec.VERSION
        ):
            raise ValueError("not an AKP request")
        command = frame[3]
        length = frame[4]
        if len(frame) != length + 7:
            raise ValueError("AKP request length does not match")
        payload = frame[5:5 + length]
        expected = struct.unpack_from("<H", frame, 5 + length)[0]
        if codec.crc16(frame[2:5 + length]) != expected:
            raise ValueError("bad AKP request CRC")

        if command == codec.HELLO:
            product = b"aerialkit-sim-radio\0"
            body = (
                bytes((codec.VERSION,))
                + product
                + struct.pack("<H", len(self.parameters))
                + b"0\0"
                + struct.pack("<I", 0)
                + struct.pack("<I", 0)
            )
        elif command == codec.STATUS:
            body = struct.pack(
                "<BBBbhhhiiBBBB",
                1, 1, 3, 12,
                25, -50, 900,
                473_977_000, -1_227_654_321,
                20, 30, 40, 50,
            )
        elif command == codec.PARAM_GET and payload and payload[0] < len(self.parameters):
            name, value = self.parameters[payload[0]]
            body = b"\x00" + name.encode() + b"\0" + value.encode() + b"\0"
        elif command == codec.PARAM_GET:
            body = b"\x01"
        else:
            body = b"\x7f"
        return self._response(command, body)


class AkpSimDevice:
    """Reassemble uplink tunnel segments and segment AKP replies downlink."""

    def __init__(self, board: AkpDemoBoard | None = None) -> None:
        self.board = board or AkpDemoBoard()
        self.reassembler = AkpTunnelReassembler(max_payload=AKP_TUNNEL_MAX_PAYLOAD)
        self.requests = 0
        self.responses = 0
        self.errors = 0

    def feed(self, frame: CrsfFrame) -> tuple[CrsfFrame, ...]:
        if frame.frame_type not in (0x7A, 0x7C):
            return ()
        try:
            segment = AkpTunnelSegment.decode(decode_msp_request(frame))
            if segment.first and self.reassembler.active and \
                    self.reassembler.session != segment.session:
                self.reassembler.reset()
            request = self.reassembler.feed(segment)
            if request is None:
                return ()
            response = self.board.handle(request)
            self.requests += 1
            self.responses += 1
            return tuple(
                encode_msp_response(item.encode())
                for item in split_akp_frame(
                    response,
                    session=segment.session,
                    chunk_size=AKP_MSP_RESPONSE_CHUNK,
                )
            )
        except (AkpTunnelError, ValueError, IndexError):
            self.errors += 1
            self.reassembler.reset()
            return ()

