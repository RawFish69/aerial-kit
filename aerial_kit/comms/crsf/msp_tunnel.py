"""Carry segmented Aerial Kit frames inside CRSF MSP envelopes."""

from __future__ import annotations

import threading
import time

from ..transport import Link
from .codec import (
    CRSF_ADDRESS_FLIGHT_CONTROLLER,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_FRAMETYPE_MSP_REQUEST,
    CRSF_FRAMETYPE_MSP_RESPONSE,
    CRSF_FRAMETYPE_MSP_WRITE,
    CrsfFrame,
    crc8_dvb_s2,
)
from .radio import CrsfRadio
from .tunnel import (
    AKP_TUNNEL_MAGIC,
    AKP_TUNNEL_MAX_PAYLOAD,
    AkpTunnelError,
    AkpTunnelReassembler,
    split_akp_frame,
)

AKP_MSP_FUNCTION = 0x4B
AKP_MSP_REQUEST_CHUNK = 3
AKP_MSP_RESPONSE_CHUNK = 49
AKP_MSP_SEND_INTERVAL_S = 0.005


class CrsfAkpTunnelError(AkpTunnelError):
    """The MSP envelope around an AKP tunnel frame is invalid."""


def _msp_v2_request(function: int, payload: bytes) -> bytes:
    body = bytes(
        (
            0,
            function & 0xFF,
            (function >> 8) & 0xFF,
            len(payload) & 0xFF,
            (len(payload) >> 8) & 0xFF,
        )
    ) + payload
    return bytes((0x50,)) + body + bytes((crc8_dvb_s2(body),))


def encode_msp_request(
    payload: bytes, *, function: int = AKP_MSP_FUNCTION
) -> CrsfFrame:
    """Wrap tunnel bytes in the MSP-v2 request shape ExpressLRS forwards."""
    ext_payload = bytes(
        (CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_ADDRESS_RADIO_TRANSMITTER)
    ) + _msp_v2_request(function, payload)
    return CrsfFrame(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_MSP_REQUEST,
        ext_payload,
    )


def decode_msp_request(frame: CrsfFrame) -> bytes:
    """Return tunnel bytes from an MSP request/write CRSF frame."""
    if frame.frame_type not in (
        CRSF_FRAMETYPE_MSP_REQUEST,
        CRSF_FRAMETYPE_MSP_WRITE,
    ):
        raise CrsfAkpTunnelError(
            f"CRSF type 0x{frame.frame_type:02x} is not an MSP uplink"
        )
    payload = frame.payload
    if len(payload) < 9:
        raise CrsfAkpTunnelError("MSP request is too short")
    if payload[0] not in (CRSF_ADDRESS_FLIGHT_CONTROLLER, 0x00):
        raise CrsfAkpTunnelError(
            f"MSP request targets 0x{payload[0]:02x}, not the flight controller"
        )
    msp = payload[2:]
    if frame.frame_type == CRSF_FRAMETYPE_MSP_WRITE:
        if msp[0] != 0x30:
            raise CrsfAkpTunnelError("MSP write does not use a 0x30 header")
        length = msp[1]
        if msp[2] != AKP_MSP_FUNCTION:
            raise CrsfAkpTunnelError(
                f"MSP function 0x{msp[2]:02x} is not AKP"
            )
        if len(msp) != length + 3:
            raise CrsfAkpTunnelError(
                f"MSP write says {length} payload bytes but carries "
                f"{max(0, len(msp) - 3)}"
            )
        if crc8_dvb_s2(msp[1:-1]) != msp[-1]:
            raise CrsfAkpTunnelError("bad inner MSP write CRC")
        return msp[3:-1]

    if msp[0] != 0x50:
        raise CrsfAkpTunnelError(
            f"unsupported MSP v2 status/version 0x{msp[0]:02x}"
        )
    function = int.from_bytes(msp[2:4], "little")
    if function != AKP_MSP_FUNCTION:
        raise CrsfAkpTunnelError(f"MSP function 0x{function:04x} is not AKP")
    length = int.from_bytes(msp[4:6], "little")
    if len(msp) != 6 + length + 1:
        raise CrsfAkpTunnelError(
            f"MSP request says {length} payload bytes but carries "
            f"{max(0, len(msp) - 7)}"
        )
    if crc8_dvb_s2(msp[1:-1]) != msp[-1]:
        raise CrsfAkpTunnelError("bad inner MSP request CRC")
    return msp[6:-1]


def encode_msp_response(
    payload: bytes, *, function: int = AKP_MSP_FUNCTION
) -> CrsfFrame:
    """Wrap tunnel bytes in the MSP response shape ExpressLRS emits."""
    if not 0 <= len(payload) <= 0xFF:
        raise CrsfAkpTunnelError("MSP response payload must fit in one byte")
    body = bytes((len(payload) + 1, function & 0xFF)) + payload
    ext_payload = bytes(
        (CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_FLIGHT_CONTROLLER)
    ) + bytes((0x30,)) + body + bytes((crc8_dvb_s2(body),))
    return CrsfFrame(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_MSP_RESPONSE,
        ext_payload,
    )


def decode_msp_response(frame: CrsfFrame) -> bytes:
    """Return tunnel bytes from an MSP response frame."""
    if frame.frame_type != CRSF_FRAMETYPE_MSP_RESPONSE:
        raise CrsfAkpTunnelError(
            f"CRSF type 0x{frame.frame_type:02x} is not an MSP response"
        )
    payload = frame.payload
    if len(payload) < 6:
        raise CrsfAkpTunnelError("MSP response is too short")
    if payload[0] not in (CRSF_ADDRESS_RADIO_TRANSMITTER, 0x00):
        raise CrsfAkpTunnelError(
            f"MSP response targets 0x{payload[0]:02x}, not the radio"
        )
    msp = payload[2:]
    if msp[0] != 0x30:
        raise CrsfAkpTunnelError(
            f"unsupported MSP response header 0x{msp[0]:02x}"
        )
    length = msp[1]
    if msp[2] != AKP_MSP_FUNCTION:
        raise CrsfAkpTunnelError(f"MSP function 0x{msp[2]:02x} is not AKP")
    if len(msp) != length + 3:
        raise CrsfAkpTunnelError(
            f"MSP response says {length} payload bytes but carries "
            f"{max(0, len(msp) - 3)}"
        )
    if crc8_dvb_s2(msp[1:-1]) != msp[-1]:
        raise CrsfAkpTunnelError("bad inner MSP response CRC")
    return msp[3:-1]


class CrsfAkpTunnel(Link):
    """A byte transport that carries whole AKP frames over a live CRSF radio.

    ``CrsfRadio`` remains responsible for RC and ordinary telemetry. This
    transport observes raw CRSF frames and sends MSP requests through the same
    locked byte transport.
    """

    def __init__(
        self,
        radio: CrsfRadio,
        *,
        chunk_size: int = AKP_MSP_REQUEST_CHUNK,
        send_interval_s: float = AKP_MSP_SEND_INTERVAL_S,
        max_payload: int = AKP_TUNNEL_MAX_PAYLOAD,
    ) -> None:
        super().__init__(timeout=0.05)
        if send_interval_s < 0:
            raise ValueError("send_interval_s must be non-negative")
        self.radio = radio
        self.chunk_size = int(chunk_size)
        self.send_interval_s = float(send_interval_s)
        self.reassembler = AkpTunnelReassembler(max_payload=max_payload)
        self._buffer = bytearray()
        self._condition = threading.Condition()
        self._session = 0
        self._active_session: int | None = None
        self.last_error: CrsfAkpTunnelError | None = None
        self.dropped_segments = 0
        self.sent_frames = 0
        self.received_frames = 0
        self.radio.add_frame_callback(self._observe)

    @property
    def available(self) -> int:
        with self._condition:
            return len(self._buffer)

    def _next_session(self) -> int:
        self._session = (self._session % 255) + 1
        return self._session

    def _observe(self, frame: CrsfFrame) -> None:
        if frame.frame_type != CRSF_FRAMETYPE_MSP_RESPONSE:
            return
        try:
            segment = decode_msp_response(frame)
            if len(segment) < 3 or segment[0] != AKP_TUNNEL_MAGIC:
                raise CrsfAkpTunnelError("downlink does not carry an AKP segment")
            session = segment[1]
            if self._active_session is not None and session != self._active_session:
                self.dropped_segments += 1
                return
        except (IndexError, CrsfAkpTunnelError) as exc:
            self.dropped_segments += 1
            self.last_error = (
                exc
                if isinstance(exc, CrsfAkpTunnelError)
                else CrsfAkpTunnelError("truncated downlink MSP segment")
            )
            return
        try:
            complete = self.reassembler.feed(segment)
        except AkpTunnelError as exc:
            self.last_error = (
                exc if isinstance(exc, CrsfAkpTunnelError)
                else CrsfAkpTunnelError(str(exc))
            )
            self.reassembler.reset()
            return
        if complete is None:
            return
        self.received_frames += 1
        with self._condition:
            self._buffer.extend(complete)
            self._condition.notify_all()

    def _write(self, data: bytes) -> int:
        session = self._next_session()
        self._active_session = session
        with self._condition:
            self.reassembler.reset()
        segments = split_akp_frame(data, session=session, chunk_size=self.chunk_size)
        for index, segment in enumerate(segments):
            self.radio.link.send_frame(encode_msp_request(segment.encode()))
            self.sent_frames += 1
            if self.send_interval_s and index + 1 < len(segments):
                time.sleep(self.send_interval_s)
        return len(data)

    def _read(self, size: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._buffer:
                left = deadline - time.monotonic()
                if left <= 0:
                    return b""
                self._condition.wait(left)
            count = len(self._buffer) if size < 0 else min(size, len(self._buffer))
            output = bytes(self._buffer[:count])
            del self._buffer[:count]
            return output

    def _close(self) -> None:
        self.radio.remove_frame_callback(self._observe)
