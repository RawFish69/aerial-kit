"""ExpressLRS binding identity helpers carried in CRSF MSP envelopes."""

from __future__ import annotations

import hashlib
import re

from .codec import (
    CRSF_ADDRESS_ELRS_LUA,
    CRSF_FRAMETYPE_MSP_REQUEST,
    CRSF_FRAMETYPE_MSP_RESPONSE,
    CRSF_FRAMETYPE_MSP_WRITE,
    CrsfFrame,
    crc8_dvb_s2,
)

MSP_ELRS_RXTX_CONFIG = 0x2D
MSP_RXTX_UID = 0x00
MSP_RXTX_BIND_PHRASE = 0x01
MSP_RXTX_MODEL_ID = 0x0A
UID_LENGTH = 6


class CrsfUidError(ValueError):
    """A UID request, write or response was invalid."""


def binding_uid(phrase: str) -> bytes:
    """Return the first six MD5 bytes ExpressLRS derives from a phrase."""
    if phrase == "":
        return bytes(UID_LENGTH)
    material = b'-DMY_BINDING_PHRASE="' + phrase.encode("utf-8") + b'"'
    return hashlib.md5(material, usedforsecurity=False).digest()[:UID_LENGTH]


def parse_uid(value: str | bytes | bytearray | memoryview) -> bytes:
    """Parse six hexadecimal bytes, accepting colons, dashes or spaces."""
    if isinstance(value, str):
        compact = re.sub(r"[\s:_-]", "", value)
        if not re.fullmatch(r"[0-9a-fA-F]{12}", compact):
            raise ValueError("UID must contain exactly six hexadecimal bytes")
        return bytes.fromhex(compact)
    raw = bytes(value)
    if len(raw) != UID_LENGTH:
        raise ValueError("UID must contain exactly six bytes")
    return raw


def format_uid(uid: str | bytes | bytearray | memoryview) -> str:
    """Format a UID as six lower-case, colon-separated bytes."""
    return ":".join(f"{byte:02x}" for byte in parse_uid(uid))


def is_unbound_uid(uid: str | bytes | bytearray | memoryview) -> bool:
    return parse_uid(uid) == bytes(UID_LENGTH)


def _extended(
    frame_type: int,
    destination: int,
    origin: int,
    payload: bytes,
) -> CrsfFrame:
    return CrsfFrame(
        address=0xC8,
        frame_type=frame_type,
        payload=bytes((destination, origin)) + payload,
    )


def encode_uid_request(
    destination: int,
    *,
    origin: int = CRSF_ADDRESS_ELRS_LUA,
) -> CrsfFrame:
    """Encode the MSP-v2 RXTX UID read request used by ExpressLRS 4.1.0."""
    msp = bytes(
        (
            0x50,
            0x00,
            MSP_ELRS_RXTX_CONFIG,
            0x00,
            0x01,
            0x00,
            MSP_RXTX_UID,
        )
    )
    return _extended(
        CRSF_FRAMETYPE_MSP_REQUEST,
        destination,
        origin,
        msp + bytes((crc8_dvb_s2(msp[1:]),)),
    )


def encode_uid_write(
    destination: int,
    uid: str | bytes | bytearray | memoryview,
    *,
    origin: int = CRSF_ADDRESS_ELRS_LUA,
) -> CrsfFrame:
    """Encode an MSP-v1 direct UID write."""
    payload = bytes((MSP_RXTX_UID,)) + parse_uid(uid)
    return _encode_msp_v1_write(destination, payload, origin=origin)


def encode_binding_phrase_write(
    destination: int,
    phrase: str,
    *,
    origin: int = CRSF_ADDRESS_ELRS_LUA,
) -> CrsfFrame:
    """Encode an MSP-v1 binding-phrase write; the module hashes it."""
    raw = phrase.encode("utf-8")
    if len(raw) > 48:
        raise ValueError("ExpressLRS binding phrases must fit in 48 UTF-8 bytes")
    payload = bytes((MSP_RXTX_BIND_PHRASE,)) + raw
    return _encode_msp_v1_write(destination, payload, origin=origin)


def _encode_msp_v1_write(
    destination: int,
    payload: bytes,
    *,
    origin: int,
) -> CrsfFrame:
    body = bytes((0x30, len(payload), MSP_ELRS_RXTX_CONFIG)) + payload
    return _extended(
        CRSF_FRAMETYPE_MSP_WRITE,
        destination,
        origin,
        body + bytes((_xor_crc(body[1:]),)),
    )


def decode_uid_response(frame: CrsfFrame) -> bytes:
    """Return the six-byte UID from an ExpressLRS MSP-v1 response."""
    if frame.frame_type != CRSF_FRAMETYPE_MSP_RESPONSE:
        raise CrsfUidError("not a CRSF MSP response")
    payload = frame.payload
    if len(payload) < 6:
        raise CrsfUidError("MSP UID response is too short")
    msp = payload[2:]
    if msp[0] != 0x30:
        raise CrsfUidError(f"unexpected MSP response header 0x{msp[0]:02x}")
    length = msp[1]
    function = msp[2]
    if function != MSP_ELRS_RXTX_CONFIG:
        raise CrsfUidError(
            f"MSP response function 0x{function:02x}, expected 0x2d"
        )
    if len(msp) != length + 4:
        raise CrsfUidError(
            f"MSP response says {length} payload bytes but carries {len(msp) - 4}"
        )
    if _xor_crc(msp[1:-1]) != msp[-1]:
        raise CrsfUidError("bad inner MSP response CRC")
    body = msp[3:-1]
    if len(body) != UID_LENGTH + 1 or body[0] != MSP_RXTX_UID:
        raise CrsfUidError("MSP response does not contain a six-byte UID")
    return body[1:]


def uid_from_frame_bytes(data: bytes) -> bytes:
    """Decode exactly one complete CRSF UID response frame."""
    if len(data) < 4:
        raise CrsfUidError("short CRSF UID response frame")
    frame = CrsfFrame(
        address=data[0],
        frame_type=data[2],
        payload=data[3:-1],
    )
    if int.from_bytes(bytes((data[1],)), "big") != len(data) - 2:
        raise CrsfUidError("CRSF UID response length does not match the frame")
    if crc8_dvb_s2(data[2:-1]) != data[-1]:
        raise CrsfUidError("bad CRSF UID response CRC")
    return decode_uid_response(frame)


def _xor_crc(data: bytes | bytearray | memoryview) -> int:
    """ExpressLRS MSP-v1 CRC: XOR, unlike CRSF's DVB-S2 outer CRC."""
    value = 0
    for byte in data:
        value ^= byte
    return value
