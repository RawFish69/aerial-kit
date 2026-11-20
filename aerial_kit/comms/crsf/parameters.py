"""CRSF device information and parameter protocol."""

from __future__ import annotations

import struct
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

from .codec import (
    CRSF_ADDRESS_BROADCAST,
    CRSF_ADDRESS_CRSF_TRANSMITTER,
    CRSF_ADDRESS_FLIGHT_CONTROLLER,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_FRAMETYPE_COMMAND,
    CRSF_FRAMETYPE_DEVICE_INFO,
    CRSF_FRAMETYPE_DEVICE_PING,
    CRSF_FRAMETYPE_PARAMETER_READ,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
    CRSF_FRAMETYPE_PARAMETER_WRITE,
    CRSF_FRAMETYPE_MSP_RESPONSE,
    CrsfFrame,
    CrsfLink,
    make_device_ping,
    make_frame,
)
from .uid import (
    UID_LENGTH,
    decode_uid_response,
    encode_binding_phrase_write,
    encode_uid_request,
    encode_uid_write,
    parse_uid,
)

CRSF_UINT8 = 0
CRSF_INT8 = 1
CRSF_UINT16 = 2
CRSF_INT16 = 3
CRSF_UINT32 = 4
CRSF_INT32 = 5
CRSF_UINT64 = 6
CRSF_INT64 = 7
CRSF_FLOAT = 8
CRSF_TEXT_SELECTION = 9
CRSF_STRING = 10
CRSF_FOLDER = 11
CRSF_INFO = 12
CRSF_COMMAND = 13
CRSF_VTX = 15
CRSF_OUT_OF_RANGE = 127
CRSF_FIELD_HIDDEN = 0x80
CRSF_FIELD_TYPE_MASK = 0x7F

CRSF_COMMAND_SUBCMD_RX = 0x10
CRSF_COMMAND_SUBCMD_RX_BIND = 0x01

PARAMETER_TYPE_NAMES = {
    CRSF_UINT8: "uint8",
    CRSF_INT8: "int8",
    CRSF_UINT16: "uint16",
    CRSF_INT16: "int16",
    CRSF_UINT32: "uint32",
    CRSF_INT32: "int32",
    CRSF_UINT64: "uint64",
    CRSF_INT64: "int64",
    CRSF_FLOAT: "float",
    CRSF_TEXT_SELECTION: "selection",
    CRSF_STRING: "string",
    CRSF_FOLDER: "folder",
    CRSF_INFO: "info",
    CRSF_COMMAND: "command",
    CRSF_VTX: "vtx",
}


class CrsfError(RuntimeError):
    """Base class for CRSF client failures."""


class CrsfTimeout(CrsfError):
    """No matching frame arrived before the deadline."""


class CrsfProtocolError(CrsfError):
    """A frame was present but did not contain a valid parameter record."""


class CrsfWriteRefused(CrsfError):
    """A mutation was not permitted or was not confirmed."""


class CrsfChunkLoss(CrsfProtocolError):
    """A multi-frame CRSF parameter record did not deliver every chunk."""

    def __init__(self, field_id: int, chunk_index: int, total: int | None) -> None:
        self.field_id = field_id
        self.chunk_index = chunk_index
        self.total = total
        suffix = "" if total is None else f" of {total}"
        super().__init__(
            f"CRSF parameter {field_id} missing chunk {chunk_index}{suffix}"
        )


@dataclass(frozen=True)
class CrsfDeviceInfo:
    destination: int
    origin: int
    name: str
    serial: int
    hardware_version: int
    software_version: int
    field_count: int
    parameter_version: int


@dataclass(frozen=True)
class CrsfParameter:
    id: int
    parent: int
    type: str
    hidden: bool
    name: str
    value: int | float | str | None = None
    minimum: int | float | None = None
    maximum: int | float | None = None
    default: int | float | None = None
    options: tuple[str, ...] = ()
    units: str = ""
    children: tuple[int, ...] = ()
    step: int | None = None
    timeout_ms: int | None = None
    info: str = ""
    raw_hex: str = field(default="", compare=False)


def _cstring(data: bytes, offset: int) -> tuple[str, int]:
    end = data.find(b"\0", offset)
    if end < 0:
        raise CrsfProtocolError("unterminated CRSF string")
    return data[offset:end].decode("ascii", "replace"), end + 1


def _extended(frame_type: int, destination: int, origin: int, payload: bytes) -> bytes:
    return make_frame(frame_type, bytes((destination, origin)) + payload)


def parse_device_info(frame: CrsfFrame) -> CrsfDeviceInfo:
    if frame.frame_type != CRSF_FRAMETYPE_DEVICE_INFO:
        raise CrsfProtocolError("not a CRSF device-information frame")
    data = frame.payload
    if len(data) < 18:
        raise CrsfProtocolError("short CRSF device-information frame")
    destination, origin = data[0], data[1]
    name, offset = _cstring(data, 2)
    if len(data) - offset < 14:
        raise CrsfProtocolError("device information is missing its fixed fields")
    serial, hardware, software, fields, version = struct.unpack_from(">IIIBB", data, offset)
    return CrsfDeviceInfo(
        destination=destination,
        origin=origin,
        name=name,
        serial=serial,
        hardware_version=hardware,
        software_version=software,
        field_count=fields,
        parameter_version=version,
    )


def parse_parameter_record(record: bytes) -> CrsfParameter:
    """Parse one fully reassembled settings-entry payload.

    ``record`` begins with field id, chunks remaining, parent and type. The
    first chunk's continuation bytes follow.
    """
    if len(record) < 5:
        raise CrsfProtocolError("short CRSF parameter record")
    field_id = record[0]
    if record[1] != 0:
        raise CrsfProtocolError("parameter record is not fully reassembled")
    parent = record[2]
    raw_type = record[3]
    type_id = raw_type & CRSF_FIELD_TYPE_MASK
    hidden = bool(raw_type & CRSF_FIELD_HIDDEN)
    name, offset = _cstring(record, 4)
    data = record[offset:]

    result = CrsfParameter(
        id=field_id,
        parent=parent,
        type=PARAMETER_TYPE_NAMES.get(type_id, "unknown"),
        hidden=hidden,
        name=name,
        raw_hex=record.hex(),
    )

    if type_id in (CRSF_UINT8, CRSF_INT8):
        if len(data) < 5:
            raise CrsfProtocolError("short 8-bit parameter")
        fmt = ">bbbb" if type_id == CRSF_INT8 else ">BBBB"
        value, minimum, maximum, default = struct.unpack_from(fmt, data)
        units, _ = _cstring(data, 4)
        result = CrsfParameter(**{**result.__dict__, "value": value,
                                  "minimum": minimum, "maximum": maximum,
                                  "default": default, "units": units})
    elif type_id in (CRSF_UINT16, CRSF_INT16):
        if len(data) < 9:
            raise CrsfProtocolError("short 16-bit parameter")
        fmt = ">hhhh" if type_id == CRSF_INT16 else ">HHHH"
        value, minimum, maximum, default = struct.unpack_from(fmt, data)
        units, _ = _cstring(data, 8)
        result = CrsfParameter(**{**result.__dict__, "value": value,
                                  "minimum": minimum, "maximum": maximum,
                                  "default": default, "units": units})
    elif type_id in (CRSF_UINT32, CRSF_INT32):
        if len(data) < 17:
            raise CrsfProtocolError("short 32-bit parameter")
        fmt = ">iiii" if type_id == CRSF_INT32 else ">IIII"
        value, minimum, maximum, default = struct.unpack_from(fmt, data)
        units, _ = _cstring(data, 16)
        result = CrsfParameter(**{**result.__dict__, "value": value,
                                  "minimum": minimum, "maximum": maximum,
                                  "default": default, "units": units})
    elif type_id == CRSF_TEXT_SELECTION:
        options_text, offset = _cstring(data, 0)
        if len(data) - offset < 4:
            raise CrsfProtocolError("short selection parameter")
        value, _minimum, _maximum, _default = struct.unpack_from(">BBBB", data, offset)
        units, _ = _cstring(data, offset + 4)
        result = CrsfParameter(**{**result.__dict__, "value": value,
                                  "options": tuple(options_text.split(";")),
                                  "units": units})
    elif type_id in (CRSF_STRING, CRSF_INFO):
        value, _ = _cstring(data, 0)
        result = CrsfParameter(**{**result.__dict__, "value": value})
    elif type_id == CRSF_FOLDER:
        children = []
        for child in data:
            if child == 0xFF:
                break
            children.append(child)
        else:
            raise CrsfProtocolError("unterminated CRSF folder")
        result = CrsfParameter(**{**result.__dict__, "children": tuple(children)})
    elif type_id == CRSF_COMMAND:
        if len(data) < 2:
            raise CrsfProtocolError("short CRSF command parameter")
        step, timeout_ms = data[0], data[1] * 10
        info, _ = _cstring(data, 2)
        result = CrsfParameter(**{**result.__dict__, "step": step,
                                  "timeout_ms": timeout_ms, "info": info})
    else:
        result = CrsfParameter(**{**result.__dict__, "value": data.hex()})
    return result


class CrsfParameterClient:
    """Synchronous device and parameter client over a ``CrsfLink``."""

    def __init__(
        self,
        link: CrsfLink,
        *,
        destination: int = CRSF_ADDRESS_CRSF_TRANSMITTER,
        origin: int = CRSF_ADDRESS_RADIO_TRANSMITTER,
        timeout: float = 0.5,
        chunk_interval: float = 0.05,
        max_chunk_attempts: int = 8,
    ) -> None:
        self.link = link
        self.destination = destination
        self.origin = origin
        self.timeout = timeout
        self.chunk_interval = max(0.0, float(chunk_interval))
        self.max_chunk_attempts = max(1, int(max_chunk_attempts))
        self._pending_frames: deque[CrsfFrame] = deque()

    def ping(self, *, destination: int | None = None, timeout: float | None = None) -> CrsfDeviceInfo:
        target = self.destination if destination is None else destination
        frame = make_device_ping(destination=target, origin=self.origin)
        self.link.transport.write(frame)
        reply = self._wait_for(
            CRSF_FRAMETYPE_DEVICE_INFO,
            timeout=self.timeout if timeout is None else timeout,
        )
        return parse_device_info(reply)

    def read_parameter(self, field_id: int, *, timeout: float | None = None) -> CrsfParameter:
        chunks: dict[int, bytes] = {}
        total: int | None = None
        attempts: dict[int, int] = {}
        per_chunk_timeout = self.timeout if timeout is None else timeout
        while total is None or len(chunks) < total:
            missing = [
                index for index in range(total or 1) if index not in chunks
            ]
            target = missing[0] if missing else 0
            attempts[target] = attempts.get(target, 0) + 1
            if attempts[target] > self.max_chunk_attempts:
                raise CrsfChunkLoss(field_id, target, total)
            # Give the endpoint's USB/CRSF output FIFO time to drain between
            # chunks. The first request for a new chunk used to be sent with no
            # spacing, which loses later chunks on the Rev A3 native-USB path.
            if self.chunk_interval and (total is not None or attempts[target] > 1):
                time.sleep(self.chunk_interval)
            request = _extended(
                CRSF_FRAMETYPE_PARAMETER_READ,
                self.destination,
                self.origin,
                bytes((field_id, target)),
            )
            self.link.transport.write(request)
            try:
                frame = self._wait_for(
                    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                    timeout=per_chunk_timeout,
                    field_id=field_id,
                    addresses=True,
                )
            except CrsfTimeout:
                continue
            payload = frame.payload
            if len(payload) < 4:
                raise CrsfProtocolError("short CRSF parameter chunk")
            response_field = payload[2]
            remaining = payload[3]
            if response_field != field_id:
                continue
            if remaining > 63:
                # ExpressLRS uses 0xff for an out-of-range chunk request.
                # Treat it as stale/corrupt rather than turning it into a
                # 256-entry record and poisoning the rest of the tree.
                continue
            if len(payload) == 4:
                raise CrsfProtocolError("CRSF parameter chunk has no data")
            if total is None:
                total = remaining + 1
            elif remaining >= total:
                raise CrsfProtocolError(
                    f"parameter {field_id} reported chunk remainder {remaining} "
                    f"for a {total}-chunk record"
                )
            response_index = total - remaining - 1
            data = payload[4:]
            previous = chunks.get(response_index)
            if previous is not None and previous != data:
                raise CrsfProtocolError(
                    f"parameter {field_id} chunk {response_index} changed between replies"
                )
            chunks[response_index] = data
        # CRSF sends (field, chunks-remaining) in every frame header but does
        # not repeat them in the data bytes. parse_parameter_record() consumes
        # one header, so synthesize only the first one.
        complete = bytes((field_id, 0)) + b"".join(
            chunks[index] for index in range(total)
        )
        return parse_parameter_record(complete)

    def read_parameter_tree(
        self,
        *,
        root: int = 0,
        timeout: float | None = None,
        on_parameter: Callable[[CrsfParameter], None] | None = None,
    ) -> dict[int, CrsfParameter]:
        pending = [root]
        result: dict[int, CrsfParameter] = {}
        while pending:
            field_id = pending.pop()
            if field_id in result:
                continue
            try:
                parameter = self.read_parameter(field_id, timeout=timeout)
            except CrsfError as exc:
                raise CrsfProtocolError(
                    f"parameter tree stopped at field {field_id}: {exc}"
                ) from exc
            result[field_id] = parameter
            if on_parameter is not None:
                on_parameter(parameter)
            pending.extend(reversed(parameter.children))
        return result

    def write_parameter(
        self,
        field_id: int,
        value: int,
        *,
        allow_write: bool = False,
        verify: bool = True,
        timeout: float | None = None,
    ) -> CrsfParameter | None:
        if not allow_write:
            raise CrsfWriteRefused("CRSF parameter writes require allow_write=True")
        parameter = self.read_parameter(field_id, timeout=timeout)
        if parameter.type in ("string", "info", "folder"):
            raise CrsfWriteRefused(f"{parameter.type} parameters are not writable here")
        type_id = next(
            (key for key, name in PARAMETER_TYPE_NAMES.items() if name == parameter.type),
            None,
        )
        if type_id in (CRSF_UINT8, CRSF_INT8, CRSF_TEXT_SELECTION, CRSF_COMMAND):
            body = bytes((int(value) & 0xFF,))
        elif type_id in (CRSF_UINT16, CRSF_INT16):
            body = struct.pack(">H", int(value) & 0xFFFF)
        elif type_id in (CRSF_UINT32, CRSF_INT32, CRSF_FLOAT):
            body = struct.pack(">I", int(value) & 0xFFFFFFFF)
        else:
            raise CrsfWriteRefused(f"no write encoding for {parameter.type}")
        request = _extended(
            CRSF_FRAMETYPE_PARAMETER_WRITE,
            self.destination,
            self.origin,
            bytes((field_id,)) + body,
        )
        self.link.transport.write(request)
        if not verify:
            return None
        updated = self.read_parameter(field_id, timeout=timeout)
        if int(updated.value if updated.value is not None else -1) != int(value):
            raise CrsfWriteRefused(
                f"parameter {field_id} did not confirm value {value}"
            )
        return updated

    def bind(self, *, allow_write: bool = False) -> None:
        if not allow_write:
            raise CrsfWriteRefused("binding requires allow_write=True")
        command = _extended(
            CRSF_FRAMETYPE_COMMAND,
            self.destination,
            self.origin,
            bytes((CRSF_COMMAND_SUBCMD_RX, CRSF_COMMAND_SUBCMD_RX_BIND)),
        )
        self.link.transport.write(command)

    def read_uid(
        self,
        *,
        attempts: int = 2,
        timeout: float = 4.0,
    ) -> bytes:
        """Read and validate the module's six-byte ExpressLRS UID."""
        last: CrsfTimeout | None = None
        for _ in range(max(1, attempts)):
            request = encode_uid_request(
                self.destination,
                origin=self.origin,
            )
            self.link.send_frame(request)
            try:
                response = self._wait_for(
                    CRSF_FRAMETYPE_MSP_RESPONSE,
                    timeout=timeout,
                    addresses=True,
                )
            except CrsfTimeout as exc:
                last = exc
                continue
            uid = decode_uid_response(response)
            if len(uid) != UID_LENGTH:
                raise CrsfProtocolError("module returned a malformed UID")
            return uid
        raise CrsfTimeout(
            f"no ExpressLRS MSP UID response from address "
            f"0x{self.destination:02x} after {max(1, attempts)} attempts"
        ) from last

    def write_uid(
        self,
        uid: str | bytes | bytearray | memoryview,
        *,
        allow_write: bool = False,
    ) -> bytes:
        """Write a direct UID. Callers must reconnect and read it back."""
        if not allow_write:
            raise CrsfWriteRefused("UID writes require allow_write=True")
        value = parse_uid(uid)
        self.link.send_frame(
            encode_uid_write(
                self.destination,
                value,
                origin=self.origin,
            )
        )
        return value

    def write_binding_phrase(
        self,
        phrase: str,
        *,
        allow_write: bool = False,
    ) -> None:
        """Write a phrase. The module stores only its derived six-byte UID."""
        if not allow_write:
            raise CrsfWriteRefused("binding-phrase writes require allow_write=True")
        self.link.send_frame(
            encode_binding_phrase_write(
                self.destination,
                phrase,
                origin=self.origin,
            )
        )

    def _wait_for(
        self,
        frame_type: int,
        *,
        timeout: float,
        field_id: int | None = None,
        addresses: bool = False,
    ) -> CrsfFrame:
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) >= 0:
            if self._pending_frames:
                frame = self._pending_frames.popleft()
            else:
                frames = self.link.poll(512, timeout=remaining)
                if not frames:
                    continue
                self._pending_frames.extend(frames)
                continue
            if frame.frame_type != frame_type:
                continue
            if frame_type == CRSF_FRAMETYPE_DEVICE_INFO or addresses:
                if len(frame.payload) < 2:
                    continue
                destination, origin = frame.payload[0], frame.payload[1]
                if destination not in (self.origin, CRSF_ADDRESS_BROADCAST):
                    continue
                if self.destination != CRSF_ADDRESS_BROADCAST and origin != self.destination:
                    continue
            if field_id is not None:
                if len(frame.payload) < 3 or frame.payload[2] != field_id:
                    continue
            return frame
        raise CrsfTimeout(f"no CRSF frame type 0x{frame_type:02x} before timeout")
