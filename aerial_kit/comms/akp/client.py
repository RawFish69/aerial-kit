"""Typed client for the Aerial Kit configuration protocol."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Any, cast

from ..transport import Transport
from . import codec


class AkError(RuntimeError):
    """Base class for Aerial Kit client failures."""


class AkProtocolError(AkError):
    """The peer returned bytes that do not describe a valid reply."""


class AkWriteRefused(AkError):
    """A mutation was not requested explicitly or was refused by the board."""


@dataclass(frozen=True)
class AkHello:
    version: int
    product: str
    parameter_count: int
    changed: str
    features: int | None
    config_hash: int | None
    feature_names: tuple[str, ...]


@dataclass(frozen=True)
class AkStatus:
    flight_state: int
    link_live: int
    gps_fix_type: int
    gps_satellites: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    latitude: float
    longitude: float
    motors: tuple[int, ...]
    raw: dict[str, Any] = field(default_factory=dict, compare=False)


@dataclass(frozen=True)
class AkTelemetry:
    uptime_ms: int
    status: AkStatus


@dataclass(frozen=True)
class AkParameterValue:
    status: int
    name: str
    value: str


@dataclass(frozen=True)
class AkWriteResult:
    status: int
    message: str

    @property
    def ok(self) -> bool:
        return self.status == 0


@dataclass(frozen=True)
class AkLogInfo:
    count: int


@dataclass(frozen=True)
class AkLogSource:
    source: int
    name: str | None
    count: int | None


@dataclass(frozen=True)
class AkLogRecord:
    values: tuple[int | float, ...]


@dataclass(frozen=True)
class AkParamInfo:
    index: int
    name: str | None
    type: int | None
    group: int | None
    group_name: str | None
    decimals: int | None
    flags: int | None
    secret: bool
    minimum: str | None
    maximum: str | None
    default: str | None
    max_len: int | None
    unavailable: str | None = None


@dataclass(frozen=True)
class AkRcState:
    status: int
    status_name: str
    flags: int
    flag_names: tuple[str, ...]
    protocol: int | None
    protocol_name: str | None
    channels: tuple[int, ...]
    sticks: tuple[int, ...]
    switches: int
    switch_names: tuple[str, ...]
    counters: dict[str, int]


@dataclass(frozen=True)
class AkSensorInfo:
    status: int
    topic: int
    topic_name: str | None
    present: bool
    body: dict[str, Any]


@dataclass(frozen=True)
class AkOutputInfo:
    status: int
    status_name: str
    count: int
    motors: int
    servos: int
    cap_pct: int
    outputs: tuple[dict[str, Any], ...]


@dataclass(frozen=True)
class AkOutputTest:
    status: int
    status_name: str
    op: int
    op_name: str | None
    kind: int
    kind_name: str | None
    index: int
    level_pct: int
    remaining_ms: int


@dataclass(frozen=True)
class AkLogStreamFrame:
    status: int
    source: int
    index: int
    record: AkLogRecord | None


@dataclass(frozen=True)
class AkPreflightLine:
    name: str
    verdict: int
    detail: str


@dataclass(frozen=True)
class AkMissionState:
    status: int
    op: int
    verb: str | None
    active: bool
    requested: bool
    count: int
    index: int
    channel: int
    reached: int
    started: int
    cancelled: int
    hold_alt_mm: int


@dataclass(frozen=True)
class AkCalibrationState:
    status: int
    status_name: str
    verb: int
    verb_name: str | None
    active: bool
    step: int
    faces: int
    samples: int
    rejected: int
    results: tuple[int, ...]
    result_names: tuple[str, ...]


@dataclass(frozen=True)
class AkPerformance:
    status: int
    loops: int
    samples: int
    nominal_us: int
    period_last_us: int
    period_min_us: int
    period_max_us: int
    late: int
    jitter_p50_us: int
    jitter_p99_us: int
    jitter_max_us: int
    jitter_over: int
    section_avg_us_x10: tuple[int, ...]
    section_max_us: tuple[int, ...]
    load_permille: int


@dataclass(frozen=True)
class AkMotorTelemetry:
    status: int
    status_name: str
    count: int
    poles: int
    motors: tuple[dict[str, Any], ...]


class AkClient:
    """Request/response client shared by serial, TCP, UDP and memory transports.

    Read operations may time out. Mutating operations never retry: a board that
    refuses a write has answered, and transmitting it again is not a recovery.
    """

    def __init__(self, transport: Transport, *, timeout: float = 2.0) -> None:
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.transport = transport
        self.timeout = float(timeout)
        self._client = codec.Client(
            lambda _size: transport.read_exact(1, self.timeout),
            transport.write,
        )

    def close(self) -> None:
        self.transport.close()

    def _request(self, command: int, payload: bytes = b"") -> bytes:
        try:
            return cast(bytes, self._client.request(command, payload))
        except (IOError, OSError) as exc:
            raise AkProtocolError(str(exc)) from exc

    def hello(self) -> AkHello:
        try:
            data = codec.parse_hello(self._request(codec.HELLO))
        except (IndexError, KeyError, ValueError) as exc:
            raise AkProtocolError(f"invalid hello reply: {exc}") from exc
        return AkHello(
            version=int(data["version"]),
            product=str(data["product"]),
            parameter_count=int(data["count"]),
            changed=str(data["changed"]),
            features=data["features"],
            config_hash=data["config_hash"],
            feature_names=tuple(codec.feature_names(data["features"])),
        )

    def status(self) -> AkStatus:
        data = codec.parse_status(self._request(codec.STATUS))
        return AkStatus(
            flight_state=data["flight_state"],
            link_live=data["link_live"],
            gps_fix_type=data["gps_fix_type"],
            gps_satellites=data["gps_satellites"],
            roll_deg=data["roll_deg"],
            pitch_deg=data["pitch_deg"],
            yaw_deg=data["yaw_deg"],
            latitude=data["lat"],
            longitude=data["lon"],
            motors=tuple(data["motors"]),
            raw=data,
        )

    def get_parameter(self, index: int) -> AkParameterValue:
        payload = self._request(codec.PARAM_GET, bytes((index,)))
        status = payload[0]
        if len(payload) == 1:
            return AkParameterValue(status=status, name="", value="")
        try:
            name = codec.c_string(payload, 1)
            value = codec.c_string(payload, 2 + len(name))
        except (IndexError, ValueError) as exc:
            raise AkProtocolError(f"invalid parameter reply: {exc}") from exc
        return AkParameterValue(status=status, name=name, value=value)

    def set_parameter(
        self,
        index: int,
        value: str,
        *,
        allow_write: bool = False,
    ) -> AkWriteResult:
        if not allow_write:
            raise AkWriteRefused("AKP parameter writes require allow_write=True")
        payload = self._request(
            codec.PARAM_SET,
            bytes((index,)) + value.encode("ascii") + b"\0",
        )
        status, message = codec.set_reply(payload)
        if status != 0:
            raise AkWriteRefused(f"parameter write refused: {message}")
        return AkWriteResult(status=status, message=message)

    def save_parameters(self, *, allow_write: bool = False) -> AkWriteResult:
        if not allow_write:
            raise AkWriteRefused("AKP save requires allow_write=True")
        payload = self._request(codec.PARAM_SAVE)
        status = payload[0]
        message = codec.c_string(payload, 1) if len(payload) > 1 else ""
        if status != 0:
            raise AkWriteRefused(f"parameter save refused: {message}")
        return AkWriteResult(status=status, message=message)

    def default_parameters(
        self,
        *,
        index: int | None = None,
        allow_write: bool = False,
    ) -> AkWriteResult:
        if not allow_write:
            raise AkWriteRefused("AKP defaults require allow_write=True")
        payload = bytes((1, index & 0xFF)) if index is not None else bytes((2,))
        reply = self._request(codec.PARAM_DEFAULT, payload)
        status = reply[0]
        message = codec.c_string(reply, 1) if len(reply) > 1 else ""
        if status != 0:
            raise AkWriteRefused(f"parameter defaults refused: {message}")
        return AkWriteResult(status=status, message=message)

    def log_info(self) -> AkLogInfo:
        payload = self._request(codec.LOG_INFO)
        if payload[0] != 0 or len(payload) < 3:
            raise AkProtocolError("log info refused or truncated")
        return AkLogInfo(count=int.from_bytes(payload[1:3], "little"))

    def log_record(self, index: int) -> AkLogRecord:
        payload = self._request(codec.LOG_GET, struct.pack("<H", index))
        if payload[0] != 0:
            raise AkProtocolError(f"log record {index} refused")
        return AkLogRecord(values=tuple(codec.parse_log_record(payload)))

    def log_source(self, source: int = 0) -> AkLogSource:
        count = codec.select_log(self._client, source)
        return AkLogSource(
            source=source,
            name=codec.LOG_SOURCE_NAMES.get(source),
            count=count,
        )

    def telemetry_subscribe(self, hz: int) -> int:
        return int(codec.subscribe(self._client, hz))

    def telemetry_next(self) -> AkTelemetry:
        data = codec.next_telemetry(self._client)
        status = AkStatus(
            flight_state=data["flight_state"],
            link_live=data["link_live"],
            gps_fix_type=data["gps_fix_type"],
            gps_satellites=data["gps_satellites"],
            roll_deg=data["roll_deg"],
            pitch_deg=data["pitch_deg"],
            yaw_deg=data["yaw_deg"],
            latitude=data["lat"],
            longitude=data["lon"],
            motors=tuple(data["motors"]),
            raw=data,
        )
        return AkTelemetry(uptime_ms=int(data["uptime_ms"]), status=status)

    def parameter_info(self, first: int = 0, limit: int | None = None) -> tuple[AkParamInfo, ...]:
        entries = codec.fetch_param_info(self._client, first, limit)
        return tuple(
            AkParamInfo(
                index=int(entry["index"]),
                name=entry.get("name"),
                type=entry.get("type"),
                group=entry.get("group"),
                group_name=entry.get("group_name"),
                decimals=entry.get("decimals"),
                flags=entry.get("flags"),
                secret=bool(entry.get("secret", False)),
                minimum=entry.get("min"),
                maximum=entry.get("max"),
                default=entry.get("default"),
                max_len=entry.get("max_len"),
                unavailable=entry.get("unavailable"),
            )
            for entry in entries
        )

    def parameter_help(self, index: int) -> str:
        return cast(str, codec.fetch_param_help(self._client, index))

    def rc_channels(self) -> AkRcState:
        data = codec.parse_rc_channels(self._request(codec.RC_CHANNELS))
        return AkRcState(
            status=data["status"],
            status_name=data["status_name"],
            flags=data["flags"],
            flag_names=tuple(data["flag_names"]),
            protocol=data["protocol"],
            protocol_name=data["protocol_name"],
            channels=tuple(data["channels"]),
            sticks=tuple(data["sticks"]),
            switches=data["switches"],
            switch_names=tuple(data["switch_names"]),
            counters={
                name: int(data[name])
                for name in ("bytes", "frames", "crc_errors", "rejected",
                             "lost", "failsafe_frames", "dropped")
            },
        )

    def sensor_info(self, topic: int) -> AkSensorInfo:
        data = codec.sensor_info(self._client, topic)
        return AkSensorInfo(
            status=data["status"],
            topic=data["topic"],
            topic_name=data["topic_name"],
            present=bool(data["present"]),
            body=dict(data["body"] or {}),
        )

    def output_info(self) -> AkOutputInfo:
        data = codec.output_info(self._client)
        return AkOutputInfo(
            status=data["status"],
            status_name=data["status_name"],
            count=data["count"],
            motors=data["motors"],
            servos=data["servos"],
            cap_pct=data["cap_pct"],
            outputs=tuple(data["outputs"]),
        )

    def output_test(
        self,
        *,
        stop: bool = False,
        kind: int = codec.OUTPUT_MOTOR,
        index: int = 0,
        level_pct: int = 0,
        allow_write: bool = False,
    ) -> AkOutputTest:
        if not allow_write:
            raise AkWriteRefused("output tests require allow_write=True")
        op = codec.OUTPUT_TEST_STOP if stop else codec.OUTPUT_TEST_HOLD
        data = codec.output_test(self._client, op, kind, index, level_pct)
        return AkOutputTest(**data)

    def start_log_stream(
        self, source: int, start: int, count: int, hz: int
    ) -> tuple[int, int, int, int] | None:
        return cast(
            tuple[int, int, int, int] | None,
            codec.start_log_stream(self._client, source, start, count, hz),
        )

    def next_log_frame(self) -> AkLogStreamFrame:
        status, source, index, record = codec.next_log_frame(self._client)
        return AkLogStreamFrame(
            status=status,
            source=source,
            index=index,
            record=AkLogRecord(tuple(record)) if record is not None else None,
        )

    def preflight(self) -> tuple[AkPreflightLine, ...]:
        return tuple(
            AkPreflightLine(name=name, verdict=verdict, detail=detail)
            for name, verdict, detail in codec.fetch_preflight(self._client)
        )

    def mission(self, verb: int = codec.MISSION_STATUS, *, allow_write: bool = False) -> AkMissionState:
        if verb != codec.MISSION_STATUS and not allow_write:
            raise AkWriteRefused("mission mutations require allow_write=True")
        data = codec.fetch_mission(self._client, verb)
        return AkMissionState(**data)

    def calibration_state(self) -> AkCalibrationState:
        data = codec.calibration_state(self._client)
        return AkCalibrationState(**data)

    def calibrate(
        self,
        verb: int,
        *,
        face: int = 0,
        millivolts: int = 0,
        allow_write: bool = False,
    ) -> AkCalibrationState:
        if not allow_write:
            raise AkWriteRefused("calibration requires allow_write=True")
        data = codec.calibrate(self._client, verb, face, millivolts)
        return AkCalibrationState(**data)

    def performance(self) -> AkPerformance:
        data = codec.parse_perf(self._request(codec.PERF))
        return AkPerformance(
            status=data["status"],
            loops=data["loops"],
            samples=data["samples"],
            nominal_us=data["nominal_us"],
            period_last_us=data["period_last_us"],
            period_min_us=data["period_min_us"],
            period_max_us=data["period_max_us"],
            late=data["late"],
            jitter_p50_us=data["jitter_p50_us"],
            jitter_p99_us=data["jitter_p99_us"],
            jitter_max_us=data["jitter_max_us"],
            jitter_over=data["jitter_over"],
            section_avg_us_x10=tuple(data["section_avg_x10"]),
            section_max_us=tuple(data["section_max_us"]),
            load_permille=data["load_permille"],
        )

    def motor_telemetry(self) -> AkMotorTelemetry:
        data = codec.parse_motor_telemetry(self._request(codec.MOTOR_TELEMETRY))
        return AkMotorTelemetry(
            status=data["status"],
            status_name=data["status_name"],
            count=data["count"],
            poles=data["poles"],
            motors=tuple(data["motors"]),
        )
