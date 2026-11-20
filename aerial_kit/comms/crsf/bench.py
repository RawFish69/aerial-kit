"""Bounded CRSF host-link bench operations.

These helpers deliberately separate bytes sent from evidence received. A drive
command can establish that valid RC frames were written; only observed
telemetry, receiver RC frames and link statistics can establish an RF link.
"""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

from .codec import (
    CRSF_ADDRESS_CRSF_TRANSMITTER,
    CRSF_ADDRESS_FLIGHT_CONTROLLER,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_ADDRESS_RECEIVER,
    CRSF_FRAMETYPE_ATTITUDE,
    CRSF_FRAMETYPE_BARO_ALTITUDE,
    CRSF_FRAMETYPE_BATTERY_SENSOR,
    CRSF_FRAMETYPE_DEVICE_INFO,
    CRSF_FRAMETYPE_GPS,
    CRSF_FRAMETYPE_LINK_STATISTICS,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
    AttitudeTelemetry,
    BarometerTelemetry,
    BatteryTelemetry,
    CrsfFrame,
    CrsfLink,
    CrsfParser,
    GpsTelemetry,
    decode_telemetry,
    make_attitude_frame,
    make_barometer_frame,
    make_battery_frame,
    make_gps_frame,
    make_rc_channels_frame,
    unpack_channels,
)
from .parameters import CrsfParameterClient, CrsfTimeout
from .radio import CrsfChannelMap, RcSticks
from ..transport import Transport

MAX_BENCH_DURATION_SEC = 300.0
MAX_DRIVE_HZ = 500.0
MAX_INJECT_FRAMES = 1024
TELEMETRY_FRAME_TYPES = {
    CRSF_FRAMETYPE_ATTITUDE,
    CRSF_FRAMETYPE_BARO_ALTITUDE,
    CRSF_FRAMETYPE_BATTERY_SENSOR,
    CRSF_FRAMETYPE_GPS,
    CRSF_FRAMETYPE_LINK_STATISTICS,
}

FRAME_NAMES = {
    CRSF_FRAMETYPE_GPS: "gps",
    CRSF_FRAMETYPE_BATTERY_SENSOR: "battery",
    CRSF_FRAMETYPE_BARO_ALTITUDE: "barometer",
    CRSF_FRAMETYPE_LINK_STATISTICS: "link_statistics",
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED: "rc_channels",
    CRSF_FRAMETYPE_ATTITUDE: "attitude",
    0x21: "flight_mode",
    CRSF_FRAMETYPE_DEVICE_INFO: "device_info",
}


def _validate_duration(duration: float) -> float:
    value = float(duration)
    if not 0.0 < value <= MAX_BENCH_DURATION_SEC:
        raise ValueError(
            f"duration must be greater than zero and at most "
            f"{MAX_BENCH_DURATION_SEC:g} seconds"
        )
    return value


def _version(value: int) -> str:
    return f"{(value >> 16) & 0xff}.{(value >> 8) & 0xff}.{value & 0xff}"


@dataclass(frozen=True)
class RadioStatusReport:
    path: str
    usb_serial: str | None
    logical_baud: int
    role: str
    product: str
    origin: int
    destination: int
    firmware_version: int
    firmware_string: str
    parameter_count: int
    parameter_version: int
    device_serial: int
    hardware_version: int

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def radio_status(
    transport: Transport,
    *,
    path: str,
    logical_baud: int,
    role: str = "auto",
    timeout: float = 1.0,
) -> RadioStatusReport:
    """Ping an endpoint and return the device information it actually sent."""
    if role not in {"auto", "tx", "rx"}:
        raise ValueError("role must be auto, tx or rx")
    link = CrsfLink(transport)
    attempts = (
        [
            ("tx", CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER),
            ("rx", CRSF_ADDRESS_RECEIVER, CRSF_ADDRESS_FLIGHT_CONTROLLER),
        ]
        if role == "auto"
        else [
            (
                role,
                CRSF_ADDRESS_CRSF_TRANSMITTER
                if role == "tx"
                else CRSF_ADDRESS_RECEIVER,
                CRSF_ADDRESS_RADIO_TRANSMITTER
                if role == "tx"
                else CRSF_ADDRESS_FLIGHT_CONTROLLER,
            )
        ]
    )
    last: Exception | None = None
    for detected_role, destination, origin in attempts:
        try:
            client = CrsfParameterClient(
                link,
                destination=destination,
                origin=origin,
                timeout=timeout,
            )
            info = client.ping()
            usb_serial = getattr(transport, "usb_serial_number", None)
            return RadioStatusReport(
                path=path,
                usb_serial=str(usb_serial) if usb_serial else None,
                logical_baud=int(logical_baud),
                role=detected_role,
                product=info.name,
                origin=info.origin,
                destination=info.destination,
                firmware_version=info.software_version,
                firmware_string=_version(info.software_version),
                parameter_count=info.field_count,
                parameter_version=info.parameter_version,
                device_serial=info.serial,
                hardware_version=info.hardware_version,
            )
        except (CrsfTimeout, TimeoutError) as exc:
            last = exc
    raise CrsfTimeout(
        f"{path} did not return CRSF DEVICE_INFO: {last}"
    ) from last


@dataclass
class _FrameCollector:
    frames_by_type: dict[str, int] = field(default_factory=dict)
    telemetry_by_kind: dict[str, dict[str, object]] = field(default_factory=dict)
    channels: tuple[int, ...] | None = None
    rc_intervals_ms: list[float] = field(default_factory=list)
    last_seen: dict[str, float] = field(default_factory=dict)
    rc_frames: int = 0
    link_statistics_frames: int = 0
    invalid_telemetry_frames: int = 0
    _last_rc_at: float | None = None

    def process(self, frames: Iterable[CrsfFrame], now: float) -> None:
        for frame in frames:
            name = FRAME_NAMES.get(frame.frame_type, f"0x{frame.frame_type:02x}")
            self.frames_by_type[name] = self.frames_by_type.get(name, 0) + 1
            self.last_seen[name] = now
            if frame.frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                if len(frame.payload) != 22:
                    self.invalid_telemetry_frames += 1
                    continue
                self.rc_frames += 1
                self.channels = unpack_channels(frame.payload)
                if self._last_rc_at is not None:
                    self.rc_intervals_ms.append((now - self._last_rc_at) * 1000.0)
                self._last_rc_at = now
            if frame.frame_type == CRSF_FRAMETYPE_LINK_STATISTICS:
                self.link_statistics_frames += 1
            try:
                telemetry = decode_telemetry(frame)
            except ValueError:
                self.invalid_telemetry_frames += 1
                continue
            if telemetry is not None:
                self.telemetry_by_kind[type(telemetry).__name__] = asdict(telemetry)

    @property
    def telemetry_kinds(self) -> tuple[str, ...]:
        return tuple(sorted(self.telemetry_by_kind))

    def cadence_hz(self) -> float | None:
        positive = [value for value in self.rc_intervals_ms if value > 0.0]
        if not positive:
            return None
        return 1000.0 / (sum(positive) / len(positive))

    def missed_intervals(self) -> int:
        positive = sorted(value for value in self.rc_intervals_ms if value > 0.0)
        if len(positive) < 3:
            return 0
        expected = positive[len(positive) // 2]
        return sum(value > expected * 1.5 for value in positive)

    def last_seen_seconds(self, end: float) -> dict[str, float]:
        return {
            name: round(max(0.0, end - seen), 3)
            for name, seen in sorted(self.last_seen.items())
        }


@dataclass(frozen=True)
class MonitorReport:
    duration_sec: float
    bytes_read: int
    frames: int
    frames_by_type: dict[str, int]
    crc_errors: int
    resyncs: int
    framing_errors: int
    last_seen_seconds: dict[str, float]
    telemetry: dict[str, dict[str, object]]
    status: str
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)


def monitor_link(
    transport: Transport,
    *,
    duration: float = 5.0,
    cancel_event: threading.Event | None = None,
) -> MonitorReport:
    seconds = _validate_duration(duration)
    link = CrsfLink(transport)
    parser = link.parser
    collector = _FrameCollector()
    start = time.monotonic()
    deadline = start + seconds
    while True:
        now = time.monotonic()
        if now >= deadline or (cancel_event is not None and cancel_event.is_set()):
            break
        timeout = min(0.05, max(0.0, deadline - now))
        try:
            frames = link.poll(512, timeout=timeout)
        except (OSError, TimeoutError):
            break
        collector.process(frames, time.monotonic())
    end = time.monotonic()
    observed_duration = max(0.001, end - start)
    status = (
        "no_bytes"
        if parser.bytes_received == 0
        else "no_valid_crsf"
        if parser.valid_frames == 0
        else "valid_crsf"
    )
    return MonitorReport(
        duration_sec=round(observed_duration, 3),
        bytes_read=parser.bytes_received,
        frames=parser.valid_frames,
        frames_by_type=dict(sorted(collector.frames_by_type.items())),
        crc_errors=parser.crc_errors,
        resyncs=parser.resyncs,
        framing_errors=parser.framing_errors,
        last_seen_seconds=collector.last_seen_seconds(end),
        telemetry=collector.telemetry_by_kind,
        status=status,
        passed=parser.valid_frames > 0,
    )


@dataclass(frozen=True)
class CaptureReport:
    duration_sec: float
    bytes_read: int
    frames: int
    frames_by_type: dict[str, int]
    rc_frames: int
    rc_cadence_hz: float | None
    channels: tuple[int, ...] | None
    missed_intervals: int
    crc_errors: int
    resyncs: int
    framing_errors: int
    link_statistics_frames: int
    telemetry: dict[str, dict[str, object]]
    injected_frames: int
    status: str
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)


def capture_link(
    transport: Transport,
    *,
    duration: float = 10.0,
    inject_frames: Sequence[bytes] = (),
    inject_interval_sec: float | None = None,
    cancel_event: threading.Event | None = None,
) -> CaptureReport:
    seconds = _validate_duration(duration)
    link = CrsfLink(transport)
    parser = link.parser
    collector = _FrameCollector()
    start = time.monotonic()
    deadline = start + seconds
    injected = 0
    next_inject = start
    while True:
        now = time.monotonic()
        if injected < len(inject_frames) and (
            injected == 0 or inject_interval_sec is None or now >= next_inject
        ):
            link.transport.write(inject_frames[injected])
            injected += 1
            if inject_interval_sec is None:
                next_inject = deadline + 1.0
            else:
                next_inject = now + max(0.01, float(inject_interval_sec))
        if now >= deadline or (cancel_event is not None and cancel_event.is_set()):
            break
        timeout = min(0.05, max(0.0, deadline - now))
        try:
            frames = link.poll(512, timeout=timeout)
        except (OSError, TimeoutError):
            break
        collector.process(frames, time.monotonic())
    end = time.monotonic()
    observed_duration = max(0.001, end - start)
    cadence = collector.cadence_hz()
    if cadence is None and collector.rc_frames:
        cadence = collector.rc_frames / observed_duration
    if parser.bytes_received == 0:
        status = "no_bytes"
    elif collector.rc_frames == 0 and parser.crc_errors:
        status = "crc_errors_without_valid_rc"
    elif collector.rc_frames == 0 and parser.resyncs:
        status = "resync_only"
    elif collector.rc_frames == 0:
        status = "no_valid_rc"
    else:
        status = "valid_rc"
    return CaptureReport(
        duration_sec=round(observed_duration, 3),
        bytes_read=parser.bytes_received,
        frames=parser.valid_frames,
        frames_by_type=dict(sorted(collector.frames_by_type.items())),
        rc_frames=collector.rc_frames,
        rc_cadence_hz=(
            round(cadence, 3) if cadence is not None else None
        ),
        channels=collector.channels,
        missed_intervals=collector.missed_intervals(),
        crc_errors=parser.crc_errors,
        resyncs=parser.resyncs,
        framing_errors=parser.framing_errors,
        link_statistics_frames=collector.link_statistics_frames,
        telemetry=collector.telemetry_by_kind,
        injected_frames=injected,
        status=status,
        passed=collector.rc_frames > 0,
    )


def _pattern_sticks(pattern: str, elapsed: float) -> RcSticks:
    if pattern == "disarmed":
        return RcSticks(0.0, 0.0, 0.0, 0.0)
    if pattern == "arm-test":
        return RcSticks(0.0, 0.0, 0.0, 0.0)
    if pattern == "channel-test":
        phase = elapsed % 8.0
        if phase < 2.0:
            return RcSticks(-0.25, 0.0, 0.0, 0.1)
        if phase < 4.0:
            return RcSticks(0.25, 0.0, 0.0, 0.1)
        if phase < 6.0:
            return RcSticks(0.0, -0.25, 0.0, 0.1)
        return RcSticks(0.0, 0.25, 0.0, 0.1)
    raise ValueError("pattern must be disarmed, arm-test or channel-test")


@dataclass(frozen=True)
class DriveReport:
    duration_sec: float
    requested_hz: float
    frames_sent: int
    actual_hz: float
    pattern: str
    arm_hold_sec: float
    arm_hold_completed: bool
    received_frames: int
    link_statistics_frames: int
    telemetry: dict[str, dict[str, object]]
    crc_errors: int
    resyncs: int
    disarmed_at_end: bool
    error: str | None
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)


def _write_safe_sequence(
    link: CrsfLink,
    channel_map: CrsfChannelMap,
    *,
    was_armed: bool,
) -> bool:
    safe = RcSticks(0.0, 0.0, 0.0, 0.0)
    if was_armed:
        link.transport.write(
            make_rc_channels_frame(channel_map.encode(safe, enabled=True))
        )
        time.sleep(0.02)
    link.transport.write(
        make_rc_channels_frame(channel_map.encode(safe, enabled=False))
    )
    return True


def drive_link(
    transport: Transport,
    *,
    hz: float = 150.0,
    duration: float = 10.0,
    pattern: str = "disarmed",
    arm_hold_sec: float = 0.75,
    allow_write: bool = False,
    channel_map: CrsfChannelMap | None = None,
    cancel_event: threading.Event | None = None,
    sticks: RcSticks | None = None,
) -> DriveReport:
    if not allow_write:
        raise PermissionError("radio drive requires explicit write authorization")
    send_hz = float(hz)
    if not 0.0 < send_hz <= MAX_DRIVE_HZ:
        raise ValueError(f"hz must be greater than zero and at most {MAX_DRIVE_HZ:g}")
    seconds = _validate_duration(duration)
    hold = float(arm_hold_sec)
    armed_pattern = pattern in {"arm-test", "channel-test"} or sticks is not None
    if armed_pattern and hold <= 0.5:
        raise ValueError("arm_hold_sec must be longer than the firmware's 500 ms arm hold")
    if armed_pattern and seconds <= hold:
        raise ValueError("duration must be longer than arm_hold_sec")

    channel_map = channel_map or CrsfChannelMap()
    link = CrsfLink(transport)
    collector = _FrameCollector()
    parser = link.parser
    start = time.monotonic()
    deadline = start + seconds
    period = 1.0 / send_hz
    next_send = start
    sent = 0
    was_armed = False
    error: str | None = None
    try:
        while True:
            now = time.monotonic()
            if now >= deadline or (cancel_event is not None and cancel_event.is_set()):
                break
            elapsed = now - start
            armed = armed_pattern and elapsed >= hold
            during_hold = armed_pattern and elapsed < hold
            if during_hold or armed:
                command_sticks = RcSticks(0.0, 0.0, 0.0, 0.0) if during_hold else (
                    sticks
                    if sticks is not None
                    else _pattern_sticks(pattern, elapsed - hold)
                )
            else:
                command_sticks = _pattern_sticks("disarmed", elapsed)
            channels = channel_map.encode(
                command_sticks,
                enabled=during_hold or armed,
            )
            link.transport.write(make_rc_channels_frame(channels))
            sent += 1
            was_armed = during_hold or armed
            collector.process(link.poll(512, timeout=0.0), time.monotonic())
            next_send += period
            delay = next_send - time.monotonic()
            if delay > 0.0:
                if cancel_event is None:
                    time.sleep(delay)
                else:
                    cancel_event.wait(delay)
            elif delay < -period:
                next_send = time.monotonic()
    except (OSError, TimeoutError) as exc:
        error = str(exc)
    finally:
        try:
            disarmed = _write_safe_sequence(link, channel_map, was_armed=was_armed)
        except OSError as exc:
            disarmed = False
            error = error or str(exc)
    end = time.monotonic()
    duration_actual = max(0.001, end - start)
    return DriveReport(
        duration_sec=round(duration_actual, 3),
        requested_hz=send_hz,
        frames_sent=sent,
        actual_hz=round(sent / duration_actual, 3),
        pattern="custom" if sticks is not None else pattern,
        arm_hold_sec=hold,
        arm_hold_completed=armed_pattern and duration_actual > hold,
        received_frames=parser.valid_frames,
        link_statistics_frames=collector.link_statistics_frames,
        telemetry=collector.telemetry_by_kind,
        crc_errors=parser.crc_errors,
        resyncs=parser.resyncs,
        disarmed_at_end=disarmed,
        error=error,
        passed=sent > 0 and disarmed and error is None,
    )


@dataclass(frozen=True)
class InjectReport:
    frames: int
    duration_sec: float
    frame_types: tuple[int, ...]
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)


def load_crsf_frames_file(path: str | Path) -> list[bytes]:
    source = Path(path)
    data = source.read_bytes()
    frames: list[CrsfFrame] = []
    parser = CrsfParser()
    if data.lstrip().startswith((b"[", b"{")):
        document = json.loads(data.decode("utf-8"))
        if (
            isinstance(document, dict)
            and document.get("protocol") == "crsf"
            and isinstance(document.get("cases"), list)
        ):
            selected: list[bytes] = []
            for case in document["cases"]:
                if not isinstance(case, dict) or not isinstance(case.get("raw_hex"), str):
                    continue
                parsed = parser.feed(bytes.fromhex(case["raw_hex"]))
                if len(parsed) == 1 and parsed[0].frame_type in TELEMETRY_FRAME_TYPES:
                    selected.append(parsed[0].encode())
            if not selected:
                raise ValueError(f"{source} contains no supported telemetry vectors")
            return selected
        hex_values: list[str] = []

        def collect(value: object) -> None:
            if isinstance(value, dict):
                for key in ("raw_hex", "frame_hex", "raw"):
                    candidate = value.get(key)
                    if isinstance(candidate, str):
                        hex_values.append(candidate)
                        return
                for child in value.values():
                    collect(child)
            elif isinstance(value, list):
                for child in value:
                    collect(child)
            elif isinstance(value, str) and all(
                character in "0123456789abcdefABCDEF xX\n\t" for character in value
            ):
                hex_values.extend(value.replace(",", " ").split())

        collect(document)
        if not hex_values:
            raise ValueError(f"{source} contains no CRSF frame hex")
        raw = bytes.fromhex("".join(value.replace("0x", "") for value in hex_values))
    elif data and data[0] in {0xC8, 0xEA, 0xEC, 0xEE}:
        raw = data
    else:
        text = data.decode("ascii")
        tokens = text.replace(",", " ").split()
        raw = bytes.fromhex("".join(token.removeprefix("0x") for token in tokens))

    frames = parser.feed(raw)
    if (
        not frames
        or parser.crc_errors
        or parser.resyncs
        or parser.framing_errors
        or parser.pending_bytes
    ):
        raise ValueError(f"{source} is not a complete valid CRSF frame sequence")
    if len(frames) > MAX_INJECT_FRAMES:
        raise ValueError(f"{source} has more than {MAX_INJECT_FRAMES} frames")
    unsupported = [frame.frame_type for frame in frames if frame.frame_type not in TELEMETRY_FRAME_TYPES]
    if unsupported:
        raise ValueError(
            "radio inject accepts telemetry frames only; refusing types "
            + ", ".join(f"0x{value:02x}" for value in unsupported)
        )
    return [frame.encode() for frame in frames]


def inject_frames(
    transport: Transport,
    frames: Sequence[bytes],
    *,
    allow_write: bool = False,
    interval_sec: float = 0.02,
) -> InjectReport:
    if not allow_write:
        raise PermissionError("radio inject requires explicit write authorization")
    if not frames:
        raise ValueError("no CRSF frames to inject")
    if len(frames) > MAX_INJECT_FRAMES:
        raise ValueError(f"at most {MAX_INJECT_FRAMES} frames may be injected")
    parsed: list[CrsfFrame] = []
    parser = CrsfParser()
    for raw in frames:
        current = parser.feed(raw)
        if len(current) != 1:
            raise ValueError("injection sequence contains an invalid CRSF frame")
        parsed.extend(current)
    if parser.pending_bytes or parser.crc_errors or parser.resyncs:
        raise ValueError("injection sequence contains malformed CRSF bytes")
    unsupported = [frame.frame_type for frame in parsed if frame.frame_type not in TELEMETRY_FRAME_TYPES]
    if unsupported:
        raise ValueError("radio inject accepts validated telemetry frames only")
    start = time.monotonic()
    for raw in frames:
        transport.write(raw)
        if interval_sec > 0:
            time.sleep(min(0.25, max(0.0, interval_sec)))
    return InjectReport(
        frames=len(frames),
        duration_sec=round(time.monotonic() - start, 3),
        frame_types=tuple(frame.frame_type for frame in parsed),
        passed=True,
    )


def default_telemetry_frames() -> list[bytes]:
    return [
        make_attitude_frame(AttitudeTelemetry(0.05, -0.02, 0.3)),
        make_battery_frame(BatteryTelemetry(16.8, 12.3, 1500, 88)),
        make_gps_frame(GpsTelemetry(47.3977, 8.5456, 10.0, 90.0, 450.0, 12)),
        make_barometer_frame(BarometerTelemetry(123.4, -2.5)),
    ]


@dataclass(frozen=True)
class BenchReport:
    tx: DriveReport
    rx: CaptureReport
    tx_status: RadioStatusReport | None
    rx_status: RadioStatusReport | None
    requirements: dict[str, str]
    overall: str

    def to_dict(self) -> dict[str, object]:
        return {
            "tx": self.tx.to_dict(),
            "rx": self.rx.to_dict(),
            "tx_status": None if self.tx_status is None else self.tx_status.to_dict(),
            "rx_status": None if self.rx_status is None else self.rx_status.to_dict(),
            "requirements": dict(sorted(self.requirements.items())),
            "overall": self.overall,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)


def run_two_board_bench(
    tx_transport: Transport,
    rx_transport: Transport,
    *,
    tx_path: str,
    rx_path: str,
    duration: float = 15.0,
    hz: float = 150.0,
    telemetry_frames: Sequence[bytes] | None = None,
    channel_map: CrsfChannelMap | None = None,
) -> BenchReport:
    seconds = _validate_duration(duration)

    def status_or_none(
        transport: Transport, path: str, logical_baud: int, role: str
    ) -> RadioStatusReport | None:
        try:
            return radio_status(
                transport,
                path=path,
                logical_baud=logical_baud,
                role=role,
            )
        except CrsfTimeout:
            return None

    tx_status = status_or_none(tx_transport, tx_path, 400_000, "tx")
    rx_status = status_or_none(rx_transport, rx_path, 420_000, "rx")
    injection = list(telemetry_frames or default_telemetry_frames())
    with ThreadPoolExecutor(max_workers=2) as pool:
        tx_future = pool.submit(
            drive_link,
            tx_transport,
            hz=hz,
            duration=seconds,
            pattern="arm-test",
            arm_hold_sec=0.75,
            allow_write=True,
            channel_map=channel_map,
        )
        rx_future = pool.submit(
            capture_link,
            rx_transport,
            duration=seconds,
            inject_frames=injection,
            inject_interval_sec=0.25,
        )
        tx_report = tx_future.result()
        rx_report = rx_future.result()

    telemetry_class_names = {
        "attitude": "AttitudeTelemetry",
        "battery": "BatteryTelemetry",
        "gps": "GpsTelemetry",
        "barometer": "BarometerTelemetry",
    }
    injected_kinds = {
        telemetry_class_names.get(
            FRAME_NAMES.get(CrsfParser().feed(raw)[0].frame_type, ""),
            "",
        )
        for raw in injection
    }
    telemetry_round_trip = bool(
        injected_kinds & set(tx_report.telemetry)
    )
    requirements = {
        "tx_device_info": "PASS" if tx_status is not None else "FAIL",
        "rx_device_info": "PASS" if rx_status is not None else "FAIL",
        "tx_rc_drive": "PASS" if tx_report.frames_sent > 0 else "FAIL",
        "rx_rc_capture": "PASS" if rx_report.rc_frames > 0 else "FAIL",
        "tx_link_statistics": (
            "PASS" if tx_report.link_statistics_frames > 0 else "NOT TESTED"
        ),
        "rx_link_statistics": (
            "PASS" if rx_report.link_statistics_frames > 0 else "NOT TESTED"
        ),
        "telemetry_round_trip": "PASS" if telemetry_round_trip else "NOT TESTED",
    }
    rf_proven = all(
        requirements[name] == "PASS"
        for name in (
            "rx_rc_capture",
            "tx_link_statistics",
            "rx_link_statistics",
        )
    )
    requirements["rf_link"] = "PASS" if rf_proven else "NOT TESTED"
    required_values = (
        "tx_device_info",
        "rx_device_info",
        "tx_rc_drive",
        "rx_rc_capture",
        "telemetry_round_trip",
    )
    basic_passed = all(requirements[name] == "PASS" for name in required_values)
    overall = (
        "PASS"
        if basic_passed and rf_proven
        else "FAIL"
        if not basic_passed
        else "NOT TESTED"
    )
    return BenchReport(
        tx=tx_report,
        rx=rx_report,
        tx_status=tx_status,
        rx_status=rx_status,
        requirements=requirements,
        overall=overall,
    )
