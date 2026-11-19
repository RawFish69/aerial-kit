"""High-level CRSF radio control and telemetry."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from ..transport import TransportClosed
from .codec import (
    CRSF_CHANNEL_COUNT,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MID,
    CRSF_CHANNEL_MIN,
    CrsfFrame,
    CrsfLink,
    CrsfParser,
    LinkStatisticsTelemetry,
    decode_telemetry,
    make_rc_channels_frame,
)


@dataclass(frozen=True)
class RcSticks:
    roll: float
    pitch: float
    yaw: float
    throttle: float


@dataclass(frozen=True)
class CrsfStickState:
    arm_request: bool
    sticks: RcSticks
    angle_mode: bool


@dataclass
class BackendArmSequencer:
    """Hold arm high at minimum throttle before handing over controller sticks."""

    arm_hold_sec: float = 1.0
    throttle_min: float = 0.0
    armed: bool = False
    hold_started: float | None = None

    def reset(self) -> None:
        self.armed = False
        self.hold_started = None

    def update(self, enabled: bool, sticks: RcSticks, now: float) -> tuple[RcSticks, bool]:
        if not enabled:
            self.reset()
            return RcSticks(0.0, 0.0, 0.0, self.throttle_min), False
        if self.armed:
            return sticks, True
        if self.hold_started is None:
            self.hold_started = now
        if now - self.hold_started >= self.arm_hold_sec:
            self.armed = True
            return sticks, True
        return RcSticks(0.0, 0.0, 0.0, self.throttle_min), True


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class CrsfChannelMap:
    """AETR switch map using one-based channel numbers."""

    roll: int = 1
    pitch: int = 2
    throttle: int = 3
    yaw: int = 4
    arm: int = 6
    mode: int = 5
    arm_threshold: int = 1300
    angle_mode_count: int = CRSF_CHANNEL_MAX
    disabled_mode_count: int = CRSF_CHANNEL_MIN
    velocity: int = 0

    def __post_init__(self) -> None:
        channels = [self.roll, self.pitch, self.throttle, self.yaw, self.arm, self.mode]
        if self.velocity:
            channels.append(self.velocity)
        if any(channel < 1 or channel > CRSF_CHANNEL_COUNT for channel in channels):
            raise ValueError(
                f"CRSF channel numbers must be 1..{CRSF_CHANNEL_COUNT}, got {channels}"
            )
        if len(set(channels)) != len(channels):
            raise ValueError(f"CRSF channel map uses a channel twice: {channels}")

    @classmethod
    def from_mapping(cls, values: dict[str, int]) -> CrsfChannelMap:
        names = ["roll", "pitch", "throttle", "yaw", "arm", "mode", "velocity"]
        return cls(**{name: int(values.get(name, getattr(cls(), name))) for name in names})

    @staticmethod
    def _stick_count(value: float) -> int:
        normalized = _clamp(value, -1.0, 1.0)
        return round(CRSF_CHANNEL_MID + normalized * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID))

    @staticmethod
    def _throttle_count(value: float) -> int:
        normalized = _clamp(value, 0.0, 1.0)
        return round(CRSF_CHANNEL_MIN + normalized * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN))

    def encode(
        self,
        sticks: RcSticks,
        *,
        enabled: bool,
        velocity_hold: bool = False,
    ) -> tuple[int, ...]:
        channels = [CRSF_CHANNEL_MID] * CRSF_CHANNEL_COUNT
        channels[self.roll - 1] = self._stick_count(sticks.roll)
        channels[self.pitch - 1] = self._stick_count(sticks.pitch)
        channels[self.yaw - 1] = self._stick_count(sticks.yaw)
        channels[self.throttle - 1] = self._throttle_count(sticks.throttle)
        channels[self.arm - 1] = CRSF_CHANNEL_MAX if enabled else CRSF_CHANNEL_MIN
        channels[self.mode - 1] = (
            self.angle_mode_count if enabled else self.disabled_mode_count
        )
        if self.velocity:
            channels[self.velocity - 1] = (
                CRSF_CHANNEL_MAX if enabled and velocity_hold else CRSF_CHANNEL_MIN
            )
        elif velocity_hold:
            raise ValueError(
                "velocity hold asked for, but the channel map has no velocity channel"
            )
        return tuple(channels)

    def decode_sticks(self, channels: Sequence[int]) -> CrsfStickState:
        if len(channels) != CRSF_CHANNEL_COUNT:
            raise ValueError(
                f"expected {CRSF_CHANNEL_COUNT} CRSF channels, got {len(channels)}"
            )

        def stick(number: int) -> float:
            return _clamp(
                (float(channels[number - 1]) - CRSF_CHANNEL_MID)
                / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID),
                -1.0,
                1.0,
            )

        throttle = _clamp(
            (float(channels[self.throttle - 1]) - CRSF_CHANNEL_MIN)
            / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN),
            0.0,
            1.0,
        )
        return CrsfStickState(
            arm_request=int(channels[self.arm - 1]) >= self.arm_threshold,
            sticks=RcSticks(
                roll=stick(self.roll),
                pitch=stick(self.pitch),
                yaw=stick(self.yaw),
                throttle=throttle,
            ),
            angle_mode=int(channels[self.mode - 1]) >= self.angle_mode_count,
        )


@dataclass(frozen=True)
class CrsfLinkQuality:
    uplink_rssi_1_dbm: int
    uplink_rssi_2_dbm: int
    uplink_link_quality: int
    uplink_snr_db: int
    active_antenna: int
    rf_mode: int
    uplink_tx_power: int
    downlink_rssi_dbm: int
    downlink_link_quality: int
    downlink_snr_db: int


TelemetryCallback = Callable[[object], None]
FrameCallback = Callable[[CrsfFrame], None]


class CrsfRadio:
    """Fixed-rate RC sender with asynchronous CRSF telemetry.

    The sender thread owns channel writes. The receiver thread only parses bytes
    and dispatches callbacks. ``stop`` stops sending RC immediately and is the
    host-side failsafe action; it does not invent neutral frames.
    """

    def __init__(
        self,
        link: CrsfLink,
        *,
        channel_map: CrsfChannelMap | None = None,
        send_rate_hz: float = 150.0,
        arm_hold_sec: float = 1.0,
        throttle_min: float = 0.0,
    ) -> None:
        if send_rate_hz <= 0:
            raise ValueError("send_rate_hz must be positive")
        self.link = link
        self.channel_map = channel_map or CrsfChannelMap()
        self.send_rate_hz = float(send_rate_hz)
        self.sequencer = BackendArmSequencer(
            arm_hold_sec=arm_hold_sec,
            throttle_min=throttle_min,
        )
        self._sticks = RcSticks(0.0, 0.0, 0.0, 0.0)
        self._enabled = False
        self._callbacks: list[TelemetryCallback] = []
        self._frame_callbacks: list[FrameCallback] = []
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._send_thread: threading.Thread | None = None
        self._receive_thread: threading.Thread | None = None
        self._started_at = 0.0
        self._sent = 0
        self._received = 0
        self._last_frame_monotonic: float | None = None
        self.link_quality: CrsfLinkQuality | None = None
        self.last_error: Exception | None = None

    @property
    def sent_frames(self) -> int:
        return self._sent

    @property
    def received_frames(self) -> int:
        return self._received

    def set_sticks(self, sticks: RcSticks) -> None:
        with self._lock:
            self._sticks = sticks

    def arm(self) -> None:
        with self._lock:
            self._enabled = True

    def disarm(self) -> None:
        with self._lock:
            self._enabled = False

    def add_telemetry_callback(self, callback: TelemetryCallback) -> None:
        self._callbacks.append(callback)

    def add_frame_callback(self, callback: FrameCallback) -> None:
        """Observe every valid CRSF frame before telemetry decoding."""
        self._frame_callbacks.append(callback)

    def remove_frame_callback(self, callback: FrameCallback) -> None:
        self._frame_callbacks.remove(callback)

    def send_once(self, now: float | None = None) -> int:
        moment = time.monotonic() if now is None else float(now)
        with self._lock:
            sticks, enabled = self.sequencer.update(
                self._enabled, self._sticks, moment
            )
        channels = self.channel_map.encode(sticks, enabled=enabled)
        return self.link.transport.write(make_rc_channels_frame(channels))

    def start(self) -> None:
        if self._send_thread is not None and self._send_thread.is_alive():
            raise RuntimeError("radio is already running")
        self._stop.clear()
        self._started_at = time.monotonic()
        self._send_thread = threading.Thread(
            target=self._send_loop, name="CrsfRadio-send", daemon=True
        )
        self._receive_thread = threading.Thread(
            target=self._receive_loop, name="CrsfRadio-receive", daemon=True
        )
        self._send_thread.start()
        self._receive_thread.start()

    def stop(self) -> None:
        self._stop.set()
        for thread in (self._send_thread, self._receive_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(1.0)
        self._send_thread = None
        self._receive_thread = None

    def close(self) -> None:
        self.stop()
        self.link.close()

    def _send_loop(self) -> None:
        period = 1.0 / self.send_rate_hz
        deadline = time.monotonic()
        while not self._stop.is_set():
            try:
                self.send_once(time.monotonic())
                self._sent += 1
            except (TransportClosed, OSError) as exc:
                self.last_error = exc
                self._stop.set()
                break
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            elif delay < -period:
                deadline = time.monotonic()

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frames = self.link.poll(512)
            except (TransportClosed, OSError) as exc:
                self.last_error = exc
                self._stop.set()
                break
            for frame in frames:
                self._dispatch(frame)

    def _dispatch(self, frame: CrsfFrame) -> None:
        self._received += 1
        self._last_frame_monotonic = time.monotonic()
        for callback in tuple(self._frame_callbacks):
            callback(frame)
        telemetry = decode_telemetry(frame)
        if isinstance(telemetry, LinkStatisticsTelemetry):
            self.link_quality = CrsfLinkQuality(
                uplink_rssi_1_dbm=telemetry.uplink_rssi_1_dbm,
                uplink_rssi_2_dbm=telemetry.uplink_rssi_2_dbm,
                uplink_link_quality=telemetry.uplink_link_quality,
                uplink_snr_db=telemetry.uplink_snr_db,
                active_antenna=telemetry.active_antenna,
                rf_mode=telemetry.rf_mode,
                uplink_tx_power=telemetry.uplink_tx_power,
                downlink_rssi_dbm=telemetry.downlink_rssi_dbm,
                downlink_link_quality=telemetry.downlink_link_quality,
                downlink_snr_db=telemetry.downlink_snr_db,
            )
        if telemetry is not None:
            for callback in tuple(self._callbacks):
                callback(telemetry)
