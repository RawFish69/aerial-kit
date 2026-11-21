"""Bounded keyboard teleoperation over a CRSF transmitter link.

The simulator and the aircraft use different plants, but the host-side safety
contract is the same: output begins disarmed, arming requires an explicit
command-line confirmation and keypress, every stick change is a bounded fixed
step, and every exit path attempts a disarmed command.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass
import os
import sys
import time
from typing import Any, Protocol

from .crsf import (
    CRSF_FRAMETYPE_LINK_STATISTICS,
    CrsfChannelMap,
    CrsfLink,
    RcSticks,
    decode_telemetry,
)
from .crsf.codec import make_rc_channels_frame
from .transport import Transport

MAX_TELEOP_DURATION_SEC = 300.0
MAX_TELEOP_HZ = 500.0
SAFE_STICKS = RcSticks(0.0, 0.0, 0.0, 0.0)

KEY_HELP = (
    "A/D roll, W/S pitch, Q/E yaw, R/F throttle, C neutralize, "
    "Enter arm, X panic/disarm and exit, Ctrl-C exit"
)


class KeyboardInput(Protocol):
    """Non-blocking keyboard source used by the teleop loop."""

    def read_key(self, timeout: float = 0.0) -> str | None:
        """Return one key or ``None`` when no key is available."""


class TerminalKeyboard:
    """Read single keypresses from a POSIX terminal without waiting for Enter."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._settings: Any = None

    def __enter__(self) -> TerminalKeyboard:
        if not sys.stdin.isatty():
            raise ValueError(
                "radio teleop needs an interactive terminal; "
                "use `radio drive` for scripted input"
            )
        import termios
        import tty

        self._fd = sys.stdin.fileno()
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        if self._fd is not None and self._settings is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
        self._fd = None
        self._settings = None

    def read_key(self, timeout: float = 0.0) -> str | None:
        if self._fd is None:
            raise RuntimeError("terminal keyboard is not open")
        import select

        ready, _writable, _errors = select.select([self._fd], [], [], max(0.0, timeout))
        if not ready:
            return None
        data = os.read(self._fd, 1)
        return data.decode("ascii", "ignore") or None


@dataclass
class KeyboardPilotState:
    """Stick state changed by discrete keypresses."""

    step: float = 0.1
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    arm_requested: bool = False
    panic: bool = False

    def __post_init__(self) -> None:
        if not 0.01 <= self.step <= 1.0:
            raise ValueError("keyboard stick step must be in 0.01..1")

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def neutralize(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0

    def handle_key(self, key: str) -> str:
        """Apply one key and return a named event for the caller."""
        value = key.lower()
        if value in {"\r", "\n"}:
            self.arm_requested = True
            return "arm"
        if value == "x" or value == "\x03":
            self.panic = True
            self.neutralize()
            return "panic"
        if value in {" ", "c"}:
            self.neutralize()
            return "neutral"
        if value == "d":
            self.roll = self._clamp(self.roll + self.step, -1.0, 1.0)
        elif value == "a":
            self.roll = self._clamp(self.roll - self.step, -1.0, 1.0)
        elif value == "w":
            self.pitch = self._clamp(self.pitch + self.step, -1.0, 1.0)
        elif value == "s":
            self.pitch = self._clamp(self.pitch - self.step, -1.0, 1.0)
        elif value == "e":
            self.yaw = self._clamp(self.yaw + self.step, -1.0, 1.0)
        elif value == "q":
            self.yaw = self._clamp(self.yaw - self.step, -1.0, 1.0)
        elif value == "r":
            self.throttle = self._clamp(self.throttle + self.step, 0.0, 1.0)
        elif value == "f":
            self.throttle = self._clamp(self.throttle - self.step, 0.0, 1.0)
        elif value in {"h", "?"}:
            return "help"
        else:
            return "ignored"
        return "sticks"

    def sticks(self) -> RcSticks:
        return RcSticks(self.roll, self.pitch, self.yaw, self.throttle)


@dataclass(frozen=True)
class KeyboardTeleopReport:
    duration_sec: float
    requested_hz: float
    frames_sent: int
    actual_hz: float
    stop_reason: str
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


def _safe_sequence(link: CrsfLink, channel_map: CrsfChannelMap, armed: bool) -> bool:
    try:
        if armed:
            link.transport.write(
                make_rc_channels_frame(channel_map.encode(SAFE_STICKS, enabled=True))
            )
            time.sleep(0.02)
        link.transport.write(
            make_rc_channels_frame(channel_map.encode(SAFE_STICKS, enabled=False))
        )
    except OSError:
        return False
    return True


def _run_keyboard_teleop(
    transport: Transport,
    keyboard: KeyboardInput,
    *,
    hz: float,
    duration: float,
    arm_hold_sec: float,
    channel_map: CrsfChannelMap,
    stick_step: float,
    status: Callable[[str], None] | None,
) -> KeyboardTeleopReport:
    send_hz = float(hz)
    if not 0.0 < send_hz <= MAX_TELEOP_HZ:
        raise ValueError(f"hz must be greater than zero and at most {MAX_TELEOP_HZ:g}")
    seconds = float(duration)
    if not 0.0 < seconds <= MAX_TELEOP_DURATION_SEC:
        raise ValueError(
            f"duration must be greater than zero and at most "
            f"{MAX_TELEOP_DURATION_SEC:g} seconds"
        )
    hold = float(arm_hold_sec)
    if hold <= 0.5:
        raise ValueError("arm_hold_sec must be longer than the firmware's 500 ms arm hold")
    if seconds <= hold:
        raise ValueError("duration must be longer than arm_hold_sec")

    pilot = KeyboardPilotState(step=stick_step)
    link = CrsfLink(transport)
    parser = link.parser
    telemetry: dict[str, dict[str, object]] = {}
    link_statistics_frames = 0
    start = time.monotonic()
    deadline = start + seconds
    period = 1.0 / send_hz
    next_send = start
    hold_started: float | None = None
    hold_completed = False
    sent = 0
    was_armed = False
    stop_reason = "duration"
    error: str | None = None

    try:
        link.transport.write(
            make_rc_channels_frame(channel_map.encode(SAFE_STICKS, enabled=False))
        )
        sent = 1
        next_send = start + period
        while True:
            now = time.monotonic()
            if now >= deadline:
                break

            while True:
                key = keyboard.read_key(0.0)
                if key is None:
                    break
                event = pilot.handle_key(key)
                if event == "arm" and hold_started is None:
                    hold_started = now
                    if status is not None:
                        status(
                            "arm request started: throttle low for the configured hold"
                        )
                elif event == "panic":
                    stop_reason = "panic"
                    break
                elif event == "neutral" and status is not None:
                    status("sticks neutral")
                elif event == "sticks" and status is not None:
                    status(
                        f"sticks roll={pilot.roll:+.2f} pitch={pilot.pitch:+.2f} "
                        f"yaw={pilot.yaw:+.2f} throttle={pilot.throttle:.2f}"
                    )
                elif event == "help" and status is not None:
                    status(KEY_HELP)
            if stop_reason == "panic":
                break

            if hold_started is not None:
                hold_completed = now - hold_started >= hold
                enabled = True
                command = pilot.sticks() if hold_completed else SAFE_STICKS
            else:
                enabled = False
                command = SAFE_STICKS
            channels = channel_map.encode(command, enabled=enabled)
            link.transport.write(make_rc_channels_frame(channels))
            sent += 1
            was_armed = enabled

            for frame in link.poll(512, timeout=0.0):
                if frame.frame_type == CRSF_FRAMETYPE_LINK_STATISTICS:
                    link_statistics_frames += 1
                decoded = decode_telemetry(frame)
                if decoded is not None:
                    telemetry[type(decoded).__name__] = asdict(decoded)

            next_send += period
            delay = next_send - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -period:
                next_send = time.monotonic()
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    except (OSError, TimeoutError) as exc:
        stop_reason = "transport_error"
        error = str(exc)
    finally:
        disarmed = _safe_sequence(link, channel_map, was_armed)
        if not disarmed:
            error = error or "failed to write the final disarmed frame"

    end = time.monotonic()
    elapsed = max(0.001, end - start)
    return KeyboardTeleopReport(
        duration_sec=round(elapsed, 3),
        requested_hz=send_hz,
        frames_sent=sent,
        actual_hz=round(sent / elapsed, 3),
        stop_reason=stop_reason,
        arm_hold_sec=hold,
        arm_hold_completed=hold_completed,
        received_frames=parser.valid_frames,
        link_statistics_frames=link_statistics_frames,
        telemetry=telemetry,
        crc_errors=parser.crc_errors,
        resyncs=parser.resyncs,
        disarmed_at_end=disarmed,
        error=error,
        passed=sent > 0 and disarmed and error is None,
    )


def run_keyboard_teleop(
    transport: Transport,
    *,
    hz: float = 150.0,
    duration: float = 30.0,
    arm_hold_sec: float = 0.75,
    allow_write: bool = False,
    confirm_arm: bool = False,
    channel_map: CrsfChannelMap | None = None,
    stick_step: float = 0.1,
    keyboard: KeyboardInput | None = None,
    status: Callable[[str], None] | None = None,
) -> KeyboardTeleopReport:
    """Run bounded keyboard control through a CRSF transmitter.

    Both ``allow_write`` and ``confirm_arm`` are mandatory. The stream starts
    disarmed and every normal or exceptional return attempts a final disarmed
    frame. A keypress only changes the next RC frame; it never calls an arming
    API or claims that the flight controller actually armed.
    """
    if not allow_write:
        raise PermissionError("radio teleop requires explicit write authorization")
    if not confirm_arm:
        raise PermissionError("radio teleop requires explicit arm confirmation")
    channel_map = channel_map or CrsfChannelMap()
    if keyboard is None:
        with TerminalKeyboard() as terminal:
            return _run_keyboard_teleop(
                transport,
                terminal,
                hz=hz,
                duration=duration,
                arm_hold_sec=arm_hold_sec,
                channel_map=channel_map,
                stick_step=stick_step,
                status=status,
            )
    return _run_keyboard_teleop(
        transport,
        keyboard,
        hz=hz,
        duration=duration,
        arm_hold_sec=arm_hold_sec,
        channel_map=channel_map,
        stick_step=stick_step,
        status=status,
    )


__all__ = [
    "KEY_HELP",
    "KeyboardInput",
    "KeyboardPilotState",
    "KeyboardTeleopReport",
    "MAX_TELEOP_DURATION_SEC",
    "MAX_TELEOP_HZ",
    "TerminalKeyboard",
    "run_keyboard_teleop",
]
