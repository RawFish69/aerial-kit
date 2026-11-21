from __future__ import annotations

import os
import pty
import sys

import pytest

from aerial_kit.comms.crsf import (
    CRSF_CHANNEL_MID,
    CRSF_CHANNEL_MIN,
    CrsfChannelMap,
    CrsfParser,
    unpack_channels,
)
from aerial_kit.comms.teleop import (
    KeyboardPilotState,
    TerminalKeyboard,
    run_keyboard_teleop,
)
from aerial_kit.comms.transport import MemoryTransport


class FakeKeyboard:
    def __init__(self, keys: str = "") -> None:
        self._keys = list(keys)

    def read_key(self, timeout: float = 0.0) -> str | None:
        if not self._keys:
            return None
        return self._keys.pop(0)


def _rc_frames(transport: MemoryTransport) -> list[tuple[int, ...]]:
    return [
        unpack_channels(frame.payload)
        for frame in CrsfParser().feed(transport.drain_tx())
        if frame.frame_type == 0x16
    ]


def test_keyboard_state_clamps_sticks_and_panic_neutralises() -> None:
    pilot = KeyboardPilotState(step=0.4)
    for key in "dddrwwwq":
        pilot.handle_key(key)
    assert pilot.sticks().roll == pytest.approx(1.0)
    assert pilot.sticks().pitch == pytest.approx(1.0)
    assert pilot.sticks().throttle == pytest.approx(0.4)
    assert pilot.sticks().yaw == pytest.approx(-0.4)
    assert pilot.handle_key("x") == "panic"
    assert pilot.panic is True
    assert pilot.sticks().roll == 0.0
    assert pilot.sticks().throttle == 0.0


def test_keyboard_teleop_arms_on_enter_and_finishes_disarmed() -> None:
    transport = MemoryTransport(timeout=0.0)
    messages: list[str] = []
    report = run_keyboard_teleop(
        transport,
        hz=100,
        duration=0.75,
        arm_hold_sec=0.55,
        allow_write=True,
        confirm_arm=True,
        channel_map=CrsfChannelMap(arm=5, mode=7),
        keyboard=FakeKeyboard("\nrd"),
        status=messages.append,
    )
    assert report.passed
    assert report.stop_reason == "duration"
    assert report.arm_hold_completed
    assert report.disarmed_at_end
    assert "arm request started: throttle low for the configured hold" in messages

    channels = _rc_frames(transport)
    channel_map = CrsfChannelMap(arm=5, mode=7)
    assert channels[0][channel_map.arm - 1] == CRSF_CHANNEL_MIN
    armed = [
        values
        for values in channels
        if values[channel_map.arm - 1] >= channel_map.arm_threshold
    ]
    assert armed
    assert armed[0][channel_map.throttle - 1] == CRSF_CHANNEL_MIN
    command = [
        values
        for values in armed
        if values[channel_map.throttle - 1] > CRSF_CHANNEL_MIN
    ]
    assert command
    assert command[-1][channel_map.roll - 1] > CRSF_CHANNEL_MID
    assert command[-1][channel_map.throttle - 1] > CRSF_CHANNEL_MIN
    assert channels[-1][channel_map.arm - 1] < channel_map.arm_threshold
    assert channels[-1][channel_map.throttle - 1] == CRSF_CHANNEL_MIN


def test_keyboard_panic_stops_sending_armed_sticks() -> None:
    transport = MemoryTransport(timeout=0.0)
    report = run_keyboard_teleop(
        transport,
        hz=100,
        duration=0.75,
        arm_hold_sec=0.55,
        allow_write=True,
        confirm_arm=True,
        keyboard=FakeKeyboard("\nx"),
    )
    assert report.stop_reason == "panic"
    assert report.passed
    channels = _rc_frames(transport)
    assert channels[-1][CrsfChannelMap().arm - 1] < CrsfChannelMap().arm_threshold


def test_keyboard_teleop_requires_both_safety_flags_before_output() -> None:
    transport = MemoryTransport(timeout=0.0)
    with pytest.raises(PermissionError, match="write"):
        run_keyboard_teleop(transport, keyboard=FakeKeyboard())
    with pytest.raises(PermissionError, match="arm confirmation"):
        run_keyboard_teleop(
            transport,
            allow_write=True,
            keyboard=FakeKeyboard(),
        )
    assert transport.drain_tx() == b""


def test_keyboard_teleop_refuses_noninteractive_terminal_before_output() -> None:
    transport = MemoryTransport(timeout=0.0)
    with pytest.raises(ValueError, match="interactive terminal"):
        run_keyboard_teleop(
            transport,
            allow_write=True,
            confirm_arm=True,
        )
    assert transport.drain_tx() == b""


def test_terminal_keyboard_reads_one_keypress(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    master, slave = pty.openpty()
    stream = os.fdopen(slave, "r", encoding="ascii", buffering=1)
    monkeypatch.setattr(sys, "stdin", stream)
    try:
        with TerminalKeyboard() as keyboard:
            os.write(master, b"w")
            assert keyboard.read_key(0.5) == "w"
    finally:
        stream.close()
        os.close(master)
