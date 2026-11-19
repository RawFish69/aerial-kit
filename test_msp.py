from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from aerial_kit.comms import (
    Msp,
    MspError,
    MspProtocolError,
    build_msp,
    describe_msp,
    identify_msp,
    msp_state,
)
from aerial_kit.comms import msp

ROOT = Path(__file__).resolve().parent
FAKE = ROOT / "firmware/tools/msp_fake_board.py"


class Pipe:
    def __init__(self) -> None:
        self.process = subprocess.Popen(
            [sys.executable, str(FAKE)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )

    def read(self, count: int) -> bytes:
        assert self.process.stdout is not None
        return self.process.stdout.read(count)

    def write(self, data: bytes) -> None:
        assert self.process.stdin is not None
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def close(self) -> None:
        self.process.kill()
        self.process.wait(timeout=2)


def open_board() -> tuple[Pipe, Msp]:
    pipe = Pipe()
    reader = msp.Deadline(pipe)
    return pipe, Msp(reader.read_some, pipe.write, timeout=2.0)


def test_msp_frame_and_identity_against_the_fake_board() -> None:
    pipe, client = open_board()
    try:
        assert build_msp(msp.MSP_API_VERSION) == b"$M<\x00\x01\x01"
        identity = identify_msp(client)
        assert identity["variant"] == "BTFL"
        assert identity["version"]["text"] == "2026.6.1"
        assert "Betaflight 2026.6.1" in describe_msp(identity)
        telemetry = msp_state(client)
        assert telemetry["vbat_v"] == pytest.approx(12.6, abs=0.05)
        assert telemetry["gps_satellites"] == 11
    finally:
        pipe.close()


def test_msp_settings_and_refusal_are_typed() -> None:
    pipe, client = open_board()
    try:
        assert client.setting("failsafe_throttle") == "failsafe_throttle = 1050"
        assert client.set_setting("failsafe_throttle", 1200) == \
            "failsafe_throttle = 1200"
        with pytest.raises(MspError):
            client.set_setting("failsafe_throttle", 9999)
    finally:
        pipe.close()


def test_msp_rejects_malformed_or_truncated_frames() -> None:
    with pytest.raises(MspProtocolError):
        msp.parse(b"$M>\x04\x01\x00\x01\x30\xFF")
    with pytest.raises(MspProtocolError):
        msp.parse(b"$M>\x04\x01\x00\x01")


def test_firmware_tool_is_a_thin_importable_wrapper() -> None:
    result = subprocess.run(
        [sys.executable, str(ROOT / "firmware/tools/msp.py"), "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "--port" in result.stdout
