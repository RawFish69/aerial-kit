"""CRSF channel mapping shared by the radio backend and SIL receiver."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from geometry_msgs.msg import Twist

from aerial_kit.comms.crsf import (
    CRSF_CHANNEL_COUNT,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MID,
    CRSF_CHANNEL_MIN,
)
from hw_bridge.rc_mapping import RcMapParams, RcSticks


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class CrsfChannelMap:
    """AETR + arm + flight mode channels, using one-based channel numbers."""

    roll: int = 1
    pitch: int = 2
    throttle: int = 3
    yaw: int = 4
    arm: int = 5
    mode: int = 6
    arm_threshold: int = 1000
    angle_mode_count: int = CRSF_CHANNEL_MID
    disabled_mode_count: int = CRSF_CHANNEL_MIN

    def __post_init__(self) -> None:
        channels = (self.roll, self.pitch, self.throttle, self.yaw, self.arm, self.mode)
        if any(channel < 1 or channel > CRSF_CHANNEL_COUNT for channel in channels):
            raise ValueError(f"CRSF channel numbers must be 1..{CRSF_CHANNEL_COUNT}, got {channels}")
        if len(set(channels)) != len(channels):
            raise ValueError(f"CRSF channel map uses a channel twice: {channels}")

    @classmethod
    def from_mapping(cls, values: dict[str, int]) -> "CrsfChannelMap":
        return cls(**{name: int(values.get(name, getattr(cls(), name))) for name in (
            "roll", "pitch", "throttle", "yaw", "arm", "mode"
        )})

    @staticmethod
    def _stick_count(value: float) -> int:
        normalized = _clamp(value, -1.0, 1.0)
        return round(CRSF_CHANNEL_MID + normalized * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID))

    @staticmethod
    def _throttle_count(value: float) -> int:
        normalized = _clamp(value, 0.0, 1.0)
        return round(CRSF_CHANNEL_MIN + normalized * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN))

    def encode(self, sticks: RcSticks, *, enabled: bool) -> tuple[int, ...]:
        channels = [CRSF_CHANNEL_MID] * CRSF_CHANNEL_COUNT
        channels[self.roll - 1] = self._stick_count(sticks.roll)
        channels[self.pitch - 1] = self._stick_count(sticks.pitch)
        channels[self.yaw - 1] = self._stick_count(sticks.yaw)
        channels[self.throttle - 1] = self._throttle_count(sticks.throttle)
        channels[self.arm - 1] = CRSF_CHANNEL_MAX if enabled else CRSF_CHANNEL_MIN
        channels[self.mode - 1] = self.angle_mode_count if enabled else self.disabled_mode_count
        return tuple(channels)

    def decode(self, channels: Sequence[int], params: RcMapParams) -> tuple[bool, Twist]:
        if len(channels) != CRSF_CHANNEL_COUNT:
            raise ValueError(f"expected {CRSF_CHANNEL_COUNT} CRSF channels, got {len(channels)}")

        def stick(number: int) -> float:
            return _clamp(
                (float(channels[number - 1]) - CRSF_CHANNEL_MID)
                / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID),
                -1.0,
                1.0,
            )

        enabled = int(channels[self.arm - 1]) >= self.arm_threshold
        command = Twist()
        if not enabled:
            return False, command

        roll = stick(self.roll)
        pitch = stick(self.pitch)
        yaw = stick(self.yaw)
        throttle = _clamp(
            (float(channels[self.throttle - 1]) - CRSF_CHANNEL_MIN)
            / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN),
            0.0,
            1.0,
        )

        max_tilt = params.max_tilt_deg * 3.141592653589793 / 180.0
        if params.kv_xy == 0.0:
            raise ValueError("kv_xy must be non-zero to decode CRSF velocity commands")
        command.linear.x = roll * max_tilt / params.kv_xy
        command.linear.y = pitch * max_tilt / params.kv_xy
        command.linear.z = (throttle - params.hover_throttle) / params.kz if params.kz else 0.0
        command.angular.z = yaw * params.max_yaw_rate_rps
        return True, command
