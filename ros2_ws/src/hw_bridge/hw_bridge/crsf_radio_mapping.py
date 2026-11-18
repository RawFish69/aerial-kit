"""CRSF channel mapping shared by the radio backend and SIL receiver."""

from __future__ import annotations

import math
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


def rotate_world_velocity_to_body(vx: float, vy: float, yaw_rad: float) -> tuple[float, float]:
    """Rotate ENU world velocity into controller stick right/forward axes.

    The multirotor controller defines heading zero along world +X and its right
    vector as ``[sin(yaw), -cos(yaw)]``.
    """
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        sine * float(vx) - cosine * float(vy),
        cosine * float(vx) + sine * float(vy),
    )


def velocity_to_manual_sticks(
    right_mps: float,
    forward_mps: float,
    climb_mps: float,
    yaw_rate_rps: float,
    *,
    max_speed_mps: float,
    max_climb_mps: float,
    max_yaw_rate_rps: float,
) -> RcSticks:
    """Normalize body velocity into velocity-mode manual sticks."""
    return RcSticks(
        roll=_clamp(right_mps / max(max_speed_mps, 1e-6), -1.0, 1.0),
        pitch=_clamp(forward_mps / max(max_speed_mps, 1e-6), -1.0, 1.0),
        yaw=_clamp(yaw_rate_rps / max(max_yaw_rate_rps, 1e-6), -1.0, 1.0),
        throttle=_clamp(0.5 + climb_mps / max(2.0 * max_climb_mps, 1e-6), 0.0, 1.0),
    )


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


@dataclass
class ReceiverArmingGate:
    """Model the FC rule: arm high and throttle low for a continuous hold."""

    arm_hold_sec: float = 0.5
    throttle_low: float = 0.05
    armed: bool = False
    hold_started: float | None = None

    def reset(self) -> None:
        self.armed = False
        self.hold_started = None

    def update(self, arm_request: bool, throttle: float, now: float) -> bool:
        if not arm_request:
            self.reset()
            return False
        if self.armed:
            return True
        if throttle > self.throttle_low:
            self.hold_started = None
            return False
        if self.hold_started is None:
            self.hold_started = now
        if now - self.hold_started >= self.arm_hold_sec:
            self.armed = True
        return self.armed


@dataclass(frozen=True)
class CrsfChannelMap:
    """AETR + arm + flight mode channels, using one-based channel numbers."""

    roll: int = 1
    pitch: int = 2
    throttle: int = 3
    yaw: int = 4
    arm: int = 6
    mode: int = 5
    arm_threshold: int = 1300
    angle_mode_count: int = CRSF_CHANNEL_MAX
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

    def decode_sticks(self, channels: Sequence[int]) -> CrsfStickState:
        if len(channels) != CRSF_CHANNEL_COUNT:
            raise ValueError(f"expected {CRSF_CHANNEL_COUNT} CRSF channels, got {len(channels)}")

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

    def decode(self, channels: Sequence[int], params: RcMapParams) -> tuple[bool, Twist]:
        decoded = self.decode_sticks(channels)
        command = Twist()
        if not decoded.arm_request:
            return False, command

        sticks = decoded.sticks
        max_tilt = params.max_tilt_deg * 3.141592653589793 / 180.0
        if params.kv_xy == 0.0:
            raise ValueError("kv_xy must be non-zero to decode CRSF velocity commands")
        command.linear.x = sticks.roll * max_tilt / params.kv_xy
        command.linear.y = sticks.pitch * max_tilt / params.kv_xy
        command.linear.z = (
            (sticks.throttle - params.hover_throttle) / params.kz if params.kz else 0.0
        )
        command.angular.z = sticks.yaw * params.max_yaw_rate_rps
        return True, command
