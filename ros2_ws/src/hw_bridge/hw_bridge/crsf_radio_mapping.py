"""CRSF channel mapping shared by the radio backend and SIL receiver."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:  # ROS message types; imported lazily so the mapping logic tests without ROS
    from geometry_msgs.msg import Twist

from aerial_kit.comms.crsf import (
    BackendArmSequencer,
    CrsfChannelMap as LibraryCrsfChannelMap,
    CrsfStickState,
    RcSticks,
)
from hw_bridge.rc_mapping import RcMapParams


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


class CrsfChannelMap(LibraryCrsfChannelMap):
    """AETR + arm + flight mode channels, using one-based channel numbers."""

    def decode(self, channels: Sequence[int], params: RcMapParams) -> tuple[bool, "Twist"]:
        from geometry_msgs.msg import Twist

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
