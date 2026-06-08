"""Map body-frame velocity + yaw-rate to normalized Betaflight Angle-mode RC sticks.

Sticks: roll/pitch/yaw in [-1, 1], throttle in [0, 1].
Body frame matches mission_executor: x = right, y = forward (nose).
Yaw is NOT inverted here — the TX firmware inverts it (see docs/HARDWARE.md).
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RcMapParams:
    kv_xy: float
    max_tilt_deg: float
    hover_throttle: float
    kz: float
    throttle_min: float
    throttle_max: float
    max_yaw_rate_rps: float


@dataclass(frozen=True)
class RcSticks:
    roll: float
    pitch: float
    yaw: float
    throttle: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def velocity_to_rc(params: RcMapParams, vx: float, vy: float, vz: float, wz: float) -> RcSticks:
    max_tilt = math.radians(params.max_tilt_deg)
    roll_angle = _clamp(params.kv_xy * vx, -max_tilt, max_tilt)
    pitch_angle = _clamp(params.kv_xy * vy, -max_tilt, max_tilt)
    roll = roll_angle / max_tilt if max_tilt > 0 else 0.0
    pitch = pitch_angle / max_tilt if max_tilt > 0 else 0.0
    throttle = _clamp(params.hover_throttle + params.kz * vz, params.throttle_min, params.throttle_max)
    yaw = _clamp(wz / params.max_yaw_rate_rps, -1.0, 1.0) if params.max_yaw_rate_rps > 0 else 0.0
    return RcSticks(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)


def neutral_sticks(params: RcMapParams) -> RcSticks:
    return RcSticks(roll=0.0, pitch=0.0, yaw=0.0, throttle=params.hover_throttle)
