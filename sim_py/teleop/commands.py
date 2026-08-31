"""Input-to-command mapping for teleoperation.

Kept separate from both the keyboard layer and the simulation loop so the whole
mapping is testable with plain numbers and no GUI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from aerial_kit.types import ControlTarget

from . import input_state as ks

#: ``ControlTarget.metadata`` key carrying a body-frame yaw rate in rad/s.
#: Backends that can rotate the airframe consume it; backends that cannot
#: simply ignore the extra metadata entry.
YAW_RATE_METADATA_KEY = "yaw_rate_cmd"


@dataclass(frozen=True)
class TeleopTuning:
    """Command-shaping gains and limits."""

    accel_xy: float = 6.0
    accel_z: float = 5.0
    vel_damp: float = 1.2
    max_speed_xy: float = 9.0
    max_speed_z: float = 5.0
    acc_max: float = 14.0
    yaw_rate: float = 1.4
    max_yaw_rate: float = 2.5

    @classmethod
    def from_config(cls, controller_cfg: Mapping[str, Any] | None) -> "TeleopTuning":
        cfg = dict((controller_cfg or {}).get("teleop", {}) or {})
        defaults = cls()
        acc_max = cfg.get("acc_max", (controller_cfg or {}).get("acc_max", defaults.acc_max))
        return cls(
            accel_xy=float(cfg.get("accel_xy", defaults.accel_xy)),
            accel_z=float(cfg.get("accel_z", defaults.accel_z)),
            vel_damp=float(cfg.get("vel_damp", defaults.vel_damp)),
            max_speed_xy=float(cfg.get("max_speed_xy", defaults.max_speed_xy)),
            max_speed_z=float(cfg.get("max_speed_z", defaults.max_speed_z)),
            acc_max=float(acc_max),
            yaw_rate=float(cfg.get("yaw_rate", defaults.yaw_rate)),
            max_yaw_rate=float(cfg.get("max_yaw_rate", defaults.max_yaw_rate)),
        )


@dataclass(frozen=True)
class InputAxes:
    """Normalized pilot demand, each component in ``[-1, 1]``."""

    forward: float = 0.0
    right: float = 0.0
    up: float = 0.0
    yaw: float = 0.0

    @property
    def is_neutral(self) -> bool:
        return not any((self.forward, self.right, self.up, self.yaw))


@dataclass(frozen=True)
class TeleopCommand:
    """World-frame acceleration plus a physical yaw rate."""

    accel_cmd: np.ndarray
    yaw_rate_cmd: float
    axes: InputAxes = field(default_factory=InputAxes)

    def to_control_target(self) -> ControlTarget:
        return ControlTarget(
            accel_cmd=np.asarray(self.accel_cmd, dtype=float).copy(),
            metadata={"controller": "teleop", YAW_RATE_METADATA_KEY: float(self.yaw_rate_cmd)},
        )


def neutral_command() -> TeleopCommand:
    return TeleopCommand(accel_cmd=np.zeros(3, dtype=float), yaw_rate_cmd=0.0, axes=InputAxes())


def axes_from_keys(keyboard: ks.KeyboardState) -> InputAxes:
    """Reduce latched keys to a normalized axis demand.

    Opposing keys cancel, so pressing both W and S yields zero forward demand.
    """
    forward = float(keyboard.is_active(ks.FORWARD)) - float(keyboard.is_active(ks.BACKWARD))
    right = float(keyboard.is_active(ks.RIGHT)) - float(keyboard.is_active(ks.LEFT))
    up = float(keyboard.is_active(ks.CLIMB)) - float(keyboard.is_active(ks.DESCEND))
    yaw = float(keyboard.is_active(ks.YAW_LEFT)) - float(keyboard.is_active(ks.YAW_RIGHT))
    return InputAxes(forward=forward, right=right, up=up, yaw=yaw)


def body_to_world_xy(yaw: float) -> tuple[np.ndarray, np.ndarray]:
    """Unit ENU forward and right vectors for a vehicle at heading ``yaw``."""
    cos_yaw = float(np.cos(yaw))
    sin_yaw = float(np.sin(yaw))
    forward = np.array([cos_yaw, sin_yaw, 0.0], dtype=float)
    right = np.array([sin_yaw, -cos_yaw, 0.0], dtype=float)
    return forward, right


def command_from_axes(
    axes: InputAxes,
    *,
    yaw: float,
    velocity: np.ndarray,
    tuning: TeleopTuning,
) -> TeleopCommand:
    """Turn an axis demand into a world-frame acceleration and yaw rate.

    Demand is interpreted in the body frame at the vehicle's *actual* heading,
    then damped against current velocity so releasing every key settles the
    vehicle instead of leaving it drifting.
    """
    velocity = np.asarray(velocity, dtype=float).reshape(3)
    forward_axis, right_axis = body_to_world_xy(yaw)

    accel = forward_axis * (tuning.accel_xy * axes.forward)
    accel = accel + right_axis * (tuning.accel_xy * axes.right)
    accel[2] += tuning.accel_z * axes.up

    accel -= tuning.vel_damp * velocity

    horizontal_speed = float(np.linalg.norm(velocity[:2]))
    if horizontal_speed > tuning.max_speed_xy:
        overspeed = horizontal_speed - tuning.max_speed_xy
        accel[:2] -= (velocity[:2] / horizontal_speed) * tuning.vel_damp * overspeed
    vertical_speed = float(velocity[2])
    if abs(vertical_speed) > tuning.max_speed_z:
        overspeed = abs(vertical_speed) - tuning.max_speed_z
        accel[2] -= np.sign(vertical_speed) * tuning.vel_damp * overspeed

    magnitude = float(np.linalg.norm(accel))
    if tuning.acc_max > 1e-6 and magnitude > tuning.acc_max:
        accel *= tuning.acc_max / magnitude

    yaw_rate = float(
        np.clip(tuning.yaw_rate * axes.yaw, -tuning.max_yaw_rate, tuning.max_yaw_rate)
    )
    return TeleopCommand(accel_cmd=accel, yaw_rate_cmd=yaw_rate, axes=axes)


def command_from_keys(
    keyboard: ks.KeyboardState,
    *,
    yaw: float,
    velocity: np.ndarray,
    tuning: TeleopTuning,
) -> TeleopCommand:
    """Full input-to-command pipeline for one simulation step."""
    return command_from_axes(
        axes_from_keys(keyboard), yaw=yaw, velocity=velocity, tuning=tuning
    )


def describe_axes(axes: InputAxes) -> str:
    """Compact HUD rendering of the current pilot demand."""

    def token(value: float, negative: str, positive: str) -> str:
        if value > 0:
            return positive
        if value < 0:
            return negative
        return "----"

    return " ".join(
        (
            token(axes.forward, "back", "fwd "),
            token(axes.right, "left", "rght"),
            token(axes.up, "down", "up  "),
            token(axes.yaw, "yawR", "yawL"),
        )
    )


__all__ = [
    "InputAxes",
    "TeleopCommand",
    "TeleopTuning",
    "YAW_RATE_METADATA_KEY",
    "axes_from_keys",
    "body_to_world_xy",
    "command_from_axes",
    "command_from_keys",
    "describe_axes",
    "neutral_command",
]
