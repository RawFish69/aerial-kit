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
    """World-frame acceleration plus a physical yaw rate.

    ``actuator_cmd``, when set, carries a fixed-wing ``[throttle_L, throttle_R,
    elevon_L, elevon_R]`` command instead -- see :func:`fixedwing_actuator_from_axes`.
    A single command type (rather than a per-airframe subclass) keeps the
    engine/renderer/HUD code that only cares about ``axes`` airframe-agnostic.
    """

    accel_cmd: np.ndarray
    yaw_rate_cmd: float
    axes: InputAxes = field(default_factory=InputAxes)
    actuator_cmd: np.ndarray | None = None

    def to_control_target(self) -> ControlTarget:
        if self.actuator_cmd is not None:
            return ControlTarget(
                accel_cmd=np.zeros(3, dtype=float),
                metadata={
                    "controller": "teleop",
                    "actuator_cmd": np.asarray(self.actuator_cmd, dtype=float).copy(),
                },
            )
        return ControlTarget(
            accel_cmd=np.asarray(self.accel_cmd, dtype=float).copy(),
            metadata={"controller": "teleop", YAW_RATE_METADATA_KEY: float(self.yaw_rate_cmd)},
        )


def neutral_command() -> TeleopCommand:
    return TeleopCommand(accel_cmd=np.zeros(3, dtype=float), yaw_rate_cmd=0.0, axes=InputAxes())


@dataclass(frozen=True)
class FixedWingTeleopTuning:
    """Stick-style gains for the twin-wing's direct actuator command.

    Values default to :class:`~aerial_kit.airframes.fixed_wing.TwinWingAirframe`'s
    own defaults (``trim_throttle_n=3.0``, ``max_thrust_n=10.0``,
    ``max_elevon_rad=25 deg``) so a freshly launched aircraft is already near
    its trimmed cruise thrust rather than idling at zero.
    """

    throttle_trim_n: float = 3.0
    throttle_range_n: float = 3.5
    #: Full deflection at 20 deg (TwinWingAirframe's own max_elevon_rad) snaps
    #: the nose to -70+ deg within half a second at cruise speed -- this
    #: model's inertia (Iyy=0.03 kg*m^2) is small enough that cm_delta_e=0.6
    #: produces a very large angular acceleration per radian of elevon. 10 deg
    #: keeps full-stick controllable rather than a bang-bang snap.
    elevon_max_rad: float = np.radians(10.0)
    yaw_diff_n: float = 1.0
    #: TwinWingAirframe.trim_elevon_rad (2 deg) was calibrated for its own
    #: allocator, which also carries a closed pitch loop; applied open-loop
    #: here it overcorrects into a climbing divergence just as fast as no trim
    #: undercorrects into a gentle nose-down drift (roughly -5 deg over the
    #: first 1.2 s either way, given this airframe's inertia). Leaving it at
    #: zero keeps that drift small and in the direction a forward-stick tap
    #: corrects naturally, rather than swapping which direction the pilot has
    #: to counter.
    trim_elevon_rad: float = 0.0

    @property
    def throttle_max_n(self) -> float:
        return self.throttle_trim_n + self.throttle_range_n

    @classmethod
    def from_config(cls, controller_cfg: Mapping[str, Any] | None) -> "FixedWingTeleopTuning":
        cfg = dict((controller_cfg or {}).get("teleop_fixedwing", {}) or {})
        defaults = cls()
        return cls(
            throttle_trim_n=float(cfg.get("throttle_trim_n", defaults.throttle_trim_n)),
            throttle_range_n=float(cfg.get("throttle_range_n", defaults.throttle_range_n)),
            elevon_max_rad=float(cfg.get("elevon_max_rad", defaults.elevon_max_rad)),
            yaw_diff_n=float(cfg.get("yaw_diff_n", defaults.yaw_diff_n)),
            trim_elevon_rad=float(cfg.get("trim_elevon_rad", defaults.trim_elevon_rad)),
        )


def fixedwing_actuator_from_axes(
    axes: InputAxes, *, tuning: FixedWingTeleopTuning
) -> np.ndarray:
    """RC-plane stick feel: ``[throttle_L, throttle_R, elevon_L, elevon_R]``.

    The twin-wing has no rudder, so ``axes.yaw`` (Q/E) is repurposed as a small
    differential-thrust nudge rather than dropped -- useful for damping yaw in
    a turn. Axis-to-control mapping:

    * ``up``/``down`` (Space/Shift): throttle, symmetric around trim.
    * ``forward``/``backward`` (W/S): elevator (pitch). Forward = nose down,
      matching the stick-forward-dives flight-sim convention.
    * ``right``/``left`` (D/A): aileron via differential elevon (bank/roll).
    * ``yaw`` (Q/E): differential thrust, a mild rudder substitute.

    Elevon sign matches ``cm_delta_e``/``cl_delta_a`` in
    :class:`~aerial_kit.dynamics.fixed_wing.FixedWingParams`: positive elevon
    is nose-up, and positive ``elevon_L - elevon_R`` rolls right.
    """
    throttle = tuning.throttle_trim_n + tuning.throttle_range_n * axes.up
    throttle = float(np.clip(throttle, 0.0, tuning.throttle_max_n))
    yaw_bias = tuning.yaw_diff_n * axes.yaw
    throttle_l = float(np.clip(throttle - yaw_bias, 0.0, tuning.throttle_max_n))
    throttle_r = float(np.clip(throttle + yaw_bias, 0.0, tuning.throttle_max_n))

    delta_e = tuning.trim_elevon_rad - tuning.elevon_max_rad * axes.forward
    delta_a = tuning.elevon_max_rad * axes.right
    elevon_max = tuning.elevon_max_rad + tuning.trim_elevon_rad
    elevon_l = float(np.clip(delta_e + delta_a, -elevon_max, elevon_max))
    elevon_r = float(np.clip(delta_e - delta_a, -elevon_max, elevon_max))

    return np.array([throttle_l, throttle_r, elevon_l, elevon_r], dtype=float)


def fixedwing_command_from_keys(
    keyboard: ks.KeyboardState, *, tuning: FixedWingTeleopTuning
) -> TeleopCommand:
    """Full input-to-command pipeline for one fixed-wing simulation step."""
    axes = axes_from_keys(keyboard)
    actuator = fixedwing_actuator_from_axes(axes, tuning=tuning)
    return TeleopCommand(
        accel_cmd=np.zeros(3, dtype=float), yaw_rate_cmd=0.0, axes=axes, actuator_cmd=actuator
    )


def neutral_fixedwing_command(tuning: FixedWingTeleopTuning) -> TeleopCommand:
    """Trimmed-thrust, trimmed-elevon command: used while paused/unfocused.

    Unlike the multirotor's ``neutral_command`` (thrust off, it can hover in
    place), a fixed wing that lost all thrust would stall and fall -- so idle
    still means cruise thrust and trim elevon, matching a centred stick.
    """
    actuator = np.array(
        [
            tuning.throttle_trim_n,
            tuning.throttle_trim_n,
            tuning.trim_elevon_rad,
            tuning.trim_elevon_rad,
        ],
        dtype=float,
    )
    return TeleopCommand(
        accel_cmd=np.zeros(3, dtype=float), yaw_rate_cmd=0.0, axes=InputAxes(), actuator_cmd=actuator
    )


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
    "FixedWingTeleopTuning",
    "InputAxes",
    "TeleopCommand",
    "TeleopTuning",
    "YAW_RATE_METADATA_KEY",
    "axes_from_keys",
    "body_to_world_xy",
    "command_from_axes",
    "command_from_keys",
    "describe_axes",
    "fixedwing_actuator_from_axes",
    "fixedwing_command_from_keys",
    "neutral_command",
    "neutral_fixedwing_command",
]
