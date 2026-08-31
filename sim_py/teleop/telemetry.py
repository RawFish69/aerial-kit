"""Session recording for teleop flights."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from aerial_kit.types import SimState

from .commands import TeleopCommand
from .model import quat_to_euler_rpy

COLUMNS = (
    "t",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "roll",
    "pitch",
    "yaw",
    "ax_cmd",
    "ay_cmd",
    "az_cmd",
    "yaw_rate_cmd",
    "axis_forward",
    "axis_right",
    "axis_up",
    "axis_yaw",
    "collision",
)


@dataclass
class TelemetryRecorder:
    """Accumulates one row per recorded simulation sample."""

    stride: int = 5
    max_samples: int = 200_000
    rows: list[tuple[float, ...]] = field(default_factory=list)
    trail: list[np.ndarray] = field(default_factory=list)
    trail_max: int = 4000

    _counter: int = field(default=0, init=False)

    def record(self, state: SimState, command: TeleopCommand, *, collision: bool = False) -> None:
        position = np.asarray(state.position, dtype=float).reshape(3)
        self._append_trail(position)

        self._counter += 1
        if self._counter % max(1, self.stride) != 0:
            return
        if len(self.rows) >= self.max_samples:
            return

        velocity = np.asarray(state.velocity, dtype=float).reshape(3)
        roll, pitch, yaw = quat_to_euler_rpy(state.attitude_quat)
        accel = np.asarray(command.accel_cmd, dtype=float).reshape(3)
        self.rows.append(
            (
                float(state.t),
                *position.tolist(),
                *velocity.tolist(),
                roll,
                pitch,
                yaw,
                *accel.tolist(),
                float(command.yaw_rate_cmd),
                float(command.axes.forward),
                float(command.axes.right),
                float(command.axes.up),
                float(command.axes.yaw),
                float(bool(collision)),
            )
        )

    def _append_trail(self, position: np.ndarray) -> None:
        if self.trail and float(np.linalg.norm(position - self.trail[-1])) < 0.05:
            return
        self.trail.append(position.copy())
        if len(self.trail) > self.trail_max:
            # Halve the trail instead of popping every frame once it is full.
            del self.trail[: self.trail_max // 2]

    def trail_array(self) -> np.ndarray:
        if not self.trail:
            return np.zeros((0, 3), dtype=float)
        return np.vstack(self.trail)

    def to_array(self) -> np.ndarray:
        if not self.rows:
            return np.zeros((0, len(COLUMNS)), dtype=float)
        return np.asarray(self.rows, dtype=float)

    def save_csv(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(COLUMNS)
            writer.writerows(self.rows)
        return target


__all__ = ["COLUMNS", "TelemetryRecorder"]
