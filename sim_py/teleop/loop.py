"""Fixed-step real-time scheduling for the teleop session.

The GUI timer fires at an unreliable rate, so wall-clock progress is measured
with :func:`time.monotonic` and converted into a whole number of fixed-size
simulation steps. Catch-up is bounded: after the process is starved (window
dragged, breakpoint hit) the simulation skips ahead in wall-clock terms rather
than running thousands of steps in one frame.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

Clock = Callable[[], float]


@dataclass
class FixedStepScheduler:
    """Convert elapsed wall-clock time into bounded fixed simulation steps."""

    dt: float
    target_fps: float = 30.0
    max_catch_up_s: float = 0.25
    clock: Clock = time.monotonic
    smoothing: float = 0.15

    frames: int = field(default=0, init=False)
    steps: int = field(default=0, init=False)
    dropped_steps: int = field(default=0, init=False)
    frame_rate: float = field(default=0.0, init=False)
    real_time_factor: float = field(default=0.0, init=False)

    _accumulator: float = field(default=0.0, init=False)
    _last_tick: float | None = field(default=None, init=False)
    _started_at: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("FixedStepScheduler requires a positive dt")
        if self.target_fps <= 0.0:
            raise ValueError("FixedStepScheduler requires a positive target_fps")

    @property
    def interval_ms(self) -> int:
        return resolve_interval_ms(self.target_fps)

    @property
    def max_steps_per_tick(self) -> int:
        return max(1, int(self.max_catch_up_s / self.dt))

    @property
    def elapsed_wall_s(self) -> float:
        if self._started_at is None:
            return 0.0
        return max(0.0, self.clock() - self._started_at)

    def start(self) -> None:
        now = self.clock()
        self._started_at = now
        self._last_tick = now
        self._accumulator = 0.0

    def tick(self) -> int:
        """Advance the scheduler one frame and return how many steps to run."""
        now = self.clock()
        if self._last_tick is None:
            self.start()
        frame_dt = max(0.0, now - float(self._last_tick if self._last_tick is not None else now))
        self._last_tick = now
        self.frames += 1

        if frame_dt > 0.0:
            instantaneous = 1.0 / frame_dt
            if self.frame_rate <= 0.0:
                self.frame_rate = instantaneous
            else:
                self.frame_rate += self.smoothing * (instantaneous - self.frame_rate)

        self._accumulator += frame_dt
        # Nudge past floating-point shortfall so an exact multiple of dt does
        # not lose a step (0.02 s of wall clock must yield two 0.01 s steps).
        steps = int(self._accumulator / self.dt + 1e-9)
        limit = self.max_steps_per_tick
        if steps > limit:
            self.dropped_steps += steps - limit
            steps = limit
            self._accumulator = 0.0
        else:
            self._accumulator = max(0.0, self._accumulator - steps * self.dt)

        self.steps += steps
        if frame_dt > 0.0:
            achieved = (steps * self.dt) / frame_dt
            if self.real_time_factor <= 0.0:
                self.real_time_factor = achieved
            else:
                self.real_time_factor += self.smoothing * (achieved - self.real_time_factor)
        return steps

    def pause_drift(self) -> None:
        """Re-baseline the clock without generating steps (used while paused)."""
        self._last_tick = self.clock()
        self._accumulator = 0.0


def resolve_interval_ms(target_fps: float) -> int:
    """GUI timer interval for a target render rate, never below 1 ms."""
    if target_fps <= 0.0:
        raise ValueError("target_fps must be positive")
    return max(1, int(round(1000.0 / target_fps)))


__all__ = ["Clock", "FixedStepScheduler", "resolve_interval_ms"]
