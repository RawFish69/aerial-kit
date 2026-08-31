"""Headless teleop physics engine.

The engine holds the keyboard state, the command mapping, the dynamics backend
and the telemetry recorder. It has no Matplotlib dependency, so a full flight
can be driven from a test by pressing keys and calling :meth:`TeleopEngine.advance`.
"""

from __future__ import annotations

import numpy as np

from aerial_kit.interfaces import DynamicsBackend
from aerial_kit.types import SimState

from .commands import TeleopCommand, TeleopTuning, command_from_keys, neutral_command
from .input_state import KeyboardState
from .model import quat_to_euler_rpy, rotor_thrust_fractions
from .telemetry import TelemetryRecorder
from .world import TeleopWorld


class TeleopEngine:
    """Fixed-step teleop simulation with no GUI attached."""

    def __init__(
        self,
        *,
        backend: DynamicsBackend,
        world: TeleopWorld,
        tuning: TeleopTuning,
        dt: float,
        keyboard: KeyboardState | None = None,
        telemetry: TelemetryRecorder | None = None,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("TeleopEngine requires a positive dt")
        self.backend = backend
        self.world = world
        self.tuning = tuning
        self.dt = float(dt)
        self.keyboard = keyboard if keyboard is not None else KeyboardState()
        self.telemetry = telemetry if telemetry is not None else TelemetryRecorder()

        self.command: TeleopCommand = neutral_command()
        self.colliding = False
        self.steps_taken = 0
        self._state = self.backend.state()
        self.telemetry.record(self._state, self.command, collision=False)

    @property
    def state(self) -> SimState:
        return self._state

    @property
    def yaw(self) -> float:
        return quat_to_euler_rpy(self._state.attitude_quat)[2]

    def current_command(self) -> TeleopCommand:
        """Command implied by the current keyboard state and vehicle state.

        Held inputs are ignored while paused or unfocused so an unattended
        window can never keep flying.
        """
        if self.keyboard.paused or not self.keyboard.focused:
            return neutral_command()
        return command_from_keys(
            self.keyboard,
            yaw=self.yaw,
            velocity=self._state.velocity,
            tuning=self.tuning,
        )

    def step(self, command: TeleopCommand | None = None) -> SimState:
        """Advance exactly one fixed simulation step."""
        active = self.current_command() if command is None else command
        self.command = active

        self.backend.step(active.to_control_target(), self.dt)
        self.backend.apply_constraints(
            min_bounds=self.world.min_bounds,
            max_bounds=self.world.max_bounds,
            terrain=self.world.terrain,
            terrain_clearance=self.world.terrain_clearance,
        )
        self._state = self.backend.state()
        self.steps_taken += 1
        self.colliding = bool(self.world.collision_check(np.asarray(self._state.position, dtype=float)))
        self.telemetry.record(self._state, active, collision=self.colliding)
        return self._state

    def advance(self, steps: int) -> SimState:
        """Run ``steps`` fixed steps, refreshing the command on each one."""
        for _ in range(max(0, int(steps))):
            self.step()
        return self._state

    def refresh_idle_command(self) -> None:
        """Recompute the displayed command without stepping physics."""
        self.command = self.current_command()

    def motor_thrust_fractions(self) -> np.ndarray:
        """Per-rotor thrust in visual order FL / FR / RR / RL, each in ``[0, 1]``."""
        hook = getattr(self.backend, "visualization_command", None)
        if not callable(hook):
            return np.full(4, 0.5, dtype=float)
        collective, rates = hook()
        max_rate = float(getattr(self.backend, "_max_body_rate", 4.0))
        return rotor_thrust_fractions(
            collective=float(collective),
            body_rate_cmd=rates,
            max_body_rate=max_rate,
        )


__all__ = ["TeleopEngine"]
