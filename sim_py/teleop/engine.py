"""Headless teleop physics engine.

The engine holds the keyboard state, the command mapping, the dynamics backend
and the telemetry recorder. It has no Matplotlib dependency, so a full flight
can be driven from a test by pressing keys and calling :meth:`TeleopEngine.advance`.
"""

from __future__ import annotations

import numpy as np

from aerial_kit.interfaces import DynamicsBackend
from aerial_kit.types import SimState

from .commands import (
    FixedWingTeleopTuning,
    TeleopCommand,
    TeleopTuning,
    command_from_keys,
    fixedwing_command_from_keys,
    neutral_command,
    neutral_fixedwing_command,
)
from .input_state import KeyboardState
from .model import quat_to_euler_rpy, rotor_thrust_fractions
from .telemetry import TelemetryRecorder
from .world import TeleopWorld


def _state_is_finite(state: SimState) -> bool:
    """True unless any of position/velocity/attitude/body_rates has gone non-finite.

    A dynamics model diverging under an extreme command is the plausible
    cause (see FixedWingDynamics' body-rate clamp for the specific case this
    was written for), but this check is airframe-agnostic on purpose: it is
    the last line of defence before a NaN reaches Matplotlib, which raises
    outright rather than merely rendering oddly when asked to set an axis
    limit to NaN.
    """
    for field in (state.position, state.velocity, state.attitude_quat, state.body_rates):
        if field is not None and not np.all(np.isfinite(field)):
            return False
    return True


class TeleopEngine:
    """Fixed-step teleop simulation with no GUI attached.

    ``tuning``'s type selects the command mapping: :class:`TeleopTuning` drives
    the multirotor's world-frame-acceleration path, :class:`FixedWingTeleopTuning`
    drives the twin-wing's direct actuator path. The two airframes need
    genuinely different physical commands (the fixed wing cannot be commanded
    by an acceleration setpoint the way a multirotor can), so this is a branch
    on tuning type rather than another constructor flag threaded everywhere.
    """

    def __init__(
        self,
        *,
        backend: DynamicsBackend,
        world: TeleopWorld,
        tuning: TeleopTuning | FixedWingTeleopTuning,
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

        self.command: TeleopCommand = (
            neutral_fixedwing_command(tuning)
            if isinstance(tuning, FixedWingTeleopTuning)
            else neutral_command()
        )
        self.colliding = False
        #: True once a step has produced non-finite state (a dynamics model
        #: diverging under an extreme sustained command, most plausibly). The
        #: engine then freezes at the last good state instead of handing NaN
        #: position/attitude to the renderer, which matplotlib's axis-limit
        #: call raises on outright rather than merely rendering oddly.
        self.crashed = False
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
        if isinstance(self.tuning, FixedWingTeleopTuning):
            if self.keyboard.paused or not self.keyboard.focused:
                return neutral_fixedwing_command(self.tuning)
            return fixedwing_command_from_keys(self.keyboard, tuning=self.tuning)
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
        if self.crashed:
            return self._state

        active = self.current_command() if command is None else command
        self.command = active

        self.backend.step(active.to_control_target(), self.dt)
        self.backend.apply_constraints(
            min_bounds=self.world.min_bounds,
            max_bounds=self.world.max_bounds,
            terrain=self.world.terrain,
            terrain_clearance=self.world.terrain_clearance,
        )
        new_state = self.backend.state()
        if not _state_is_finite(new_state):
            # Whatever produced this (an extreme sustained command driving the
            # dynamics model past where its integration is stable, most
            # plausibly) already happened; there is no recovery, only
            # reporting it instead of handing the renderer a NaN it can't
            # draw. self._state is deliberately left at its last-good value.
            self.crashed = True
            self.colliding = True
            return self._state

        self._state = new_state
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
        """Per-motor thrust in ``[0, 1]``.

        Multirotor: FL / FR / RR / RL, read from the mixer via the backend's
        ``visualization_command`` hook. Fixed wing: L / R, read straight off
        the last commanded actuator array -- there is no mixer to query, the
        teleop command *is* the per-motor thrust.
        """
        if isinstance(self.tuning, FixedWingTeleopTuning):
            actuator = self.command.actuator_cmd
            if actuator is None:
                return np.full(2, 0.5, dtype=float)
            throttle_l, throttle_r = np.asarray(actuator, dtype=float)[:2]
            max_n = max(self.tuning.throttle_max_n, 1e-6)
            return np.clip(np.array([throttle_l, throttle_r], dtype=float) / max_n, 0.0, 1.0)

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
