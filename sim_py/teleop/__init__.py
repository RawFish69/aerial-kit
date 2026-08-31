"""Real-time keyboard teleoperation for the standalone simulator.

The implementation is split so each concern can be exercised on its own:

* :mod:`~sim_py.teleop.input_state` -- latched keyboard state, no GUI imports
* :mod:`~sim_py.teleop.commands` -- input-to-command mapping and tuning
* :mod:`~sim_py.teleop.loop` -- monotonic-clock fixed-step scheduling
* :mod:`~sim_py.teleop.engine` -- headless fixed-step simulation
* :mod:`~sim_py.teleop.model` -- quadrotor geometry and quaternion helpers
* :mod:`~sim_py.teleop.renderer` -- Matplotlib artists and HUD
* :mod:`~sim_py.teleop.camera` -- follow/world camera
* :mod:`~sim_py.teleop.telemetry` -- session recording
* :mod:`~sim_py.teleop.session` -- GUI wiring and the public entry point

Only :mod:`~sim_py.teleop.renderer` and :mod:`~sim_py.teleop.session` require
Matplotlib, so they are imported lazily.
"""

from __future__ import annotations

from typing import Any

from .camera import FOLLOW, WORLD, CameraController
from .commands import (
    YAW_RATE_METADATA_KEY,
    InputAxes,
    TeleopCommand,
    TeleopTuning,
    axes_from_keys,
    command_from_axes,
    command_from_keys,
    neutral_command,
)
from .engine import TeleopEngine
from .input_state import CONTROL_HELP_LINES, HOLD_BINDINGS, KeyboardState, normalize_key
from .loop import FixedStepScheduler, resolve_interval_ms
from .model import QuadGeometry, quat_to_euler_rpy, quat_to_rotation_matrix
from .telemetry import TelemetryRecorder
from .world import TeleopWorld, build_world

_LAZY = {
    "HudInfo": ("renderer", "HudInfo"),
    "TeleopRenderer": ("renderer", "TeleopRenderer"),
    "TeleopSession": ("session", "TeleopSession"),
    "build_session": ("session", "build_session"),
    "ensure_interactive_backend": ("session", "ensure_interactive_backend"),
    "run_teleop_session": ("session", "run_teleop_session"),
}


def __getattr__(name: str) -> Any:
    target = _LAZY.get(name)
    if target is None:
        raise AttributeError(name)
    from importlib import import_module

    module = import_module(f"{__name__}.{target[0]}")
    return getattr(module, target[1])


__all__ = [
    "CONTROL_HELP_LINES",
    "FOLLOW",
    "HOLD_BINDINGS",
    "HudInfo",
    "InputAxes",
    "CameraController",
    "FixedStepScheduler",
    "KeyboardState",
    "QuadGeometry",
    "TeleopCommand",
    "TeleopEngine",
    "TeleopRenderer",
    "TeleopSession",
    "TeleopTuning",
    "TeleopWorld",
    "TelemetryRecorder",
    "WORLD",
    "YAW_RATE_METADATA_KEY",
    "axes_from_keys",
    "build_session",
    "build_world",
    "command_from_axes",
    "command_from_keys",
    "ensure_interactive_backend",
    "neutral_command",
    "normalize_key",
    "quat_to_euler_rpy",
    "quat_to_rotation_matrix",
    "resolve_interval_ms",
    "run_teleop_session",
]
