"""Interactive teleop entry point for the standalone simulator.

The implementation lives in :mod:`sim_py.teleop`, which splits input handling,
command mapping, the fixed-step loop, rendering, the camera and telemetry into
separately testable pieces. This module stays as the stable import path used by
the CLI and by ``aerial_kit.sim.teleop``.
"""

from __future__ import annotations

from ..teleop.session import build_session, run_teleop_session

__all__ = ["build_session", "run_teleop_session"]
