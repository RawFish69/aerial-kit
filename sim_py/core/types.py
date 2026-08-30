"""Re-exports ``aerial_kit.types`` for backward compatibility.

The canonical definitions live in ``aerial_kit`` so both ``sim_py`` and
the ROS 2 nodes share one implementation. Import from ``aerial_kit``
directly in new code.
"""

from __future__ import annotations

from aerial_kit.types import (
    Capabilities,
    CommandKind,
    ControlTarget,
    SimState,
    TrajectoryLog,
    Waypoint,
    Wrench,
)

__all__ = [
    "Capabilities",
    "CommandKind",
    "ControlTarget",
    "SimState",
    "TrajectoryLog",
    "Waypoint",
    "Wrench",
]
