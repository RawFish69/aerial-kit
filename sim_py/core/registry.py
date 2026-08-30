"""Re-exports ``aerial_kit.registry`` for backward compatibility.

The canonical registry lives in ``aerial_kit`` so both ``sim_py`` and
the ROS 2 nodes share one implementation. Import from ``aerial_kit``
directly in new code.
"""

from __future__ import annotations

from aerial_kit.registry import (
    AIRFRAMES,
    BACKENDS,
    CONTROLLERS,
    PLANNERS,
    AirframeFactory,
    BackendFactory,
    ControllerFactory,
    PlannerFactory,
    create_airframe,
    create_backend,
    create_controller,
    create_planner,
    register_airframe,
    register_backend,
    register_builtin_components,
    register_controller,
    register_planner,
)

__all__ = [
    "AIRFRAMES",
    "BACKENDS",
    "CONTROLLERS",
    "PLANNERS",
    "AirframeFactory",
    "BackendFactory",
    "ControllerFactory",
    "PlannerFactory",
    "create_airframe",
    "create_backend",
    "create_controller",
    "create_planner",
    "register_airframe",
    "register_backend",
    "register_builtin_components",
    "register_controller",
    "register_planner",
]
