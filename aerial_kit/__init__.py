"""Public Python API for Aerial Kit.

The top-level package is deliberately ROS- and Matplotlib-free. It exposes the
stable data contracts, abstract interfaces and component registry used by
applications and by :mod:`sim_py`. Airframe and controller implementations
live in their subpackages and register on demand.
"""

from .interfaces import Controller, DynamicsBackend, Planner
from .registry import (
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
from .types import (
    Capabilities,
    CommandKind,
    ControlMode,
    ControlTarget,
    SimState,
    TrajectoryLog,
    Waypoint,
    Wrench,
)

__all__ = [
    "Capabilities",
    "CommandKind",
    "Controller",
    "ControlMode",
    "ControlTarget",
    "DynamicsBackend",
    "Planner",
    "SimState",
    "TrajectoryLog",
    "Waypoint",
    "Wrench",
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
