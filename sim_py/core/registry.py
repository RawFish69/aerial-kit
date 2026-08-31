"""Re-exports ``aerial_kit.registry`` for backward compatibility, plus
sim_py's own builtin registration.

The canonical registry lives in ``aerial_kit`` so both ``sim_py`` and
the ROS 2 nodes share one implementation. Import from ``aerial_kit``
directly in new code.
"""

from __future__ import annotations

import aerial_kit.registry as _aerial_kit_registry
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
    register_controller,
    register_planner,
)

_SIM_PY_BUILTINS_REGISTERED = False


def register_builtin_components() -> None:
    """Register aerial_kit's builtins, plus sim_py's own planners/backends.

    aerial_kit's own ``register_builtin_components()`` only knows about
    airframes/controllers that live inside aerial_kit itself (it has no ROS
    or matplotlib dependency and doesn't import ``sim_py``). Planners and
    backends are sim_py-specific -- some pull in matplotlib-adjacent legacy
    planner code, or an optional external dependency like RotorPy -- so
    sim_py registers those itself here.
    """
    global _SIM_PY_BUILTINS_REGISTERED
    _aerial_kit_registry.register_builtin_components()
    if _SIM_PY_BUILTINS_REGISTERED:
        return

    from sim_py.backends.fixedwing_backend import FixedWingBackend
    from sim_py.backends.multirotor_backend import MultirotorBackend
    from sim_py.backends.pointmass_backend import PointMassBackend
    from sim_py.backends.rotorpy_backend import RotorPyBackend
    from sim_py.planners.basic import AStarPlanner, RRTPlanner, RRTStarPlanner, StraightPlanner
    from sim_py.planners.dubins import DubinsPlanner

    register_planner("straight", StraightPlanner)
    register_planner("astar", AStarPlanner)
    register_planner("rrt", RRTPlanner)
    register_planner("rrtstar", RRTStarPlanner)
    register_planner("dubins", DubinsPlanner)

    register_backend("pointmass", PointMassBackend)
    register_backend("multirotor", MultirotorBackend)
    register_backend("rotorpy", RotorPyBackend)
    register_backend("fixedwing", FixedWingBackend)

    _SIM_PY_BUILTINS_REGISTERED = True


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
