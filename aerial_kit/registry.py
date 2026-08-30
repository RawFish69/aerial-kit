"""Component registries for planners, controllers, and dynamics backends."""

from __future__ import annotations

from collections.abc import Callable

from .airframes.base import Airframe
from .interfaces import Controller, DynamicsBackend, Planner

PlannerFactory = Callable[[], Planner]
ControllerFactory = Callable[[], Controller]
BackendFactory = Callable[[], DynamicsBackend]
AirframeFactory = Callable[[], Airframe]

PLANNERS: dict[str, PlannerFactory] = {}
CONTROLLERS: dict[str, ControllerFactory] = {}
BACKENDS: dict[str, BackendFactory] = {}
AIRFRAMES: dict[str, AirframeFactory] = {}

_BUILTINS_REGISTERED = False


def _normalize_planner_name(name: str) -> str:
    key = str(name).lower().strip()
    if key in {"rrt*", "rrtstar", "rrt_star"}:
        return "rrtstar"
    return key


def _normalize_name(name: str) -> str:
    return str(name).lower().strip()


def register_planner(name: str, factory: PlannerFactory) -> None:
    PLANNERS[_normalize_planner_name(name)] = factory


def register_controller(name: str, factory: ControllerFactory) -> None:
    CONTROLLERS[_normalize_name(name)] = factory


def register_backend(name: str, factory: BackendFactory) -> None:
    BACKENDS[_normalize_name(name)] = factory


def register_airframe(name: str, factory: AirframeFactory) -> None:
    AIRFRAMES[_normalize_name(name)] = factory


def create_planner(name: str) -> Planner:
    key = _normalize_planner_name(name)
    if key not in PLANNERS:
        available = ", ".join(sorted(PLANNERS)) or "none"
        raise ValueError(f"Unknown planner '{name}'. Available: {available}")
    return PLANNERS[key]()


def create_controller(name: str) -> Controller:
    key = _normalize_name(name)
    if key not in CONTROLLERS:
        available = ", ".join(sorted(CONTROLLERS)) or "none"
        raise ValueError(f"Unknown controller '{name}'. Available: {available}")
    return CONTROLLERS[key]()


def create_backend(name: str) -> DynamicsBackend:
    key = _normalize_name(name)
    if key not in BACKENDS:
        available = ", ".join(sorted(BACKENDS)) or "none"
        raise ValueError(f"Unknown backend '{name}'. Available: {available}")
    return BACKENDS[key]()


def create_airframe(name: str) -> Airframe:
    key = _normalize_name(name)
    if key not in AIRFRAMES:
        available = ", ".join(sorted(AIRFRAMES)) or "none"
        raise ValueError(f"Unknown airframe '{name}'. Available: {available}")
    return AIRFRAMES[key]()


def register_builtin_components() -> None:
    """Register built-in planners/controllers/backends once."""
    global _BUILTINS_REGISTERED
    if _BUILTINS_REGISTERED:
        return

    from sim_py.backends.fixedwing_backend import FixedWingBackend
    from sim_py.backends.pointmass_backend import PointMassBackend
    from sim_py.backends.rotorpy_backend import RotorPyBackend
    from sim_py.planners.basic import AStarPlanner, RRTPlanner, RRTStarPlanner, StraightPlanner
    from sim_py.planners.dubins import DubinsPlanner

    from .airframes.fixed_wing import TwinWingAirframe
    from .airframes.multirotor import MultirotorAirframe
    from .controllers.basic import LQRController, MPCController, PIDController
    from .controllers.fixed_wing import FixedWingL1TECSController

    register_planner("straight", StraightPlanner)
    register_planner("astar", AStarPlanner)
    register_planner("rrt", RRTPlanner)
    register_planner("rrtstar", RRTStarPlanner)
    register_planner("dubins", DubinsPlanner)

    register_controller("pid", PIDController)
    register_controller("lqr", LQRController)
    register_controller("mpc", MPCController)
    register_controller("l1_tecs", FixedWingL1TECSController)

    register_backend("pointmass", PointMassBackend)
    register_backend("rotorpy", RotorPyBackend)
    register_backend("fixedwing", FixedWingBackend)

    register_airframe("quad", lambda: MultirotorAirframe(arms=4, layout="x"))
    register_airframe("hex", lambda: MultirotorAirframe(arms=6, layout="x"))
    register_airframe("octo", lambda: MultirotorAirframe(arms=8, layout="x"))
    register_airframe("twin_wing", TwinWingAirframe)

    _BUILTINS_REGISTERED = True
