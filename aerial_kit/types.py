"""Core datatypes for the standalone simulator framework."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

import numpy as np


class CommandKind(Enum):
    """What kind of control target an airframe's inner loop accepts."""

    ACCEL = auto()  # multirotor / point mass -- today's behavior
    AIRSPEED_NAV = auto()  # fixed wing: (airspeed, bank_or_course_rate, climb_rate)
    WRENCH = auto()  # low-level: body thrust + moment


@dataclass(frozen=True)
class Capabilities:
    """What an airframe can physically do -- used to validate controller/planner fit."""

    can_hover: bool
    min_airspeed_mps: float | None  # None for hovering craft
    max_airspeed_mps: float
    max_climb_rate_mps: float
    max_bank_deg: float
    min_turn_radius_m: float | None  # None if it can turn in place
    n_actuators: int
    command_kind: CommandKind


@dataclass
class Wrench:
    """Body-frame thrust + moment, the common output of an inner-loop controller."""

    force_body: np.ndarray  # 3D body-frame force [N]
    moment_body: np.ndarray  # 3D body-frame moment [N*m]


@dataclass
class SimState:
    """Canonical simulator state used across controllers and backends.

    Quaternion convention is ``[w, x, y, z]`` when present.
    """

    position: np.ndarray
    velocity: np.ndarray
    t: float = 0.0
    attitude_quat: np.ndarray | None = None
    body_rates: np.ndarray | None = None


@dataclass
class ControlTarget:
    """Canonical high-level control target produced by controllers."""

    accel_cmd: np.ndarray
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class Waypoint:
    """Single waypoint in 3D ENU coordinates."""

    position: np.ndarray


@dataclass
class TrajectoryLog:
    """Simulation outputs and diagnostics used for plotting/reporting."""

    trajectory: np.ndarray
    planned_waypoints: np.ndarray
    obstacles: list[Any]
    space_dim: np.ndarray
    terrain_type: str
    visual_cfg: dict[str, Any]
    goal_position: np.ndarray
    planner_type: str
    final_time: float
    final_waypoint_index: int
    final_waypoint_count: int
    distance_to_goal: float
    collisions_detected: int
    attitude_quats: np.ndarray | None = None
    backend_name: str = ""
