"""Simulation-specific result types."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class SimulationResult:
    """Trajectory, environment, and diagnostics produced by a simulation."""

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
