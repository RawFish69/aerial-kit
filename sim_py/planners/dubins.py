"""Dubins path steering geometry: curvature-constrained paths between two poses.

Implements the four CSC (curve-straight-curve) primitives: LSL, RSR, LSR, RSL. The two
CCC (RLR/LRL) primitives are not implemented -- they only ever win when start and goal
are close together relative to the turn radius, which is not the regime curvature-aware
waypoint planning for a cruising fixed wing needs to handle first.

``DubinsPlanner`` (registered as ``"dubins"`` in ``aerial_kit.registry``) wraps
``plan_dubins_path``/``sample_dubins_path`` as a ``Planner``: it reads
``turn_radius_m`` from ``cfg["path"]`` -- ``runner.py`` populates this from the
selected airframe's ``Capabilities.min_turn_radius_m`` when it's not ``None`` (per
plan 04's design), so selecting ``twin_wing`` with the default planner config
automatically gets a curvature-feasible path. Start/goal heading defaults to the
straight-line bearing between them (both ends), since the planner interface is not
given an initial heading -- override via ``cfg["path"]["start_heading_rad"]``/
``"goal_heading_rad"`` for a multi-leg circuit where each leg's start heading should
match the previous leg's arrival heading.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Mapping

import numpy as np

from ..core.interfaces import Planner
from ..core.types import Waypoint

Pose2D = tuple[float, float, float]  # (x, y, yaw_rad)


def _mod2pi(theta: float) -> float:
    return float(theta % (2.0 * np.pi))


@dataclass(frozen=True)
class DubinsSegment:
    """One arc ("L"/"R") or straight ("S") segment, length in meters."""

    mode: str
    length_m: float


@dataclass(frozen=True)
class DubinsPath:
    segments: tuple[DubinsSegment, DubinsSegment, DubinsSegment]
    turn_radius_m: float

    @property
    def total_length_m(self) -> float:
        return sum(seg.length_m for seg in self.segments)


def _lsl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    sa, sb = np.sin(alpha), np.sin(beta)
    ca, cb = np.cos(alpha), np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb)
    if p_sq < 0.0:
        return None
    tmp = np.arctan2(cb - ca, d + sa - sb)
    t = _mod2pi(-alpha + tmp)
    p = float(np.sqrt(p_sq))
    q = _mod2pi(beta - tmp)
    return t, p, q


def _rsr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    sa, sb = np.sin(alpha), np.sin(beta)
    ca, cb = np.cos(alpha), np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sb - sa)
    if p_sq < 0.0:
        return None
    tmp = np.arctan2(ca - cb, d - sa + sb)
    t = _mod2pi(alpha - tmp)
    p = float(np.sqrt(p_sq))
    q = _mod2pi(-beta + tmp)
    return t, p, q


def _lsr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    sa, sb = np.sin(alpha), np.sin(beta)
    ca, cb = np.cos(alpha), np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_sq = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb)
    if p_sq < 0.0:
        return None
    p = float(np.sqrt(p_sq))
    tmp = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
    t = _mod2pi(-alpha + tmp)
    q = _mod2pi(-_mod2pi(beta) + tmp)
    return t, p, q


def _rsl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    sa, sb = np.sin(alpha), np.sin(beta)
    ca, cb = np.cos(alpha), np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_sq = d * d - 2.0 + 2.0 * c_ab - 2.0 * d * (sa + sb)
    if p_sq < 0.0:
        return None
    p = float(np.sqrt(p_sq))
    tmp = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
    t = _mod2pi(alpha - tmp)
    q = _mod2pi(beta - tmp)
    return t, p, q


_PRIMITIVES: dict[str, tuple[str, str, str]] = {
    "LSL": ("L", "S", "L"),
    "RSR": ("R", "S", "R"),
    "LSR": ("L", "S", "R"),
    "RSL": ("R", "S", "L"),
}
_SOLVERS = {"LSL": _lsl, "RSR": _rsr, "LSR": _lsr, "RSL": _rsl}


def plan_dubins_path(start: Pose2D, goal: Pose2D, turn_radius_m: float) -> DubinsPath:
    """Shortest CSC Dubins path from ``start`` to ``goal`` at a fixed turn radius."""
    if turn_radius_m <= 0.0:
        raise ValueError(f"turn_radius_m must be positive, got {turn_radius_m}")

    sx, sy, syaw = start
    gx, gy, gyaw = goal
    curvature = 1.0 / turn_radius_m

    dx = gx - sx
    dy = gy - sy
    # Local frame: start at origin, start heading along local +x.
    lex = np.cos(syaw) * dx + np.sin(syaw) * dy
    ley = -np.sin(syaw) * dx + np.cos(syaw) * dy
    leyaw = gyaw - syaw

    d = float(np.hypot(lex, ley)) * curvature
    theta = _mod2pi(np.arctan2(ley, lex))
    alpha = _mod2pi(-theta)
    beta = _mod2pi(leyaw - theta)

    best_cost = np.inf
    best: tuple[str, float, float, float] | None = None
    for name, solver in _SOLVERS.items():
        result = solver(alpha, beta, d)
        if result is None:
            continue
        t, p, q = result
        cost = abs(t) + abs(p) + abs(q)
        if cost < best_cost:
            best_cost = cost
            best = (name, t, p, q)

    if best is None:
        raise ValueError(
            f"No CSC Dubins path found for start={start}, goal={goal}, "
            f"turn_radius_m={turn_radius_m}"
        )

    name, t, p, q = best
    modes = _PRIMITIVES[name]
    lengths_normalized = (t, p, q)
    segments = tuple(
        DubinsSegment(mode=mode, length_m=length * turn_radius_m)
        for mode, length in zip(modes, lengths_normalized)
    )
    return DubinsPath(segments=segments, turn_radius_m=turn_radius_m)  # type: ignore[arg-type]


def _step_pose(mode: str, length_m: float, radius_m: float, pose: Pose2D) -> Pose2D:
    x, y, yaw = pose
    if mode == "S":
        return x + length_m * np.cos(yaw), y + length_m * np.sin(yaw), yaw
    if mode == "L":
        new_yaw = yaw + length_m / radius_m
        return (
            x + radius_m * (np.sin(new_yaw) - np.sin(yaw)),
            y + radius_m * (-np.cos(new_yaw) + np.cos(yaw)),
            new_yaw,
        )
    if mode == "R":
        new_yaw = yaw - length_m / radius_m
        return (
            x + radius_m * (-np.sin(new_yaw) + np.sin(yaw)),
            y + radius_m * (np.cos(new_yaw) - np.cos(yaw)),
            new_yaw,
        )
    raise ValueError(f"Unknown segment mode {mode!r}")


def sample_dubins_path(start: Pose2D, path: DubinsPath, step_size_m: float) -> np.ndarray:
    """Sample ``path`` starting at ``start`` into an (N, 3) array of (x, y, yaw)."""
    if step_size_m <= 0.0:
        raise ValueError(f"step_size_m must be positive, got {step_size_m}")

    poses = [start]
    pose = start
    for segment in path.segments:
        remaining = segment.length_m
        while remaining > step_size_m:
            pose = _step_pose(segment.mode, step_size_m, path.turn_radius_m, pose)
            poses.append(pose)
            remaining -= step_size_m
        if remaining > 1e-9:
            pose = _step_pose(segment.mode, remaining, path.turn_radius_m, pose)
            poses.append(pose)

    return np.array(poses, dtype=float)


class DubinsPlanner(Planner):
    """Curvature-feasible straight-goal planner: one Dubins path, start to goal."""

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: Iterable[Any],
        cfg: Mapping[str, Any],
    ) -> list[Waypoint]:
        path_cfg = dict(cfg.get("path", {}) or {})
        turn_radius_m = float(path_cfg.get("turn_radius_m", 20.0))
        step_size_m = float(path_cfg.get("dubins_step_size_m", 5.0))

        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        bearing = float(np.arctan2(goal[1] - start[1], goal[0] - start[0]))
        start_heading = float(path_cfg.get("start_heading_rad", bearing))
        goal_heading = float(path_cfg.get("goal_heading_rad", bearing))

        start_pose: Pose2D = (float(start[0]), float(start[1]), start_heading)
        goal_pose: Pose2D = (float(goal[0]), float(goal[1]), goal_heading)
        dubins_path = plan_dubins_path(start_pose, goal_pose, turn_radius_m)
        samples = sample_dubins_path(start_pose, dubins_path, step_size_m)

        z_progress = np.linspace(float(start[2]), float(goal[2]), len(samples))
        return [
            Waypoint(position=np.array([x, y, z], dtype=float))
            for (x, y, _yaw), z in zip(samples, z_progress)
        ]
