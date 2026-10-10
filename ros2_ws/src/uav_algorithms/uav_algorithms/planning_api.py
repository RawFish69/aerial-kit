from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .repo_paths import ensure_repo_root_on_path


def _resolve_terrain_config(repo_root: Path) -> Path:
    candidates = [
        repo_root / 'ros2_ws' / 'src' / 'terrain_generator' / 'config' / 'terrain_params.yaml',
    ]
    for c in candidates:
        if c.exists():
            return c
    # Fall back to the canonical path; the caller will get a useful error.
    return candidates[0]


def _normalize_planner_key(planner_type: str) -> str:
    planner_key = str(planner_type).lower().strip()
    if planner_key in {'rrt*', 'rrt_star'}:
        return 'rrtstar'
    return planner_key


def build_terrain_obstacles(terrain_profile: str = 'plains'):
    """Build terrain obstacles using the same config stack as the planner."""
    repo_root = ensure_repo_root_on_path()
    from sim_py.terrain_wrapper import generate_terrain, load_terrain_config  # type: ignore

    terrain_cfg = load_terrain_config(_resolve_terrain_config(repo_root))
    if terrain_profile:
        terrain_cfg.terrain_type = str(terrain_profile).lower()
    obstacles = generate_terrain(terrain_cfg)
    return terrain_cfg, obstacles


def terrain_obstacles_from_markers(markers) -> list:
    """Published terrain markers -> the planner's obstacle objects.

    The markers on ``/terrain/obstacles`` are what RViz draws and what the
    predictive tracker avoids; a planner that generates its own random forest
    instead plans around trees that are not there and through ones that are.
    ``CYLINDER`` markers (pose at mid-height, scale = diameter, diameter,
    height) become ``CylinderObstacle``s, whose ``center`` is the *base*;
    ``CUBE`` markers become ``BoxObstacle``s, whose ``center`` is the middle;
    a ``SPHERE`` is taken as the cube around it. ``DELETEALL`` starts afresh,
    ``DELETE`` is skipped. Duck-typed on the message fields, so testable
    without ROS.
    """
    ensure_repo_root_on_path()
    from aerial_kit.sim.terrain import BoxObstacle, CylinderObstacle

    cube, sphere, cylinder = 1, 2, 3  # visualization_msgs/Marker types
    delete, delete_all = 2, 3  # Marker actions
    out: list = []
    for m in markers:
        action = int(getattr(m, 'action', 0))
        if action == delete_all:
            out = []
            continue
        if action == delete:
            continue
        x, y, z = float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)
        sx, sy, sz = float(m.scale.x), float(m.scale.y), float(m.scale.z)
        kind = int(m.type)
        if kind == cylinder:
            out.append(CylinderObstacle(center=[x, y, z - 0.5 * sz], height=sz, radius=0.5 * max(sx, sy)))
        elif kind == cube:
            out.append(BoxObstacle(center=[x, y, z], size=[sx, sy, sz]))
        elif kind == sphere:
            d = max(sx, sy, sz)
            out.append(BoxObstacle(center=[x, y, z], size=[d, d, d]))
    return out


def validate_trajectory_segments(
    points_xyz: Iterable[Iterable[float]],
    obstacles,
    inflation_m: float = 0.0,
    step_m: float = 0.5,
) -> tuple[bool, str]:
    """Validate every segment by dense point sampling against the obstacle list."""
    from sim_py.terrain_wrapper import is_point_in_collision  # type: ignore

    points = [np.asarray(p, dtype=float).reshape(3) for p in points_xyz]
    if not points:
        return False, 'empty path'
    if len(points) == 1:
        if is_point_in_collision(points[0], obstacles, inflation=float(inflation_m)):
            return False, 'single waypoint is in collision'
        return True, 'single waypoint path is collision free'

    sample_step = max(0.05, float(step_m))
    inflation = float(inflation_m)
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        seg = p1 - p0
        seg_len = float(np.linalg.norm(seg))
        if seg_len <= 1e-9:
            if is_point_in_collision(p0, obstacles, inflation=inflation):
                return False, f'segment {i} degenerate point is in collision'
            continue
        steps = max(2, int(np.ceil(seg_len / sample_step)) + 1)
        for s in np.linspace(0.0, 1.0, steps):
            p = (1.0 - s) * p0 + s * p1
            if is_point_in_collision(p, obstacles, inflation=inflation):
                return False, f'segment collision detected at segment {i}'
    return True, 'path is collision free'


def plan_trajectory_points(
    start_xyz: Iterable[float],
    goal_xyz: Iterable[float],
    planner_type: str = 'rrtstar',
    terrain_profile: str = 'plains',
    collision_inflation_m: float = 0.5,
    terrain_clearance_m: float | None = None,
    planner_options: dict[str, Any] | None = None,
) -> list[np.ndarray]:
    """Return a list of ENU waypoint positions using the existing planner stack."""
    points, _ = plan_trajectory_points_with_obstacles(
        start_xyz=start_xyz,
        goal_xyz=goal_xyz,
        planner_type=planner_type,
        terrain_profile=terrain_profile,
        collision_inflation_m=collision_inflation_m,
        terrain_clearance_m=terrain_clearance_m,
        planner_options=planner_options,
    )
    return points


def plan_trajectory_points_with_obstacles(
    start_xyz: Iterable[float],
    goal_xyz: Iterable[float],
    planner_type: str = 'rrtstar',
    terrain_profile: str = 'plains',
    collision_inflation_m: float = 0.5,
    terrain_clearance_m: float | None = None,
    planner_options: dict[str, Any] | None = None,
    obstacles: list | None = None,
) -> tuple[list[np.ndarray], Any]:
    """Plan a path and return both points and the exact obstacle list used.

    ``obstacles`` plans around a given set (e.g. the published terrain, via
    :func:`terrain_obstacles_from_markers`) instead of generating one; the
    terrain config still supplies the world's extent.
    """
    ensure_repo_root_on_path()
    from sim_py.planner import default_plan  # type: ignore

    terrain_cfg, generated = build_terrain_obstacles(terrain_profile)
    if obstacles is None:
        obstacles = generated

    planner_key = _normalize_planner_key(planner_type)

    path_cfg: dict[str, Any] = {
        'planner_type': planner_key,
        'start_abs': list(np.asarray(start_xyz, dtype=float).reshape(3)),
        'goal_abs': list(np.asarray(goal_xyz, dtype=float).reshape(3)),
        'collision_inflation': float(collision_inflation_m),
    }
    if terrain_clearance_m is not None:
        path_cfg['terrain_clearance'] = float(terrain_clearance_m)
    if planner_options:
        path_cfg.update({str(k): v for k, v in planner_options.items() if v is not None})

    waypoints = default_plan(
        space_dim=np.asarray(terrain_cfg.space_dim, dtype=float),
        start_pos=np.asarray(terrain_cfg.start_pos, dtype=float),
        obstacles=obstacles,
        path_cfg=path_cfg,
    )
    points = [np.asarray(wp.position, dtype=float).reshape(3) for wp in waypoints]
    return points, obstacles
