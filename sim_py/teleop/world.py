"""World construction for teleop sessions (terrain, bounds, start pose)."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from ..core.config import NormalizedSimConfig

logger = logging.getLogger(__name__)


def _obstacle_top(obstacle: Any) -> float:
    if hasattr(obstacle, "heights"):
        return float(np.max(obstacle.heights))
    top = float(obstacle.center[2])
    if hasattr(obstacle, "height"):
        return top + float(obstacle.height)
    if hasattr(obstacle, "size"):
        return top + float(obstacle.size[2]) / 2.0
    return top


def max_obstacle_top(obstacles: list[Any]) -> float:
    return max((_obstacle_top(o) for o in obstacles), default=0.0)


@dataclass
class TeleopWorld:
    """Everything the renderer and the physics need to know about the map."""

    terrain_type: str
    space_dim: np.ndarray
    max_z_allowed: float
    start_position: np.ndarray
    terrain_clearance: float
    obstacles: list[Any] = field(default_factory=list)
    terrain: Any | None = None
    collision_check: Callable[[np.ndarray], bool] = lambda _pos: False
    planned_waypoints: np.ndarray | None = None
    goal_position: np.ndarray | None = None
    planner_type: str = ""

    @property
    def min_bounds(self) -> np.ndarray:
        return np.zeros(3, dtype=float)

    @property
    def max_bounds(self) -> np.ndarray:
        return np.array(
            [float(self.space_dim[0]), float(self.space_dim[1]), float(self.max_z_allowed)],
            dtype=float,
        )

    @property
    def camera_bounds(self) -> np.ndarray:
        return np.vstack([self.min_bounds, self.max_bounds])

    def ground_height(self, x: float, y: float) -> float:
        if self.terrain is not None and hasattr(self.terrain, "height_at"):
            return float(self.terrain.height_at(float(x), float(y)))
        return 0.0


def build_world(cfg_norm: NormalizedSimConfig) -> TeleopWorld:
    """Generate the teleop map from a normalized simulator config."""
    path_cfg = dict(cfg_norm.path_cfg)
    vis_cfg = dict(cfg_norm.visual_cfg)
    terrain_clearance = float(path_cfg.get("terrain_clearance", 2.0))

    use_empty_world = cfg_norm.terrain_override is None and cfg_norm.terrain_config_path is None
    if use_empty_world:
        space_dim = np.asarray(
            vis_cfg.get("teleop_empty_space_dim", [80.0, 80.0, 40.0]), dtype=float
        ).reshape(3)
        logger.info(
            "Teleop using empty world %s (pass --terrain <forest|mountains|plains> for terrain).",
            space_dim.tolist(),
        )
        world = TeleopWorld(
            terrain_type="empty",
            space_dim=space_dim,
            max_z_allowed=float(space_dim[2]),
            start_position=np.zeros(3),
            terrain_clearance=terrain_clearance,
        )
    else:
        from aerial_kit.sim.terrain import generate_terrain, is_point_in_collision, load_terrain_config

        if cfg_norm.terrain_config_path is not None:
            terrain_cfg = load_terrain_config(Path(cfg_norm.terrain_config_path))
        else:
            terrain_cfg = load_terrain_config()
        if cfg_norm.terrain_override is not None:
            terrain_cfg.terrain_type = cfg_norm.terrain_override
        # Applied before generation (not after) so obstacles scatter across the
        # enlarged area rather than staying clustered in one corner of it --
        # generate_plains/generate_forest/etc. all sample positions from
        # cfg.space_dim. A fixed wing cruising at 15-25 m/s crosses the shared
        # 120x120 m default in a handful of seconds; scenarios that need more
        # room set this rather than everything inheriting a wing-sized world.
        if "teleop_terrain_space_dim_m" in vis_cfg:
            terrain_cfg.space_dim = np.asarray(vis_cfg["teleop_terrain_space_dim_m"], dtype=float)

        obstacles = generate_terrain(
            terrain_cfg,
            forest_density_scale=float(vis_cfg.get("forest_density_scale", 1.0)),
            tree_height_scale=float(vis_cfg.get("tree_height_scale", 1.0)),
        )
        terrain = next((o for o in obstacles if hasattr(o, "height_at")), None)
        obstacle_top = max_obstacle_top(obstacles)

        space_dim = np.asarray(terrain_cfg.space_dim, dtype=float).copy()
        if terrain_cfg.terrain_type in {"forest", "mountains"} and obstacle_top > 0.0:
            space_dim[2] = obstacle_top * float(vis_cfg.get("height_ratio", 1.2))

        world = TeleopWorld(
            terrain_type=str(terrain_cfg.terrain_type),
            space_dim=space_dim,
            max_z_allowed=float(space_dim[2]),
            start_position=np.zeros(3),
            terrain_clearance=terrain_clearance,
            obstacles=list(obstacles),
            terrain=terrain,
            collision_check=lambda pos: bool(is_point_in_collision(pos, obstacles, inflation=0.0)),
        )

    start = np.array(
        [
            float(path_cfg.get("start_relative_x", 0.5)) * float(world.space_dim[0]),
            float(path_cfg.get("start_relative_y", 0.5)) * float(world.space_dim[1]),
            float(path_cfg.get("teleop_start_relative_z", path_cfg.get("start_relative_z", 0.2)))
            * float(world.space_dim[2]),
        ],
        dtype=float,
    )
    ground = world.ground_height(start[0], start[1])
    start[2] = max(start[2], ground + terrain_clearance)
    world.start_position = start
    attach_plan(world, cfg_norm)
    return world


def goal_position_from_config(world: TeleopWorld, path_cfg: dict[str, Any]) -> np.ndarray:
    """Goal in metres from ``path.end_relative_*``, raised above terrain."""
    gx = float(np.clip(float(path_cfg.get("end_relative_x", 0.9)) * float(world.space_dim[0]), 0.0, float(world.space_dim[0])))
    gy = float(np.clip(float(path_cfg.get("end_relative_y", 0.9)) * float(world.space_dim[1]), 0.0, float(world.space_dim[1])))
    gz_raw = path_cfg.get("end_relative_z", 0.4)
    try:
        gz_rel = float(gz_raw)
    except (TypeError, ValueError):
        gz_rel = 0.4
    gz = gz_rel * float(world.space_dim[2])
    gz = max(gz, world.ground_height(gx, gy) + world.terrain_clearance)
    gz = min(gz, float(world.max_z_allowed))
    return np.array([gx, gy, gz], dtype=float)


def attach_plan(world: TeleopWorld, cfg_norm: NormalizedSimConfig) -> None:
    """Run the configured planner and stash the waypoints on ``world``.

    Failures are logged and leave the world without a plan so teleop still
    opens; the overlay is optional.
    """
    from ..core.registry import create_planner, register_builtin_components

    path_cfg = dict(cfg_norm.path_cfg)
    goal = goal_position_from_config(world, path_cfg)
    world.goal_position = goal
    planner_type_raw = str(path_cfg.get("planner_type", "straight")).lower()
    planner_type = "rrtstar" if planner_type_raw in {"rrt*", "rrt_star", "rrtstar"} else planner_type_raw
    try:
        register_builtin_components()
        planner = create_planner(planner_type)
        waypoints = planner.plan(
            start=world.start_position,
            goal=goal,
            obstacles=world.obstacles,
            cfg={
                "path": path_cfg,
                "space_dim": world.space_dim,
                "start_pos": world.start_position,
            },
        )
    except Exception:
        logger.exception("Teleop planner '%s' failed; flying without a planned overlay.", planner_type)
        return
    if not waypoints:
        return
    world.planner_type = planner_type
    world.planned_waypoints = np.array([wp.position for wp in waypoints], dtype=float)
    logger.info(
        "Teleop planned %s path with %d waypoints.",
        planner_type,
        len(world.planned_waypoints),
    )


__all__ = [
    "TeleopWorld",
    "attach_plan",
    "build_world",
    "goal_position_from_config",
    "max_obstacle_top",
]
