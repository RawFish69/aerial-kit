"""ROS-free terrain primitives and generators used by the Python simulator."""

from __future__ import annotations

from dataclasses import dataclass, field
from importlib.resources import as_file, files
from pathlib import Path
from typing import Literal, Sequence

import numpy as np

TerrainType = Literal["forest", "mountains", "plains"]


class CylinderObstacle:
    def __init__(self, center, height, radius):
        self.center = np.asarray(center, dtype=float)
        self.height = float(height)
        self.radius = float(radius)

    def is_inside(self, point, inflation: float = 0.0) -> bool:
        point = np.asarray(point, dtype=float)
        horizontal = float(np.linalg.norm(point[:2] - self.center[:2]))
        if horizontal > self.radius + inflation:
            return False
        z = self.center[2] + self.height / 2.0 if point.size == 2 else point[2]
        return bool(self.center[2] <= z <= self.center[2] + self.height + inflation)


class BoxObstacle:
    def __init__(self, center, size):
        self.center = np.asarray(center, dtype=float)
        self.size = np.asarray(size, dtype=float)
        self.half_size = self.size / 2.0

    def is_inside(self, point, inflation: float = 0.0) -> bool:
        point = np.asarray(point, dtype=float)
        z = self.center[2] if point.size == 2 else point[2]
        xyz = np.array([point[0], point[1], z], dtype=float)
        return bool(np.all(np.abs(xyz - self.center) <= self.half_size + inflation))


class HeightFieldTerrain:
    def __init__(self, xs: np.ndarray, ys: np.ndarray, heights: np.ndarray):
        self.xs = np.asarray(xs, dtype=float)
        self.ys = np.asarray(ys, dtype=float)
        self.heights = np.asarray(heights, dtype=float)
        expected = (self.xs.size, self.ys.size)
        if self.heights.shape != expected:
            raise ValueError(f"heights shape {self.heights.shape} does not match {expected}")

    @property
    def center(self) -> np.ndarray:
        return np.array(
            [
                float(np.mean(self.xs)) if self.xs.size else 0.0,
                float(np.mean(self.ys)) if self.ys.size else 0.0,
                0.0,
            ]
        )

    def height_at(self, x: float, y: float) -> float:
        if not self.xs.size or not self.ys.size:
            return 0.0
        i = int(np.clip(np.searchsorted(self.xs, x), 0, self.xs.size - 1))
        j = int(np.clip(np.searchsorted(self.ys, y), 0, self.ys.size - 1))
        return float(self.heights[i, j])

    def is_inside(self, point, inflation: float = 0.0) -> bool:
        if len(point) == 2:
            return False
        return bool(float(point[2]) <= self.height_at(float(point[0]), float(point[1])) + inflation)


def is_point_in_collision(point, obstacles, inflation: float = 0.0) -> bool:
    return any(obstacle.is_inside(point, inflation) for obstacle in obstacles)


@dataclass
class TerrainConfig:
    terrain_type: TerrainType = "plains"
    space_dim: np.ndarray = field(default_factory=lambda: np.array([120.0, 120.0, 60.0]))
    start_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    forest: dict | None = None
    mountains: dict | None = None
    plains: dict | None = None


def load_terrain_config(yaml_path: Path | str | None = None) -> TerrainConfig:
    try:
        import yaml
    except ModuleNotFoundError as exc:  # pragma: no cover
        raise RuntimeError('Terrain config requires PyYAML. Install "aerial-kit[sim]".') from exc

    if yaml_path is None:
        resource = files("aerial_kit.sim.defaults").joinpath("terrain.yaml")
        with as_file(resource) as resource_path:
            raw = yaml.safe_load(resource_path.read_text(encoding="utf-8")) or {}
    else:
        raw = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}

    # Accept both standalone and ROS parameter-file layouts.
    params = raw.get("terrain", raw.get("terrain_generator", {}).get("ros__parameters", raw))
    return TerrainConfig(
        terrain_type=params.get("terrain_type", "plains"),
        space_dim=np.asarray(params.get("space_dim", [120.0, 120.0, 60.0]), dtype=float),
        start_pos=np.asarray(params.get("start_pos", [0.0, 0.0, 0.0]), dtype=float),
        forest=params.get("forest"),
        mountains=params.get("mountains"),
        plains=params.get("plains"),
    )


def _sample_xy(space: np.ndarray, margin_x: float, margin_y: float) -> tuple[float, float]:
    return (
        float(np.random.uniform(margin_x, max(margin_x + 1e-6, space[0] - margin_x))),
        float(np.random.uniform(margin_y, max(margin_y + 1e-6, space[1] - margin_y))),
    )


def generate_forest(
    space_dim,
    grid_size: int = 10,
    radius_range=(0.5, 1.5),
    height_range=(5.0, 15.0),
    density: float = 0.7,
    start_pos=None,
    min_start_distance: float = 3.0,
    local_cluster_radius: float = 0.0,
    local_cluster_num_trees: int = 0,
):
    space = np.asarray(space_dim, dtype=float)
    start = np.zeros(3) if start_pos is None else np.asarray(start_pos, dtype=float)
    trees: list[CylinderObstacle] = []
    cell_x, cell_y = space[0] / grid_size, space[1] / grid_size
    for i in range(grid_size):
        for j in range(grid_size):
            if np.random.random() > density:
                continue
            radius = float(np.random.uniform(*radius_range))
            x_lo, x_hi = i * cell_x + radius, (i + 1) * cell_x - radius
            y_lo, y_hi = j * cell_y + radius, (j + 1) * cell_y - radius
            if x_lo >= x_hi or y_lo >= y_hi:
                continue
            center = np.array([np.random.uniform(x_lo, x_hi), np.random.uniform(y_lo, y_hi), 0.0])
            if local_cluster_radius > 0 and np.linalg.norm(center[:2] - start[:2]) > local_cluster_radius:
                continue
            if np.linalg.norm(center[:2] - start[:2]) < radius + min_start_distance:
                continue
            trees.append(CylinderObstacle(center, np.random.uniform(*height_range), radius))

    for _ in range(max(0, int(local_cluster_num_trees))):
        theta = np.random.uniform(0.0, 2.0 * np.pi)
        rho = max(0.0, local_cluster_radius) * np.sqrt(np.random.random())
        radius = float(np.random.uniform(*radius_range))
        center = np.array([start[0] + rho * np.cos(theta), start[1] + rho * np.sin(theta), 0.0])
        if (
            radius <= center[0] <= space[0] - radius
            and radius <= center[1] <= space[1] - radius
            and np.linalg.norm(center[:2] - start[:2]) >= radius + min_start_distance
        ):
            trees.append(CylinderObstacle(center, np.random.uniform(*height_range), radius))
    return trees


def generate_plains(
    space_dim,
    num_obstacles: int = 10,
    obstacle_types: Sequence[str] = ("bush", "rock"),
    start_pos=None,
    min_start_distance: float = 3.0,
    local_cluster_radius: float = 0.0,
):
    del local_cluster_radius
    space = np.asarray(space_dim, dtype=float)
    start = np.zeros(3) if start_pos is None else np.asarray(start_pos, dtype=float)
    obstacles: list[CylinderObstacle | BoxObstacle] = []
    for _ in range(int(num_obstacles)):
        kind = str(np.random.choice(obstacle_types))
        if kind in {"bush", "tree"}:
            radius = float(np.random.uniform(0.3, 0.8) if kind == "bush" else np.random.uniform(0.4, 1.0))
            height = float(np.random.uniform(0.5, 2.0) if kind == "bush" else np.random.uniform(3.0, 8.0))
            x, y = _sample_xy(space, radius, radius)
            if np.linalg.norm(np.array([x, y]) - start[:2]) >= radius + min_start_distance:
                obstacles.append(CylinderObstacle([x, y, 0.0], height, radius))
        elif kind == "rock":
            size = np.array([np.random.uniform(0.5, 2.0), np.random.uniform(0.5, 2.0), np.random.uniform(0.3, 1.5)])
            x, y = _sample_xy(space, size[0] / 2.0, size[1] / 2.0)
            if np.linalg.norm(np.array([x, y]) - start[:2]) >= size[0] / 2.0 + min_start_distance:
                obstacles.append(BoxObstacle([x, y, size[2] / 2.0], size))
    return obstacles


def generate_mountains(
    space_dim,
    num_peaks: int = 12,
    base_size_range=(8.0, 22.0),
    height_range=(8.0, 30.0),
    start_pos=None,
    min_start_distance: float = 5.0,
    grid_resolution: float = 4.0,
    ridge_count: int = 0,
    ridge_chain_count: int = 0,
    ridge_chain_points: int = 4,
    ridge_chain_width: float = 18.0,
    ridge_chain_peak_spacing: float = 18.0,
    ridge_chain_peak_boost: float = 1.0,
    pit_count: int = 0,
    pit_depth_range=(5.0, 20.0),
    steepness: float = 1.4,
    min_height: float = 0.5,
):
    del ridge_count, ridge_chain_count, ridge_chain_points, ridge_chain_width
    del ridge_chain_peak_spacing, ridge_chain_peak_boost, min_height
    space = np.asarray(space_dim, dtype=float)
    start = np.zeros(3) if start_pos is None else np.asarray(start_pos, dtype=float)
    resolution = max(0.5, float(grid_resolution))
    xs = np.arange(resolution / 2.0, space[0], resolution)
    ys = np.arange(resolution / 2.0, space[1], resolution)
    x_grid, y_grid = np.meshgrid(xs, ys, indexing="ij")
    heights = np.zeros_like(x_grid)
    for _ in range(int(num_peaks)):
        x0, y0 = _sample_xy(space, 0.0, 0.0)
        sigma = float(np.random.uniform(*base_size_range))
        amplitude = float(np.random.uniform(*height_range))
        heights += amplitude * np.exp(-((x_grid - x0) ** 2 + (y_grid - y0) ** 2) / (2.0 * sigma**2))
    for _ in range(int(pit_count)):
        x0, y0 = _sample_xy(space, 0.0, 0.0)
        sigma = float(np.random.uniform(*base_size_range))
        depth = float(np.random.uniform(*pit_depth_range))
        heights -= depth * np.exp(-((x_grid - x0) ** 2 + (y_grid - y0) ** 2) / (2.0 * sigma**2))
    heights = np.clip(heights, 0.0, None)
    heights[np.hypot(x_grid - start[0], y_grid - start[1]) < min_start_distance] = 0.0
    if heights.size and heights.max() > 0:
        heights = (heights / heights.max()) ** max(0.5, steepness) * min(heights.max(), space[2])
    return [HeightFieldTerrain(xs, ys, np.clip(heights, 0.0, space[2]))]


def generate_terrain(
    cfg: TerrainConfig,
    forest_density_scale: float = 1.0,
    tree_height_scale: float = 1.0,
):
    if cfg.terrain_type == "forest":
        params = dict(cfg.forest or {})
        params["density"] = np.clip(float(params.get("density", 0.7)) * forest_density_scale, 0.0, 1.0)
        params["height_range"] = tuple(float(v) * tree_height_scale for v in params.get("height_range", [5.0, 15.0]))
        return generate_forest(cfg.space_dim, start_pos=cfg.start_pos, **params)
    if cfg.terrain_type == "mountains":
        return generate_mountains(cfg.space_dim, start_pos=cfg.start_pos, **dict(cfg.mountains or {}))
    if cfg.terrain_type == "plains":
        return generate_plains(cfg.space_dim, start_pos=cfg.start_pos, **dict(cfg.plains or {}))
    raise ValueError(f"Unknown terrain_type: {cfg.terrain_type}")


__all__ = [
    "BoxObstacle", "CylinderObstacle", "HeightFieldTerrain", "TerrainConfig",
    "generate_terrain", "is_point_in_collision", "load_terrain_config",
]
