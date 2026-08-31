"""Backward-compatible terrain exports.

The canonical ROS-free implementation now lives in :mod:`aerial_kit.sim.terrain`.
"""

from aerial_kit.sim.terrain import (
    BoxObstacle,
    CylinderObstacle,
    HeightFieldTerrain,
    TerrainConfig,
    generate_terrain,
    is_point_in_collision,
    load_terrain_config,
)

__all__ = [
    "BoxObstacle",
    "CylinderObstacle",
    "HeightFieldTerrain",
    "TerrainConfig",
    "generate_terrain",
    "is_point_in_collision",
    "load_terrain_config",
]
