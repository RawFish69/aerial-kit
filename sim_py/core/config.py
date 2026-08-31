"""Config loading and normalization for the standalone simulator."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

@dataclass
class NormalizedSimConfig:
    """Normalized config used by the simulation runner."""

    sim_config_path: Path
    terrain_override: str | None
    terrain_config_path: str | None
    controller_name: str
    backend_name: str
    dt: float
    sim_time: float
    path_cfg: dict[str, Any]
    controller_cfg: dict[str, Any]
    visual_cfg: dict[str, Any]
    simulation_cfg: dict[str, Any]
    raw_cfg: dict[str, Any]
    seed: int | None
    airframe_name: str = "quad"
    initial_state_cfg: dict[str, Any] = field(default_factory=dict)


def load_sim_config(path: Path) -> dict[str, Any]:
    """Load simulator YAML config from disk.

    Missing files are handled gracefully and return an empty config.
    """
    if not path.exists():
        return {}

    import yaml

    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def normalize_sim_config(args: argparse.Namespace) -> NormalizedSimConfig:
    """Normalize legacy and new config fields into one runtime object."""
    sim_config_path = Path(args.sim_config)
    raw_cfg = load_sim_config(sim_config_path)

    config = config_from_mapping(raw_cfg, sim_config_path=sim_config_path)
    terrain_config_path = args.terrain_config
    if terrain_config_path is None:
        source_tree_terrain = (
            Path(__file__).resolve().parents[2]
            / "ros2_ws"
            / "src"
            / "terrain_generator"
            / "config"
            / "terrain_params.yaml"
        )
        if source_tree_terrain.exists():
            terrain_config_path = str(source_tree_terrain)
    planner_override = getattr(args, "planner", None)
    path_cfg = (
        {**config.path_cfg, "planner_type": str(planner_override)}
        if planner_override is not None
        else config.path_cfg
    )
    return NormalizedSimConfig(
        **{
            **vars(config),
            "path_cfg": path_cfg,
            "terrain_override": args.terrain,
            "terrain_config_path": terrain_config_path,
            "controller_name": str(args.controller) if args.controller is not None else config.controller_name,
            "backend_name": str(args.backend) if args.backend is not None else config.backend_name,
            "dt": float(args.dt) if args.dt is not None else config.dt,
            "sim_time": float(args.sim_time) if args.sim_time is not None else config.sim_time,
            "airframe_name": (
                str(args.airframe)
                if getattr(args, "airframe", None) is not None
                else config.airframe_name
            ),
        }
    )


def config_from_mapping(
    raw_cfg: dict[str, Any],
    sim_config_path: Path | str = Path("<mapping>"),
) -> NormalizedSimConfig:
    """Normalize a config mapping for the public Python API and legacy CLI."""
    sim_config_path = Path(sim_config_path)

    path_cfg = dict(raw_cfg.get("path", {}) or {})
    ctrl_cfg = dict(raw_cfg.get("controller", {}) or {})
    vis_cfg = dict(raw_cfg.get("visual", {}) or {})
    sim_cfg = dict(raw_cfg.get("simulation", {}) or {})

    controller_name = str(ctrl_cfg.get("controller_type", ctrl_cfg.get("type", "pid")))

    # New backend selection with additive config support.
    backend_name = str(sim_cfg.get("backend", "pointmass"))

    dt = float(sim_cfg.get("dt", ctrl_cfg.get("dt", 0.01)))
    sim_time = float(sim_cfg.get("duration", ctrl_cfg.get("sim_time", 20.0)))

    seed_raw = sim_cfg.get("seed")
    seed = None if seed_raw is None else int(seed_raw)

    vehicle_cfg = dict(raw_cfg.get("vehicle", {}) or {})
    airframe_name = str(vehicle_cfg.get("airframe", "quad"))
    terrain_raw = raw_cfg.get("terrain")
    terrain_override = str(terrain_raw) if isinstance(terrain_raw, str) else None

    return NormalizedSimConfig(
        sim_config_path=sim_config_path,
        terrain_override=terrain_override,
        terrain_config_path=raw_cfg.get("terrain_config"),
        controller_name=controller_name,
        backend_name=backend_name,
        dt=dt,
        sim_time=sim_time,
        path_cfg=path_cfg,
        controller_cfg=ctrl_cfg,
        visual_cfg=vis_cfg,
        simulation_cfg=sim_cfg,
        raw_cfg=raw_cfg,
        seed=seed,
        airframe_name=airframe_name,
        initial_state_cfg=dict(raw_cfg.get("initial_state", {}) or {}),
    )
