"""Stable public API around the standalone simulation implementation."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from sim_py.core.config import NormalizedSimConfig, config_from_mapping
from sim_py.core.runner import run_simulation as _run_simulation

from .result import SimulationResult

SimulationConfig = NormalizedSimConfig


def load_config(path: str | Path) -> SimulationConfig:
    """Load and normalize an aerial-kit simulator YAML file."""
    try:
        import yaml
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on install extras
        raise RuntimeError(
            'Simulation config support requires PyYAML. Install "aerial-kit[sim]".'
        ) from exc

    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"Simulation config must be a YAML mapping: {config_path}")
    return config_from_mapping(dict(raw), sim_config_path=config_path)


def run_simulation(config: SimulationConfig | Mapping[str, Any] | str | Path) -> SimulationResult:
    """Run a simulation from a normalized config, mapping, or YAML path."""
    if isinstance(config, (str, Path)):
        normalized = load_config(config)
    elif isinstance(config, NormalizedSimConfig):
        normalized = config
    elif isinstance(config, Mapping):
        normalized = config_from_mapping(dict(config))
    else:
        raise TypeError("config must be a SimulationConfig, mapping, or YAML path")
    return _run_simulation(normalized)
