"""Optional, ROS-free simulation API for :mod:`aerial_kit`.

Install it with ``pip install "aerial-kit[sim]"``. Importing the base
``aerial_kit`` package does not import Matplotlib or PyYAML.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .api import SimulationConfig
    from .result import SimulationResult


def __getattr__(name: str) -> Any:
    if name == "SimulationResult":
        from .result import SimulationResult

        return SimulationResult
    if name in {"SimulationConfig", "load_config", "run_simulation"}:
        from . import api

        return getattr(api, name)
    raise AttributeError(name)

__all__ = ["SimulationConfig", "SimulationResult", "load_config", "run_simulation"]
