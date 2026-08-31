"""Optional Matplotlib visualization for simulation results."""

from __future__ import annotations

from typing import Any

from aerial_kit.sim.result import SimulationResult


def plot_simulation(result: SimulationResult, *, show: bool = True, **overrides: Any):
    """Plot a :class:`~aerial_kit.sim.SimulationResult` and return its figure.

    Matplotlib is imported lazily, so the base aerial-kit package remains usable
    without visualization dependencies.
    """
    try:
        from sim_py.visualizer import plot_simulation as _plot
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on install extras
        if exc.name and exc.name.startswith("matplotlib"):
            raise RuntimeError(
                'Visualization requires Matplotlib. Install "aerial-kit[viz]".'
            ) from exc
        raise

    options = {
        "space_dim": result.space_dim,
        "terrain_type": result.terrain_type,
        "visual_cfg": result.visual_cfg,
        "planned_waypoints": result.planned_waypoints,
        "goal_position": result.goal_position,
        "planner_type": result.planner_type,
        "attitude_quats": result.attitude_quats,
        "backend_name": result.backend_name,
        "show": show,
    }
    options.update(overrides)
    return _plot(result.trajectory, result.obstacles, **options)


def plot_follow_view(result: SimulationResult, *, show: bool = True, **overrides: Any):
    """Local chase-camera plot of an aircraft following its path."""
    try:
        from sim_py.visualizer import plot_follow_view as _plot
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on install extras
        if exc.name and exc.name.startswith("matplotlib"):
            raise RuntimeError(
                'Visualization requires Matplotlib. Install "aerial-kit[viz]".'
            ) from exc
        raise

    options = {
        "attitude_quats": result.attitude_quats,
        "planned_waypoints": result.planned_waypoints,
        "space_dim": result.space_dim,
        "backend_name": result.backend_name,
        "planner_type": result.planner_type,
        "show": show,
    }
    options.update(overrides)
    return _plot(result.trajectory, **options)


__all__ = ["plot_simulation", "plot_follow_view"]
