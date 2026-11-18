from pathlib import Path

import numpy as np
import pytest

import aerial_kit
from aerial_kit import (
    Controller,
    DynamicsBackend,
    Planner,
    SimState,
    Waypoint,
    create_airframe,
    create_controller,
    register_builtin_components,
)


def test_top_level_exports_are_exactly_the_documented_public_api() -> None:
    assert aerial_kit.__all__ == [
        "Capabilities",
        "CommandKind",
        "Controller",
        "ControlMode",
        "ControlTarget",
        "DynamicsBackend",
        "Planner",
        "SimState",
        "TrajectoryLog",
        "Waypoint",
        "Wrench",
        "create_airframe",
        "create_backend",
        "create_controller",
        "create_planner",
        "register_airframe",
        "register_backend",
        "register_builtin_components",
        "register_controller",
        "register_planner",
    ]
    assert all(hasattr(aerial_kit, name) for name in aerial_kit.__all__)


def test_public_interfaces_and_registry_work_together() -> None:
    register_builtin_components()
    airframe = create_airframe("quad")
    controller = create_controller("pid")
    assert isinstance(controller, Controller)
    assert not isinstance(airframe, DynamicsBackend)
    assert not isinstance(airframe, Planner)
    target = controller.compute(
        SimState(position=np.zeros(3), velocity=np.zeros(3)),
        Waypoint(position=np.array([1.0, 0.0, 1.0])),
        {},
    )
    assert target.accel_cmd.shape == (3,)


def test_readme_quickstart_runs() -> None:
    readme = Path(__file__).with_name("aerial_kit").joinpath("README.md")
    text = readme.read_text(encoding="utf-8")
    section = text.split("## Quick start", 1)[1].split("```python", 1)[1].split("```", 1)[0]
    namespace = {"__name__": "__readme_quickstart__"}
    exec(compile(section, str(readme), "exec"), namespace)
    assert "target" in namespace
    assert namespace["target"].accel_cmd.shape == (3,)
    assert namespace["airframe"].capabilities.n_actuators == 4
