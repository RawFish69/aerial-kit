"""The actuator backend: wrench controllers flying planned missions in the runner."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from aerial_kit.sim import load_config, run_simulation
from aerial_kit.types import ControlTarget, SimState, Wrench
from aerial_kit.dynamics.rotations import rotmat_to_quat, quat_to_rotmat
from sim_py.backends.actuator_backend import ActuatorBackend

REPO_ROOT = Path(__file__).resolve().parents[2]
QUAD = REPO_ROOT / "examples" / "quadrotor" / "config.yaml"


@pytest.fixture(scope="module")
def config():
    return load_config(QUAD)


def _backend(**actuator):
    b = ActuatorBackend()
    yaw = 0.7
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    b.reset(
        SimState(position=np.array([1.0, 2.0, 3.0]), velocity=np.array([0.5, 0.0, 0.0]), attitude_quat=rotmat_to_quat(R)),
        world={}, cfg={"simulation": {"actuator": actuator}},
    )
    return b, R


def test_reset_round_trips_the_state_through_the_plants_frames():
    b, R = _backend(mass_kg=1.3)
    s = b.state()
    np.testing.assert_allclose(s.position, [1.0, 2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(s.velocity, [0.5, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(quat_to_rotmat(s.attitude_quat), R, atol=1e-12)
    assert b.mass_kg() == 1.3


def test_hover_wrench_holds_altitude():
    b, R = _backend()
    hover = ControlTarget(accel_cmd=np.zeros(3), wrench=Wrench(np.array([0.0, 0.0, 9.81]), np.zeros(3)))
    b.step(hover, 0.5)  # starts at hover thrust, so nothing should change
    s = b.state()
    assert abs(s.position[2] - (3.0 + 0.0)) < 0.02
    np.testing.assert_allclose(quat_to_rotmat(s.attitude_quat), R, atol=1e-6)


def test_an_acceleration_only_target_is_refused():
    b, _ = _backend()
    with pytest.raises(ValueError, match="geometric, nmpc, cascade"):
        b.step(ControlTarget(accel_cmd=np.zeros(3)), 0.01)


@pytest.mark.parametrize("controller", ["geometric", "cascade"])
def test_wrench_controllers_fly_the_quadrotor_example_on_motors(config, controller):
    result = run_simulation(replace(config, controller_name=controller, backend_name="actuator", sim_time=60.0))
    assert result.distance_to_goal <= 3.0
    assert result.collisions_detected == 0


def test_an_acceleration_controller_is_refused_on_the_actuator_backend(config):
    with pytest.raises(ValueError, match="needs a controller that returns a wrench"):
        run_simulation(replace(config, controller_name="pid", backend_name="actuator", sim_time=1.0))


def test_the_trajectory_mode_is_validated_and_waypoints_mode_works(config):
    with pytest.raises(ValueError, match="path.trajectory"):
        run_simulation(replace(
            config, controller_name="geometric", backend_name="actuator", sim_time=1.0,
            path_cfg={**config.path_cfg, "trajectory": "splines"},
        ))
    # Point-to-point on the planned waypoints instead of a trajectory.
    result = run_simulation(replace(
        config, controller_name="geometric", backend_name="actuator", sim_time=60.0,
        path_cfg={**config.path_cfg, "trajectory": "waypoints"},
    ))
    assert result.distance_to_goal <= 3.0
