"""Wind on the actuator plant: steady wind, gusts, and the runner flying in both.

Wind enters through the drag term alone, so everything here is checkable
against the one-line model ``F = -kv (v - w)``: a hovering aircraft is carried
downwind with time constant ``m / kv``, the accelerometer feels the drag, and a
wrench controller has to lean into it.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from aerial_kit.dynamics.multirotor_actuator import ActuatorPlant, ActuatorPlantParams
from aerial_kit.sim import load_config, run_simulation
from aerial_kit.types import ControlTarget, SimState, Wrench
from sim_py.backends.actuator_backend import ActuatorBackend

QUAD = Path(__file__).resolve().parents[2] / "examples" / "quadrotor" / "config.yaml"


def hovering(**params) -> ActuatorPlant:
    """A plant in the air, motors spun up to hover, ground out of the way."""
    plant = ActuatorPlant(params=ActuatorPlantParams(ground_enabled=False, **params))
    plant.position[2] = -10.0
    plant.set_hover()
    plant.motor_actual[...] = plant.motor_command
    return plant


def test_a_hovering_aircraft_is_carried_downwind_at_the_drag_time_constant():
    wind = np.array([4.0, -2.0, 0.0])
    plant = hovering(wind_ned_mps=tuple(wind), kv_drag=0.2, mass_kg=1.0)
    tau = plant.params.mass_kg / plant.params.kv_drag
    for _ in range(int(5.0 / 0.002)):
        plant.step(0.002)
    expected = wind * (1.0 - np.exp(-5.0 / tau))
    np.testing.assert_allclose(plant.velocity, expected, atol=1e-6)


def test_still_air_is_the_old_plant_exactly():
    a, b = hovering(), hovering(wind_ned_mps=(0.0, 0.0, 0.0), gust_sigma_mps=0.0)
    a.velocity[:] = b.velocity[:] = [1.0, 0.5, 0.0]
    for _ in range(500):
        a.step(0.002)
        b.step(0.002)
    np.testing.assert_array_equal(a.position, b.position)
    np.testing.assert_array_equal(a.velocity, b.velocity)


def test_the_accelerometer_feels_the_wind():
    plant = hovering(wind_ned_mps=(5.0, 0.0, 0.0), kv_drag=0.2, mass_kg=1.0)
    s = plant.sense()
    # Drag pushes north (+x, level body) at kv*w/m = 1 m/s^2; the firmware's
    # accelerometer is the specific force negated.
    assert s.accel_g[0] == pytest.approx(-1.0 / 9.81, rel=1e-9)
    assert s.accel_g[1] == pytest.approx(0.0, abs=1e-12)


def test_gusts_have_the_configured_spread_and_correlation_time():
    sigma, tau, dt = 1.5, 1.0, 0.01
    plant = hovering(gust_sigma_mps=sigma, gust_tau_s=tau, motor_tau_s=0.03, seed=3)
    samples = []
    for _ in range(int(600.0 / dt)):
        plant._advance_gust(dt)
        samples.append(plant.gust.copy())
    g = np.array(samples)
    assert np.all(np.abs(g.mean(axis=0)) < 0.3)
    np.testing.assert_allclose(g.std(axis=0), sigma, rtol=0.15)
    lag = int(tau / dt)
    x = g[:, 0] - g[:, 0].mean()
    rho = float(np.dot(x[:-lag], x[lag:]) / np.dot(x, x))
    assert rho == pytest.approx(np.exp(-1.0), abs=0.12)


def test_gusts_do_not_change_the_sensor_noise_sequence():
    # Same seed, gusts on and off: the gust generator is separate, and wind
    # makes no moment, so the gyro reads the same noise either way.
    a = hovering(gyro_noise_dps=0.5, seed=11)
    b = hovering(gyro_noise_dps=0.5, seed=11, gust_sigma_mps=2.0)
    for _ in range(50):
        a.step(0.002)
        b.step(0.002)
        np.testing.assert_allclose(a.sense().gyro_rps, b.sense().gyro_rps, atol=1e-12)
    assert np.linalg.norm(b.gust) > 0.0


@pytest.mark.parametrize(
    "params, match",
    [
        ({"wind_ned_mps": (1.0, 2.0)}, "3 components"),
        ({"gust_sigma_mps": -1.0}, "gust_sigma_mps"),
        ({"gust_tau_s": 0.0}, "gust_tau_s"),
    ],
)
def test_bad_wind_settings_are_refused(params, match):
    with pytest.raises(ValueError, match=match):
        ActuatorPlant(params=ActuatorPlantParams(**params))


def _backend(**actuator) -> ActuatorBackend:
    b = ActuatorBackend()
    b.reset(
        SimState(position=np.array([0.0, 0.0, 10.0]), velocity=np.zeros(3), attitude_quat=np.array([1.0, 0, 0, 0])),
        world={}, cfg={"simulation": {"actuator": actuator}},
    )
    return b


def test_the_backend_takes_the_wind_in_the_runners_enu_frame():
    b = _backend(wind_mps=[3.0, 0.0, 0.0], kv_drag=0.3)
    np.testing.assert_allclose(b.plant.params.wind_ned_mps, [0.0, 3.0, 0.0])
    hover = ControlTarget(accel_cmd=np.zeros(3), wrench=Wrench(np.array([0.0, 0.0, 9.81]), np.zeros(3)))
    b.step(hover, 2.0)
    v = b.state().velocity
    assert v[0] > 0.3  # carried east
    assert abs(v[1]) < 1e-9 and abs(v[2]) < 1e-3


def test_the_backend_refuses_the_wind_given_twice():
    with pytest.raises(ValueError, match="not both"):
        _backend(wind_mps=[1.0, 0.0, 0.0], wind_ned_mps=[0.0, 1.0, 0.0])


@pytest.fixture(scope="module")
def config():
    return load_config(QUAD)


def _windy(config, controller, wind, gust, **estimator):
    actuator = {**config.simulation_cfg.get("actuator", {}), "wind_mps": wind, "gust_sigma_mps": gust,
                "kv_drag": 0.3}
    sim = {**config.simulation_cfg, "actuator": actuator}
    if estimator:
        sim["estimator"] = estimator
    return run_simulation(replace(config, controller_name=controller, backend_name="actuator",
                                  sim_time=60.0, simulation_cfg=sim))


@pytest.mark.parametrize("controller", ["geometric", "nmpc"])
def test_wrench_controllers_finish_the_mission_in_a_gusty_crosswind(config, controller):
    result = _windy(config, controller, wind=[0.0, -5.0, 0.0], gust=1.0)
    assert result.distance_to_goal <= 3.0
    assert result.collisions_detected == 0


def test_geometric_finishes_in_wind_on_the_estimated_state(config):
    result = _windy(config, "geometric", wind=[0.0, -5.0, 0.0], gust=1.0, mode="ekf")
    assert result.distance_to_goal <= 3.5
    assert result.collisions_detected == 0
    assert result.estimation_error_rms() < 1.0
