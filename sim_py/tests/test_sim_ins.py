"""The INS EKF in the simulation loop: simulated sensors, and the runner using it.

The truth is the analytic flight ``test_ins_ekf.py`` uses, sampled as a
sequence of ``SimState`` the way a backend hands them out, so the sensor
synthesis (specific force from the change in velocity, GPS and baro at their
own rates) is checked as well as the filter behind it.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from aerial_kit.dynamics.rotations import rotmat_to_quat
from aerial_kit.estimation import InsConfig, SensorConfig, SimulatedIns
from aerial_kit.sim import load_config, run_simulation
from aerial_kit.types import SimState

QUAD = Path(__file__).resolve().parents[2] / "examples" / "quadrotor" / "config.yaml"
DT = 0.02
G = 9.81


def truth(t):
    """Position, velocity, acceleration (world ENU) and heading at time t."""
    p = np.array([10 * np.sin(0.2 * t), 6 * np.sin(0.4 * t), 5 + 2 * np.sin(0.3 * t)])
    v = np.array([2 * np.cos(0.2 * t), 2.4 * np.cos(0.4 * t), 0.6 * np.cos(0.3 * t)])
    a = np.array([-0.4 * np.sin(0.2 * t), -0.96 * np.sin(0.4 * t), -0.18 * np.sin(0.3 * t)])
    return p, v, a, 0.1 * t


def attitude(a, yaw):
    """A quadrotor's attitude for acceleration ``a``: thrust along a + g e3."""
    b3 = a + np.array([0.0, 0.0, G])
    b3 /= np.linalg.norm(b3)
    c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    b2 = np.cross(b3, c)
    b2 /= np.linalg.norm(b2)
    return np.column_stack([np.cross(b2, b3), b2, b3])


def states(duration=60.0, dt=DT):
    """True states along the analytic flight, as a backend would report them."""
    for k in range(int(duration / dt) + 1):
        t = k * dt
        p, v, a, yaw = truth(t)
        yield SimState(position=p, velocity=v, t=t, attitude_quat=rotmat_to_quat(attitude(a, yaw)))


def fly(ins: SimulatedIns, duration=60.0, dt=DT):
    errors, sigmas = [], []
    it = states(duration, dt)
    ins.reset(next(it))
    for s in it:
        est = ins.update(s)
        errors.append(est.position - s.position)
        sigmas.append(ins.filter.position_sigma())
    return np.array(errors), np.array(sigmas)


def test_perfect_sensors_track_the_truth():
    ins = SimulatedIns(SensorConfig(gps_sigma_xy_m=0.0, gps_sigma_z_m=0.0, baro_sigma_m=0.0, accel_sigma_mps2=0.0))
    errors, _ = fly(ins, duration=20.0)
    # The accelerometer is the average specific force over each step, so the
    # prediction between fixes is exact: the same attitude rotates the specific
    # force into and back out of body axes.
    assert np.max(np.linalg.norm(errors, axis=1)) < 1e-4


def test_noisy_sensors_beat_the_gps_and_the_filter_knows_how_well():
    ins = SimulatedIns(SensorConfig(gps_sigma_xy_m=1.5, gps_sigma_z_m=3.0, seed=4))
    errors, sigmas = fly(ins)
    settled = slice(len(errors) // 4, None)
    rms = np.sqrt(np.mean(errors[settled] ** 2, axis=0))
    assert np.all(rms[:2] < 0.4 * 1.5)
    # Consistency: the error stays inside three of the filter's own sigmas
    # almost all the time - an over-confident filter fails this, an
    # under-confident one fails the RMS bound above.
    inside = np.abs(errors[settled]) <= 3.0 * sigmas[settled]
    assert inside.mean() > 0.97


def test_the_accelerometer_bias_and_baro_offset_are_learned():
    bias = (0.15, -0.1, 0.2)
    ins = SimulatedIns(SensorConfig(accel_bias_mps2=bias, baro_offset_m=2.5, seed=1))
    fly(ins, duration=90.0)
    np.testing.assert_allclose(ins.filter.accel_bias, bias, atol=0.02)
    assert ins.filter.baro_bias == pytest.approx(2.5, abs=0.3)


def test_sensor_rates_are_honoured():
    ins = SimulatedIns(SensorConfig(gps_rate_hz=5.0, baro_rate_hz=20.0))
    fly(ins, duration=10.0)
    assert ins.gps_updates == pytest.approx(50, abs=1)
    assert ins.baro_updates == pytest.approx(200, abs=1)
    # A step longer than the GPS period still gets one fix per step, not a backlog.
    ins = SimulatedIns(SensorConfig(gps_rate_hz=20.0))
    fly(ins, duration=10.0, dt=0.2)
    assert ins.gps_updates == pytest.approx(50, abs=1)


def test_a_level_backend_without_attitude_works():
    ins = SimulatedIns(SensorConfig(seed=2))
    ins.reset(SimState(position=np.zeros(3), velocity=np.zeros(3), t=0.0))
    for k in range(1, 500):
        est = ins.update(SimState(position=np.array([0.5 * k * DT, 0.0, 0.0]),
                                  velocity=np.array([0.5, 0.0, 0.0]), t=k * DT))
    assert est.attitude_quat is None
    assert abs(est.velocity[0] - 0.5) < 0.3


def test_the_same_seed_gives_the_same_run():
    a, _ = fly(SimulatedIns(SensorConfig(seed=9)), duration=5.0)
    b, _ = fly(SimulatedIns(SensorConfig(seed=9)), duration=5.0)
    c, _ = fly(SimulatedIns(SensorConfig(seed=10)), duration=5.0)
    np.testing.assert_array_equal(a, b)
    assert not np.array_equal(a, c)


def test_from_config_reads_sensors_and_filter_and_refuses_typos():
    ins = SimulatedIns.from_config({"mode": "ekf", "gps_sigma_xy_m": 2.0, "ekf": {"gps_sigma_xy": 4.0}})
    assert ins.sensors.gps_sigma_xy_m == 2.0
    assert ins.ekf.gps_sigma_xy == 4.0
    assert ins.ekf.accel_mode == "body_specific_force"
    with pytest.raises(ValueError, match="gps_sigma"):
        SimulatedIns.from_config({"gps_sigma": 1.0})
    with pytest.raises(ValueError, match="estimator.ekf"):
        SimulatedIns.from_config({"ekf": {"bogus": 1.0}})
    with pytest.raises(ValueError, match="body specific force"):
        SimulatedIns(ekf=InsConfig(accel_mode="none"))
    with pytest.raises(ValueError, match="gps_rate_hz"):
        SensorConfig(gps_rate_hz=0.0)


@pytest.fixture(scope="module")
def config():
    return load_config(QUAD)


def _run(config, controller="pid", backend="multirotor", **estimator):
    sim = {**config.simulation_cfg, "estimator": estimator}
    return run_simulation(replace(config, controller_name=controller, backend_name=backend,
                                  sim_time=60.0, simulation_cfg=sim))


def test_the_runner_flies_on_the_estimate_and_records_it(config):
    result = _run(config, mode="ekf")
    assert result.estimated_trajectory is not None
    assert result.estimated_trajectory.shape == result.trajectory.shape
    assert 0.0 < result.estimation_error_rms() < 1.0
    assert result.distance_to_goal <= 3.5
    assert result.collisions_detected == 0


def test_truth_is_the_default_and_records_no_estimate(config):
    result = run_simulation(replace(config, sim_time=5.0))
    assert result.estimated_trajectory is None
    assert result.estimation_error_rms() is None


def test_worse_gps_means_a_worse_estimate(config):
    good = _run(config, mode="ekf", gps_sigma_xy_m=0.3, gps_sigma_z_m=0.5)
    bad = _run(config, mode="ekf", gps_sigma_xy_m=3.0, gps_sigma_z_m=5.0)
    assert bad.estimation_error_rms() > 1.5 * good.estimation_error_rms()


def test_an_unknown_estimator_mode_is_refused(config):
    with pytest.raises(ValueError, match="estimator.mode"):
        _run(config, mode="ukf")
