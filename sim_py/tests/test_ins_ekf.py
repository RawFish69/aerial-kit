"""INS EKF against a synthetic flight with known truth.

The truth is a smooth 3-D trajectory with a turning heading, so attitude,
specific force and position are all analytic. The sensors are what a small
flight controller and receiver give: a 100 Hz IMU with bias and noise, 5 Hz GPS
with metre-level noise, a 25 Hz barometer with an offset.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

from aerial_kit.dynamics.rotations import rotmat_to_quat
from aerial_kit.estimation import InsConfig, InsEkf

G = 9.81
HW_BRIDGE = Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "hw_bridge"


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


def flight(duration=60.0, seed=0, accel_bias=(0.15, -0.1, 0.2), baro_offset=2.0,
           gps_dropout=None, gps_outlier_at=None):
    """Yield sensor events in time order: ("imu", t, f_body, q, a_world) / ("gps", t, p) / ("baro", t, z)."""
    rng = np.random.default_rng(seed)
    bias = np.array(accel_bias)
    events = []
    for k in range(int(duration * 100)):
        t = k * 0.01
        _, _, a, yaw = truth(t)
        R = attitude(a, yaw)
        f = R.T @ (a + np.array([0.0, 0.0, G])) + bias + rng.normal(0, 0.05, 3)
        events.append(("imu", t, f, rotmat_to_quat(R), a + rng.normal(0, 0.05, 3)))
        if k % 4 == 0:
            events.append(("baro", t, truth(t)[0][2] + baro_offset + rng.normal(0, 0.3)))
        if k % 20 == 0:
            if gps_dropout and gps_dropout[0] <= t < gps_dropout[1]:
                continue
            p = truth(t)[0] + rng.normal(0, [1.5, 1.5, 3.0])
            if gps_outlier_at is not None and abs(t - gps_outlier_at) < 1e-9:
                p = p + np.array([40.0, -30.0, 0.0])
            events.append(("gps", t, p))
    return events


def run(cfg, events, record_from=10.0):
    ekf = InsEkf(cfg)
    p0, v0, _, _ = truth(0.0)
    ekf.reset(p0 + np.array([1.0, -1.0, 0.5]), 0.0)
    errs_p, errs_v, sig_p, ts = [], [], [], []
    for ev in events:
        kind, t = ev[0], ev[1]
        if kind == "imu":
            accel = ev[4] if cfg.accel_mode == "world_linear" else ev[2]
            ekf.predict(t, accel, ev[3])
            if t >= record_from:
                p, v, _, _ = truth(t)
                errs_p.append(ekf.position - p)
                errs_v.append(ekf.velocity - v)
                sig_p.append(ekf.position_sigma())
                ts.append(t)
        elif kind == "gps":
            ekf.update_gps(ev[2])
        else:
            ekf.update_baro(ev[2])
    return ekf, np.array(errs_p), np.array(errs_v), np.array(sig_p), np.array(ts)


def rms(a, axis=0):
    return np.sqrt(np.mean(np.asarray(a) ** 2, axis=axis))


# ---------------------------------------------------------------------------


def test_with_imu_the_ekf_tracks_position_and_velocity_far_below_gps_noise():
    ekf, ep, ev, _, _ = run(InsConfig(accel_mode="body_specific_force"), flight())
    assert np.all(rms(ep)[:2] < 0.55)  # GPS alone is 1.5 m per axis
    assert rms(ep)[2] < 0.45  # baro + GPS; limited by resolving the baro offset
    assert np.all(rms(ev) < 0.2)


def test_the_accelerometer_bias_is_estimated():
    ekf, *_ = run(InsConfig(accel_mode="body_specific_force"), flight(duration=120.0))
    np.testing.assert_allclose(ekf.accel_bias, [0.15, -0.1, 0.2], atol=0.06)


def test_the_baro_offset_is_estimated_against_gps_altitude():
    ekf, *_ = run(InsConfig(accel_mode="body_specific_force"), flight(duration=120.0, baro_offset=2.0))
    assert ekf.baro_bias == pytest.approx(2.0, abs=0.5)


def test_world_linear_mode_matches_the_fake_flight_controllers_convention():
    _, ep, ev, _, _ = run(InsConfig(accel_mode="world_linear"), flight())
    assert np.all(rms(ep)[:2] < 0.55)
    assert np.all(rms(ev) < 0.2)


def test_without_an_accelerometer_it_still_beats_gps_differencing():
    """Betaflight over CRSF: attitude only. Constant-velocity model, compared
    with the per-fix GPS differencing it replaces in hw_bridge."""
    sys.path.insert(0, str(HW_BRIDGE))
    try:
        from hw_bridge.fusion import GpsVelocity
    finally:
        sys.path.remove(str(HW_BRIDGE))
    events = flight()
    _, ep, ev, _, _ = run(InsConfig(accel_mode="none"), events)
    assert np.all(rms(ep)[:2] < 1.2)  # GPS alone: 1.5
    gps_v = GpsVelocity()
    errs = []
    for e in events:
        if e[0] == "gps":
            vx, vy = gps_v.update(e[2][0], e[2][1], e[1])
            if e[1] >= 10.0:
                errs.append(np.array([vx, vy]) - truth(e[1])[1][:2])
    assert np.all(rms(ev)[:2] < 0.5 * rms(errs))


def test_reading_zeros_as_specific_force_is_why_none_is_the_default():
    assert InsConfig().accel_mode == "none"
    ekf = InsEkf(InsConfig(accel_mode="body_specific_force", gate=False))
    ekf.reset(np.zeros(3), 0.0)
    for k in range(1, 101):  # 1 s of an all-zero accelerometer field
        ekf.predict(k * 0.01, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    assert ekf.velocity[2] < -9.0  # it thinks it is falling


def test_ekf_climb_rate_beats_the_baro_complementary_filter():
    """The filter it replaces in hw_bridge differentiates the barometer alone."""
    sys.path.insert(0, str(HW_BRIDGE))
    try:
        from hw_bridge.fusion import BaroVerticalFilter
    finally:
        sys.path.remove(str(HW_BRIDGE))
    events = flight()
    _, _, ev, _, ts = run(InsConfig(accel_mode="body_specific_force"), events)
    baro = BaroVerticalFilter(hz=1.0)
    errs = []
    last_t = None
    for e in events:
        if e[0] == "baro":
            dt = 0.0 if last_t is None else e[1] - last_t
            last_t = e[1]
            _, vz = baro.update(e[2], dt)
            if e[1] >= 10.0:
                errs.append(vz - truth(e[1])[1][2])
    assert rms(ev[:, 2]) < 0.5 * rms(errs)


def test_a_gps_jump_is_rejected():
    events = flight(gps_outlier_at=30.0)
    ekf, ep, _, _, ts = run(InsConfig(accel_mode="body_specific_force"), events)
    assert ekf.rejected["gps"] >= 1
    assert np.abs(ep[(ts > 29.9) & (ts < 32.0)][:, :2]).max() < 2.0


def test_through_a_gps_dropout_it_coasts_and_its_uncertainty_grows():
    events = flight(gps_dropout=(30.0, 40.0))
    ekf, ep, _, sig, ts = run(InsConfig(accel_mode="body_specific_force"), events)
    during = (ts > 30.0) & (ts < 40.0)
    assert np.abs(ep[during][:, :2]).max() < 4.0
    assert sig[during][-1, 0] > 2.0 * sig[ts < 30.0][-1, 0]
    after = ts > 45.0
    assert np.all(rms(ep[after])[:2] < 0.8)


def test_the_position_sigma_is_honest():
    """Errors within 3 sigma nearly all the time: the filter is not overconfident."""
    _, ep, _, sig, _ = run(InsConfig(accel_mode="body_specific_force"), flight(seed=3))
    inside = np.abs(ep) < 3.0 * sig
    assert inside.mean() > 0.97


def test_first_gps_fix_initialises_an_unset_filter_and_out_of_order_predicts_are_ignored():
    ekf = InsEkf()
    assert ekf.update_gps(np.array([3.0, 4.0, 5.0])).accepted
    np.testing.assert_allclose(ekf.position, [3.0, 4.0, 5.0])
    ekf.predict(1.0)
    ekf.predict(0.5)  # earlier: ignored
    assert ekf.t == 1.0


@pytest.mark.parametrize("kwargs", [{"accel_mode": "ned"}, {"gps_sigma_xy": 0.0}, {"baro_bias_walk": -1.0}])
def test_bad_config_is_rejected(kwargs):
    with pytest.raises(ValueError):
        InsConfig(**kwargs)


def test_horizontal_only_gps_leaves_the_vertical_to_the_baro():
    ekf = InsEkf(InsConfig())
    ekf.reset(np.zeros(3), 0.0)
    sigma_z = ekf.position_sigma()[2]
    ekf.predict(0.1)
    ekf.update_gps(np.array([1.0, 2.0, 500.0]), use_altitude=False)  # bogus altitude ignored
    assert ekf.position[2] == pytest.approx(0.0, abs=1e-9)
    assert ekf.position_sigma()[2] >= sigma_z  # no false confidence in z
