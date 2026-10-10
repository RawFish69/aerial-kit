from hw_bridge.fusion import AltitudeFilter


def test_initial_altitude_tracks_first_baro():
    f = AltitudeFilter(alpha=0.98)
    z = f.update(baro_alt=5.0, vz=0.0, dt=0.0, gps_alt=None)
    assert abs(z - 5.0) < 1e-6


def test_converges_to_steady_baro():
    f = AltitudeFilter(alpha=0.9)
    f.update(baro_alt=0.0, vz=0.0, dt=0.0, gps_alt=None)
    z = 0.0
    for _ in range(200):
        z = f.update(baro_alt=10.0, vz=0.0, dt=0.05, gps_alt=None)
    assert abs(z - 10.0) < 0.1


def test_integrates_velocity_between_baro_updates():
    f = AltitudeFilter(alpha=1.0)  # ignore baro correction, pure integration
    f.update(baro_alt=0.0, vz=0.0, dt=0.0, gps_alt=None)
    z = f.update(baro_alt=0.0, vz=2.0, dt=0.5, gps_alt=None)
    assert abs(z - 1.0) < 1e-6  # 2 m/s * 0.5 s


# ---------------------------------------------------------------------------
# BaroVerticalFilter: altitude *and* climb rate from the barometer alone
# ---------------------------------------------------------------------------

import math  # noqa: E402

import pytest  # noqa: E402

from hw_bridge.fusion import BaroVerticalFilter  # noqa: E402


def test_vertical_filter_starts_at_the_first_sample_at_rest():
    f = BaroVerticalFilter(hz=1.0)
    assert f.update(baro_alt=3.0, dt=0.0) == (3.0, 0.0)


def test_vertical_filter_recovers_a_steady_climb_rate():
    f = BaroVerticalFilter(hz=1.0)
    dt = 1.0 / 30.0
    z = vz = 0.0
    for k in range(300):  # 10 s climbing at 0.8 m/s
        z, vz = f.update(baro_alt=0.8 * k * dt, dt=dt)
    assert vz == pytest.approx(0.8, abs=0.01)
    assert z == pytest.approx(0.8 * 299 * dt, abs=0.02)  # no steady lag on a ramp


def test_vertical_filter_settles_on_a_step_without_drift():
    f = BaroVerticalFilter(hz=1.0)
    f.update(baro_alt=0.0, dt=0.0)
    for _ in range(300):
        z, vz = f.update(baro_alt=2.0, dt=1.0 / 30.0)
    assert z == pytest.approx(2.0, abs=1e-3)
    assert vz == pytest.approx(0.0, abs=1e-3)


def test_vertical_filter_smooths_baro_noise():
    import random

    rng = random.Random(0)
    f = BaroVerticalFilter(hz=1.0)
    vzs = []
    for k in range(600):
        _, vz = f.update(baro_alt=rng.gauss(0.0, 0.2), dt=1.0 / 30.0)
        vzs.append(vz)
    rms = math.sqrt(sum(v * v for v in vzs[100:]) / len(vzs[100:]))
    assert rms < 0.5  # 0.2 m of baro noise differentiated raw at 30 Hz would be ~8 m/s


def test_vertical_filter_rejects_bad_bandwidth():
    with pytest.raises(ValueError):
        BaroVerticalFilter(hz=0.0)


# ---------------------------------------------------------------------------
# GpsVelocity: differenced per fix, not per estimator tick
# ---------------------------------------------------------------------------

from hw_bridge.fusion import GpsVelocity  # noqa: E402


def test_gps_velocity_ignores_repeated_fixes():
    """A 10 Hz GPS read on a 30 Hz tick: two of every three ticks see the same
    fix. Per-tick differencing read 0, 0, 3x; this reads the true speed."""
    g = GpsVelocity(smoothing=0.0)
    out = []
    for tick in range(90):  # 3 s at 30 Hz, moving east at 2 m/s
        t_fix = (tick // 3) * 0.1  # the fix only changes every third tick
        out.append(g.update(east=2.0 * t_fix, north=0.0, t=t_fix))
    vxs = [vx for vx, _ in out[6:]]
    assert min(vxs) == pytest.approx(2.0) and max(vxs) == pytest.approx(2.0)


def test_gps_velocity_resets_after_a_dropout():
    g = GpsVelocity(smoothing=0.0, max_gap_s=1.0)
    g.update(0.0, 0.0, 0.0)
    assert g.update(1.0, 0.0, 0.5) == (pytest.approx(2.0), 0.0)
    assert g.update(50.0, 0.0, 5.0) == (0.0, 0.0)  # 4.5 s gap


def test_gps_velocity_rejects_bad_smoothing():
    with pytest.raises(ValueError):
        GpsVelocity(smoothing=1.0)
