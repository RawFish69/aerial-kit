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
