import math

from hw_bridge.geo import GeoOrigin, enu_to_lla, lla_to_enu


def test_origin_maps_to_zero():
    o = GeoOrigin(lat=37.0, lon=-122.0)
    e, n = lla_to_enu(o, 37.0, -122.0)
    assert abs(e) < 1e-6 and abs(n) < 1e-6


def test_north_offset_positive_north():
    o = GeoOrigin(lat=37.0, lon=-122.0)
    # ~0.001 deg latitude north ≈ 111 m
    e, n = lla_to_enu(o, 37.001, -122.0)
    assert abs(e) < 1.0
    assert 100.0 < n < 120.0


def test_east_offset_positive_east():
    o = GeoOrigin(lat=37.0, lon=-122.0)
    e, n = lla_to_enu(o, 37.0, -121.999)
    assert n == 0.0 or abs(n) < 1.0
    assert 80.0 < e < 100.0  # cos(37deg)*111m ≈ 88 m


def test_round_trip():
    o = GeoOrigin(lat=37.0, lon=-122.0)
    e0, n0 = 53.0, -27.5
    lat, lon = enu_to_lla(o, e0, n0)
    e1, n1 = lla_to_enu(o, lat, lon)
    assert abs(e1 - e0) < 0.05 and abs(n1 - n0) < 0.05
