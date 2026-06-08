"""Equirectangular GPS <-> local ENU conversion about a fixed origin.

Valid for the small operating areas these drones fly. Not valid over long
distances (no datum/ellipsoid correction beyond a cos(lat) scale).
"""

import math
from dataclasses import dataclass

_EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius


@dataclass(frozen=True)
class GeoOrigin:
    lat: float  # degrees
    lon: float  # degrees


def lla_to_enu(origin: GeoOrigin, lat: float, lon: float) -> tuple[float, float]:
    """Return (east_m, north_m) of (lat, lon) relative to origin."""
    lat0 = math.radians(origin.lat)
    dlat = math.radians(lat - origin.lat)
    dlon = math.radians(lon - origin.lon)
    north = dlat * _EARTH_RADIUS_M
    east = dlon * _EARTH_RADIUS_M * math.cos(lat0)
    return east, north


def enu_to_lla(origin: GeoOrigin, east: float, north: float) -> tuple[float, float]:
    """Inverse of lla_to_enu. Return (lat, lon) in degrees."""
    lat0 = math.radians(origin.lat)
    lat = origin.lat + math.degrees(north / _EARTH_RADIUS_M)
    lon = origin.lon + math.degrees(east / (_EARTH_RADIUS_M * math.cos(lat0)))
    return lat, lon
