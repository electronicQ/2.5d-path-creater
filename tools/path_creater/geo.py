"""WGS84 geodetic, ECEF, and local ENU conversions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


@dataclass(frozen=True)
class GeoPoint:
    latitude: float
    longitude: float
    altitude: float = 0.0


def _geodetic_to_ecef(point: GeoPoint) -> Tuple[float, float, float]:
    lat = math.radians(point.latitude)
    lon = math.radians(point.longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    x = (n + point.altitude) * cos_lat * math.cos(lon)
    y = (n + point.altitude) * cos_lat * math.sin(lon)
    z = (n * (1.0 - _E2) + point.altitude) * sin_lat
    return x, y, z


def _ecef_to_geodetic(x: float, y: float, z: float) -> GeoPoint:
    b = _A * (1.0 - _F)
    ep2 = (_A * _A - b * b) / (b * b)
    p = math.hypot(x, y)
    theta = math.atan2(z * _A, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(
        z + ep2 * b * math.sin(theta) ** 3,
        p - _E2 * _A * math.cos(theta) ** 3,
    )
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return GeoPoint(latitude=math.degrees(lat), longitude=math.degrees(lon), altitude=alt)


def geodetic_to_enu(point: GeoPoint, origin: GeoPoint) -> Tuple[float, float, float]:
    x, y, z = _geodetic_to_ecef(point)
    x0, y0, z0 = _geodetic_to_ecef(origin)
    lat0 = math.radians(origin.latitude)
    lon0 = math.radians(origin.longitude)
    dx, dy, dz = x - x0, y - y0, z - z0
    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = (
        -math.sin(lat0) * math.cos(lon0) * dx
        - math.sin(lat0) * math.sin(lon0) * dy
        + math.cos(lat0) * dz
    )
    up = (
        math.cos(lat0) * math.cos(lon0) * dx
        + math.cos(lat0) * math.sin(lon0) * dy
        + math.sin(lat0) * dz
    )
    return east, north, up


def enu_to_geodetic(x: float, y: float, z: float, origin: GeoPoint) -> GeoPoint:
    x0, y0, z0 = _geodetic_to_ecef(origin)
    lat0 = math.radians(origin.latitude)
    lon0 = math.radians(origin.longitude)
    dx = (
        -math.sin(lon0) * x
        - math.sin(lat0) * math.cos(lon0) * y
        + math.cos(lat0) * math.cos(lon0) * z
    )
    dy = (
        math.cos(lon0) * x
        - math.sin(lat0) * math.sin(lon0) * y
        + math.cos(lat0) * math.sin(lon0) * z
    )
    dz = math.cos(lat0) * y + math.sin(lat0) * z
    return _ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)
