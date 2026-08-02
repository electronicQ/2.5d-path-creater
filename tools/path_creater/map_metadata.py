"""Load geodetic metadata associated with a static map."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from .geo import GeoPoint


@dataclass(frozen=True)
class MapMetadata:
    enu_origin: GeoPoint


def load_map_metadata(path: Path) -> MapMetadata:
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError("Map metadata not found: {}".format(path))
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    origin = data.get("enu_origin")
    if not isinstance(origin, dict):
        raise ValueError("Map metadata missing enu_origin.")
    return MapMetadata(
        enu_origin=GeoPoint(
            latitude=float(origin["latitude"]),
            longitude=float(origin["longitude"]),
            altitude=float(origin.get("altitude", 0.0)),
        )
    )
