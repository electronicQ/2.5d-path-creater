"""Typed request and result models used by the ground station UI."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class GeoPoint:
    latitude: float
    longitude: float
    altitude: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        if not -90 <= self.latitude <= 90 or not -180 <= self.longitude <= 180:
            raise ValueError("latitude or longitude is out of range")
        return {"latitude": float(self.latitude), "longitude": float(self.longitude), "altitude": float(self.altitude)}


@dataclass
class PlannerRequest:
    map_id: str
    pgm_name: str
    start: GeoPoint
    goal: GeoPoint
    overrides: Dict[str, Any] = field(default_factory=dict)

    def to_message(self, job_id: str) -> Dict[str, Any]:
        if not self.map_id or "/" in self.map_id or "\\" in self.map_id:
            raise ValueError("invalid map_id")
        if not self.pgm_name or "/" in self.pgm_name or "\\" in self.pgm_name:
            raise ValueError("invalid pgm_name")
        return {
            "type": "plan.start",
            "job_id": job_id,
            "map_id": self.map_id,
            "pgm_name": self.pgm_name,
            "start": self.start.to_dict(),
            "goal": self.goal.to_dict(),
            "config_overrides": dict(self.overrides),
        }
