"""Transactional, allow-listed updates to the path planner YAML."""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml

from .validation import validate_geo_point, validate_identifier, safe_basename


ALLOWED_SCALARS = {
    "unknown_gray": (int, 0, 255),
    "absolute_obstacle_gray": (int, 0, 255),
    "inflation_radius_m": (float, 0.0, 100.0),
    "d_safe_m": (float, 0.0, 1000.0),
    "smooth_paths": (bool, None, None),
    "write_overlay": (bool, None, None),
    "write_resolved_config": (bool, None, None),
}
ALLOWED_FLOATS = {
    "gray_cost.weight": (0.0, 1000.0),
    "curve_smoothing.sample_interval_m": (0.01, 100.0),
    "curve_smoothing.corner_cut_fraction": (0.0, 1.0),
    "curve_smoothing.max_corner_cut_m": (0.0, 1000.0),
    "curve_smoothing.min_corner_cut_m": (0.0, 1000.0),
    "curve_smoothing.safety_check_step_m": (0.001, 100.0),
}
ALLOWED_BOOL = {"gray_cost.enabled", "curve_smoothing.enabled"}
PLANNER_WEIGHTS = {"w_turn", "w_clear", "w_gray"}


def _set_path(config: Dict[str, Any], path: str, value: Any) -> None:
    pieces = path.split(".")
    node = config
    for piece in pieces[:-1]:
        if not isinstance(node.get(piece), dict):
            node[piece] = {}
        node = node[piece]
    node[pieces[-1]] = value


def _check_number(value: Any, low: float, high: float, path: str, integer: bool = False) -> Any:
    if isinstance(value, bool):
        raise ValueError("{} must be numeric".format(path))
    try:
        number = int(value) if integer else float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("{} must be numeric".format(path)) from exc
    if number < low or number > high:
        raise ValueError("{} is out of range".format(path))
    return number


def apply_overrides(config: Mapping[str, Any], request: Mapping[str, Any], pgm_name: str, output_dir: str) -> Dict[str, Any]:
    result = copy.deepcopy(dict(config))
    pgm_name = safe_basename(pgm_name, ".pgm") if str(pgm_name).lower().endswith(".pgm") else safe_basename(pgm_name)
    if pgm_name.lower().endswith(".pgm"):
        pgm_name = pgm_name[:-4]
    result["map_yaml"] = "../../../map/pgm/{}.yaml".format(pgm_name)
    result["map_meta"] = "map_enu.meta.yaml"
    result["output_dir"] = output_dir
    result["start"] = validate_geo_point(request.get("start"), "start")
    result["goal"] = validate_geo_point(request.get("goal"), "goal")

    overrides = request.get("config_overrides") or {}
    if not isinstance(overrides, Mapping):
        raise ValueError("config_overrides must be an object")
    allowed = set(ALLOWED_SCALARS) | set(ALLOWED_FLOATS) | ALLOWED_BOOL | {"planners"}
    unknown = set(overrides) - allowed
    if unknown:
        raise ValueError("unsupported config override: {}".format(sorted(unknown)[0]))
    for key, (kind, low, high) in ALLOWED_SCALARS.items():
        if key not in overrides:
            continue
        value = overrides[key]
        if kind is bool:
            if not isinstance(value, bool):
                raise ValueError("{} must be boolean".format(key))
        else:
            value = _check_number(value, low, high, key, integer=kind is int)
        result[key] = value
    for key, (low, high) in ALLOWED_FLOATS.items():
        if key in overrides:
            _set_path(result, key, _check_number(overrides[key], low, high, key))
    for key in ALLOWED_BOOL:
        if key in overrides:
            if not isinstance(overrides[key], bool):
                raise ValueError("{} must be boolean".format(key))
            _set_path(result, key, overrides[key])
    planners = overrides.get("planners")
    if planners is not None:
        if not isinstance(planners, Mapping):
            raise ValueError("planners must be an object")
        for mode, values in planners.items():
            if mode not in ("distance", "smooth", "cost_aware") or not isinstance(values, Mapping):
                raise ValueError("invalid planner override")
            for weight in values:
                if weight not in PLANNER_WEIGHTS:
                    raise ValueError("invalid planner weight")
                _set_path(result, "planners.{}.{}".format(mode, weight), _check_number(values[weight], 0.0, 100000.0, weight))
    return result


class ConfigStore:
    def __init__(self, path: Path) -> None:
        self.path = Path(path).resolve()

    def load(self) -> Dict[str, Any]:
        with self.path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}

    def update(self, request: Mapping[str, Any], pgm_name: str, output_dir: str, backup_path: Optional[Path] = None) -> Dict[str, Any]:
        current = self.load()
        updated = apply_overrides(current, request, pgm_name, output_dir)
        temp = self.path.with_name(self.path.name + ".tmp")
        backup = backup_path or self.path.with_name(self.path.name + ".previous")
        with temp.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(updated, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        if self.path.exists():
            os.replace(self.path, backup)
        os.replace(temp, self.path)
        return updated
