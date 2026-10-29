"""Entry point: load map, plan three cost modes, write trajectories."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml

from .auto_endpoints import resolve_endpoints
from .clearance import build_clearance_from_free, build_corridor_distance, expand_cells
from .curve_smoothing import (
    CurveSmoothingConfig,
    TrackingSamplingConfig,
    ensure_path_endpoints,
    smooth_and_adaptive_sample_path,
)
from .geo import GeoPoint, enu_to_geodetic, geodetic_to_enu, geodetic_to_local_utm
from .map_loader import load_map
from .map_metadata import load_map_metadata
from .map_semantics import MapSemantics, build_semantic_layers
from .path_smoothing import shortcut_path, turning_points
from .planner import plan_mode
from .trajectory import remove_legacy_outputs, write_overlay, write_road_map_xml
from .timing import format_duration


# Stable filenames used by downstream vehicle software.  The numbering is
# part of the external interface and intentionally differs from planner mode
# iteration order: 1=cost-aware, 2=distance, 3=smooth.
ROAD_MAP_FILENAMES = {
    "cost_aware": "road_map1.xml",
    "distance": "road_map2.xml",
    "smooth": "road_map3.xml",
}


def default_config_path() -> Path:
    return Path(__file__).resolve().parent / "config" / "config.yaml"


def _load_config(config_path: Path) -> Dict[str, Any]:
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return cfg


def _resolve_map_yaml(config_path: Path, cfg: Dict[str, Any]) -> Path:
    rel = cfg.get("map_yaml", "PCD/map.yaml")
    p = Path(rel)
    if not p.is_absolute():
        p = (config_path.parent / p).resolve()
    return p


def _resolve_config_path(config_path: Path, path_value: str) -> Path:
    p = Path(path_value)
    if not p.is_absolute():
        p = (config_path.parent / p).resolve()
    return p


def _load_geo_point(cfg: Dict[str, Any], key: str) -> GeoPoint:
    value = cfg.get(key)
    if not isinstance(value, dict):
        raise ValueError("{} must contain latitude and longitude.".format(key))
    return GeoPoint(
        latitude=float(value["latitude"]),
        longitude=float(value["longitude"]),
        altitude=float(value.get("altitude", 0.0)),
    )


def _load_curve_smoothing_config(cfg: Dict[str, Any]) -> CurveSmoothingConfig:
    value = cfg.get("curve_smoothing", {})
    if isinstance(value, bool):
        return CurveSmoothingConfig(enabled=value)
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError("curve_smoothing must be a mapping or boolean.")
    return CurveSmoothingConfig(
        enabled=bool(value.get("enabled", True)),
        sample_interval_m=float(value.get("sample_interval_m", 1.0)),
        corner_cut_fraction=float(value.get("corner_cut_fraction", 0.35)),
        max_corner_cut_m=float(value.get("max_corner_cut_m", 8.0)),
        min_corner_cut_m=float(value.get("min_corner_cut_m", 0.2)),
        safety_check_step_m=float(value.get("safety_check_step_m", 0.25)),
        max_corner_attempts=int(value.get("max_corner_attempts", 8)),
    )


def _load_tracking_sampling_config(cfg: Dict[str, Any]) -> TrackingSamplingConfig:
    value = cfg.get("tracking_sampling", {})
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError("tracking_sampling must be a mapping.")
    return TrackingSamplingConfig(
        enabled=bool(value.get("enabled", True)),
        max_spacing_m=float(value.get("max_spacing_m", 10.0)),
        min_spacing_m=float(value.get("min_spacing_m", 0.3)),
        chord_tolerance_m=float(value.get("chord_tolerance_m", 0.15)),
        max_heading_change_deg=float(value.get("max_heading_change_deg", 3.0)),
    )


def _load_corridor_centering_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    value = cfg.get("corridor_centering", {})
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError("corridor_centering must be a mapping.")
    config = {
        "enabled": bool(value.get("enabled", False)),
        "apply_to": str(value.get("apply_to", "cost_aware")),
        "corner_only": bool(value.get("corner_only", True)),
        "corner_radius_m": float(value.get("corner_radius_m", 8.0)),
        "corner_turn_threshold_deg": float(value.get("corner_turn_threshold_deg", 20.0)),
        "high_cost_gray_threshold": int(value.get("high_cost_gray_threshold", 220)),
        "target_clearance_m": float(value.get("target_clearance_m", 3.0)),
        "weight": float(value.get("weight", 20.0)),
        "shortcut_min_clearance_m": float(value.get("shortcut_min_clearance_m", 2.5)),
        "curve_min_clearance_m": float(value.get("curve_min_clearance_m", 2.0)),
    }
    if not 0 <= config["high_cost_gray_threshold"] <= 255:
        raise ValueError("high_cost_gray_threshold must be in [0, 255].")
    if config["target_clearance_m"] <= 0.0 or config["weight"] <= 0.0:
        raise ValueError("corridor target and weight must be positive.")
    if config["corner_radius_m"] < 0.0:
        raise ValueError("corner_radius_m must be non-negative.")
    if not 0.0 < config["corner_turn_threshold_deg"] <= 180.0:
        raise ValueError("corner_turn_threshold_deg must be in (0, 180].")
    if config["shortcut_min_clearance_m"] < 0.0 or config["curve_min_clearance_m"] < 0.0:
        raise ValueError("corridor minimum clearances must be non-negative.")
    return config


def _world_segment_is_free(omap, free, safety_step_m: float, a, b) -> bool:
    ax, ay = a
    bx, by = b
    length = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
    steps = max(1, int(length / max(float(safety_step_m), 1e-6)) + 1)
    for i in range(steps + 1):
        t = i / float(steps)
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        r, c = omap.world_to_grid(x, y)
        if not omap.in_bounds(r, c) or not free[r, c]:
            return False
    return True


def _world_segment_meets_minimum(
    omap,
    values: np.ndarray,
    minimum: float,
    safety_step_m: float,
    a,
    b,
    required_mask: Optional[np.ndarray] = None,
) -> bool:
    ax, ay = a
    bx, by = b
    length = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
    steps = max(1, int(length / max(float(safety_step_m), 1e-6)) + 1)
    for i in range(steps + 1):
        t = i / float(steps)
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        r, c = omap.world_to_grid(x, y)
        if not omap.in_bounds(r, c):
            return False
        if required_mask is None or required_mask[r, c]:
            if float(values[r, c]) < float(minimum):
                return False
    return True


def main(argv: Optional[list] = None) -> int:
    total_start = time.perf_counter()
    parser = argparse.ArgumentParser(description="Three-cost global planner on pgm/yaml maps.")
    parser.add_argument(
        "--config",
        type=str,
        default=str(default_config_path()),
        help="Path to config.yaml",
    )
    args = parser.parse_args(argv)

    config_path = Path(args.config).resolve()
    if not config_path.is_file():
        print("Config not found: {}".format(config_path), file=sys.stderr)
        return 1

    cfg = _load_config(config_path)
    root = config_path.parent
    map_yaml = _resolve_map_yaml(config_path, cfg)
    if "map_meta" not in cfg:
        raise ValueError("Config missing required map_meta path.")
    map_meta = load_map_metadata(_resolve_config_path(config_path, str(cfg["map_meta"])))

    print("Loading map: {}".format(map_yaml))
    omap = load_map(map_yaml, occupied_thresh_override=None)

    inflation = float(cfg.get("inflation_radius_m", 0.6))
    d_safe = float(cfg.get("d_safe_m", 1.0))
    gray_cfg = cfg.get("gray_cost") or {}
    semantics = MapSemantics(
        unknown_gray=int(cfg.get("unknown_gray", 205)),
        absolute_obstacle_gray=int(cfg.get("absolute_obstacle_gray", 90)),
        gray_cost_weight=float(gray_cfg.get("weight", cfg.get("gray_cost_weight", 1.0)))
        if bool(gray_cfg.get("enabled", True))
        else 0.0,
    )
    print(
        "Map {}x{}, res={:.3f} m, unknown_gray={}, absolute_obstacle_gray={}, inflation={:.2f} m".format(
            omap.width,
            omap.height,
            omap.resolution,
            semantics.unknown_gray,
            semantics.absolute_obstacle_gray,
            inflation,
        )
    )

    semantic_layers = build_semantic_layers(omap.gray, semantics)
    clearance = build_clearance_from_free(
        semantic_layers.free,
        omap.resolution,
        inflation_radius_m=inflation,
        occupied_thresh=0.0,
    )
    free_count = int(clearance.free.sum())
    print("Free cells after inflation: {}".format(free_count))
    corridor_cfg = _load_corridor_centering_config(cfg)
    corridor_distance = None
    if corridor_cfg["enabled"]:
        corridor_distance = build_corridor_distance(
            omap.gray,
            omap.resolution,
            corridor_cfg["high_cost_gray_threshold"],
            hard_boundary=semantic_layers.forbidden,
        )
        print(
            "Corridor centering: mode={}, edge_gray<={}, target={:.2f} m".format(
                corridor_cfg["apply_to"],
                corridor_cfg["high_cost_gray_threshold"],
                corridor_cfg["target_clearance_m"],
            )
        )

    start_geo = _load_geo_point(cfg, "start")
    goal_geo = _load_geo_point(cfg, "goal")
    sx, sy, _ = geodetic_to_enu(start_geo, map_meta.enu_origin)
    gx, gy, _ = geodetic_to_enu(goal_geo, map_meta.enu_origin)
    start_rc, goal_rc, start_xy, goal_xy, auto_picked = resolve_endpoints(
        omap,
        clearance,
        [sx, sy],
        [gx, gy],
        seed=int(cfg.get("seed", 0)),
    )
    print(
        "Start: grid={} world=({:.3f}, {:.3f}){}".format(
            start_rc, start_xy[0], start_xy[1], " [auto]" if auto_picked else ""
        )
    )
    print(
        "Goal:  grid={} world=({:.3f}, {:.3f}){}".format(
            goal_rc, goal_xy[0], goal_xy[1], " [auto]" if auto_picked else ""
        )
    )

    out_dir = Path(cfg.get("output_dir", "output"))
    if not out_dir.is_absolute():
        out_dir = (root / out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    planners_cfg = cfg.get("planners") or {
        "distance": {"w_turn": 0.0, "w_clear": 1.0, "w_gray": 0.5},
        "smooth": {"w_turn": 4.0, "w_clear": 2.0, "w_gray": 1.0},
        "cost_aware": {"w_turn": 0.5, "w_clear": 12.0, "w_gray": 35.0},
    }

    paths = {}
    smooth_paths = bool(cfg.get("smooth_paths", True))
    curve_cfg = _load_curve_smoothing_config(cfg)
    sampling_cfg = _load_tracking_sampling_config(cfg)
    base_world_segment_is_free = lambda a, b: _world_segment_is_free(
        omap,
        clearance.free,
        curve_cfg.safety_check_step_m,
        a,
        b,
    )
    for mode, weights in planners_cfg.items():
        print("Planning mode: {} ...".format(mode))
        mode_start = time.perf_counter()

        plan_start = time.perf_counter()
        use_corridor = corridor_distance is not None and mode == corridor_cfg["apply_to"]
        center_weight_map = None
        if use_corridor:
            base_path = plan_mode(
                clearance,
                semantic_layers.gray_cost,
                start_rc,
                goal_rc,
                weights,
                d_safe_m=d_safe,
            )
            if corridor_cfg["corner_only"]:
                guide_path = shortcut_path(clearance.free, base_path)
                turns = turning_points(guide_path, corridor_cfg["corner_turn_threshold_deg"])
                corner_seed = np.zeros(clearance.free.shape, dtype=bool)
                for row, col in turns:
                    corner_seed[row, col] = True
                center_weight_map = expand_cells(
                    corner_seed,
                    omap.resolution,
                    corridor_cfg["corner_radius_m"],
                )
                print("Corner-centered zones: {} turns".format(len(turns)))
            if center_weight_map is not None and not center_weight_map.any():
                path = base_path
            else:
                mode_weights = dict(weights)
                mode_weights["w_center"] = corridor_cfg["weight"]
                path = plan_mode(
                    clearance,
                    semantic_layers.gray_cost,
                    start_rc,
                    goal_rc,
                    mode_weights,
                    d_safe_m=d_safe,
                    corridor_distance_m=corridor_distance,
                    corridor_target_m=corridor_cfg["target_clearance_m"],
                    corridor_weight_map=center_weight_map,
                )
        else:
            path = plan_mode(clearance, semantic_layers.gray_cost, start_rc, goal_rc, weights, d_safe_m=d_safe)
        center_active = use_corridor and (
            center_weight_map is None or bool(center_weight_map.any())
        )
        plan_elapsed = time.perf_counter() - plan_start
        raw_len = len(path)

        shortcut_elapsed = 0.0
        if smooth_paths:
            shortcut_start = time.perf_counter()
            path = shortcut_path(
                clearance.free,
                path,
                clearance_values=corridor_distance if center_active else None,
                minimum_clearance=corridor_cfg["shortcut_min_clearance_m"] if center_active else 0.0,
                required_mask=center_weight_map if center_active else None,
            )
            shortcut_elapsed = time.perf_counter() - shortcut_start
        shortcut_len = len(path)
        control_points = [omap.grid_to_world(r, c) for r, c in path]

        curve_start = time.perf_counter()
        if use_corridor:
            is_world_segment_valid = lambda a, b: (
                base_world_segment_is_free(a, b)
                and _world_segment_meets_minimum(
                    omap,
                    corridor_distance,
                    corridor_cfg["curve_min_clearance_m"],
                    curve_cfg.safety_check_step_m,
                    a,
                    b,
                    required_mask=center_weight_map,
                )
            )
        else:
            is_world_segment_valid = base_world_segment_is_free
        enu_points = smooth_and_adaptive_sample_path(
            control_points,
            is_world_segment_valid,
            curve_cfg,
            sampling_cfg,
        )
        enu_points = ensure_path_endpoints(enu_points, control_points[0], control_points[-1])
        curve_elapsed = time.perf_counter() - curve_start

        write_start = time.perf_counter()
        paths[mode] = [omap.world_to_grid(x, y) for x, y in enu_points]
        geo_points = [enu_to_geodetic(x, y, 0.0, map_meta.enu_origin) for x, y in enu_points]
        local_utm_points = [
            geodetic_to_local_utm(point, map_meta.enu_origin)[:2]
            for point in geo_points
        ]
        try:
            road_map_name = ROAD_MAP_FILENAMES[mode]
        except KeyError as exc:
            raise ValueError("unsupported planner mode for road map output: {}".format(mode)) from exc
        road_map_path = out_dir / road_map_name
        write_road_map_xml(road_map_path, local_utm_points)
        write_elapsed = time.perf_counter() - write_start
        mode_elapsed = time.perf_counter() - mode_start

        print(
            "  {} raw waypoints -> {} shortcut waypoints -> {} adaptive tracking points -> {}".format(
                raw_len, shortcut_len, len(enu_points), road_map_path
            )
        )
        print(
            "  timing: astar={}, shortcut={}, curve={}, write={}, total={}".format(
                format_duration(plan_elapsed),
                format_duration(shortcut_elapsed),
                format_duration(curve_elapsed),
                format_duration(write_elapsed),
                format_duration(mode_elapsed),
            )
        )

    if cfg.get("write_overlay", True):
        overlay_path = out_dir / "overlay.png"
        write_overlay(omap, clearance.free, paths, overlay_path, start_rc, goal_rc)
        print("Wrote {}".format(overlay_path))

    remove_legacy_outputs(out_dir)
    print("Done. Total elapsed: {}".format(format_duration(time.perf_counter() - total_start)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
