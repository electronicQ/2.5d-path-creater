"""Obstacle inflation and clearance (distance-to-obstacle) field."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy import ndimage

from .map_loader import OccupancyMap


@dataclass
class ClearanceMap:
    """Inflated free space and distance-to-obstacle in meters."""

    # True = traversable after inflation
    free: np.ndarray
    # Distance to nearest hard obstacle (meters)
    dist_m: np.ndarray
    inflation_radius_m: float
    occupied_thresh: float


def _binary_obstacles(occ: np.ndarray, occupied_thresh: float) -> np.ndarray:
    """Cells at or above occupied_thresh are hard obstacles."""
    return occ >= occupied_thresh


def build_clearance(
    omap: OccupancyMap,
    inflation_radius_m: float,
    occupied_thresh: Optional[float] = None,
) -> ClearanceMap:
    thresh = float(omap.occupied_thresh if occupied_thresh is None else occupied_thresh)
    hard = _binary_obstacles(omap.occ, thresh)

    # EDT in cell units from nearest obstacle (False cells are free space input)
    # distance_transform_edt computes distance to nearest False / zero
    dist_cells = ndimage.distance_transform_edt(~hard).astype(np.float32)
    dist_m = dist_cells * float(omap.resolution)

    inflate_cells = float(inflation_radius_m) / float(omap.resolution)
    free = dist_cells > inflate_cells

    return ClearanceMap(
        free=free,
        dist_m=dist_m,
        inflation_radius_m=float(inflation_radius_m),
        occupied_thresh=thresh,
    )


def build_clearance_from_free(
    free: np.ndarray,
    resolution: float,
    inflation_radius_m: float,
    occupied_thresh: float = 0.0,
) -> ClearanceMap:
    hard = ~np.asarray(free, dtype=bool)
    dist_cells = ndimage.distance_transform_edt(~hard).astype(np.float32)
    dist_m = dist_cells * float(resolution)
    inflate_cells = float(inflation_radius_m) / float(resolution)
    inflated_free = dist_cells > inflate_cells
    return ClearanceMap(
        free=inflated_free,
        dist_m=dist_m,
        inflation_radius_m=float(inflation_radius_m),
        occupied_thresh=float(occupied_thresh),
    )


def build_corridor_distance(
    gray: np.ndarray,
    resolution: float,
    high_cost_gray_threshold: float,
    hard_boundary: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Return distance in meters to the nearest high-cost road boundary."""
    boundary = np.asarray(gray, dtype=np.uint8) <= int(high_cost_gray_threshold)
    if hard_boundary is not None:
        boundary = boundary | np.asarray(hard_boundary, dtype=bool)
    distance_cells = ndimage.distance_transform_edt(~boundary).astype(np.float32)
    return distance_cells * float(resolution)
