"""Line-of-sight shortcut smoothing for grid paths."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


GridPoint = Tuple[int, int]


def _candidate_indices(value: float, upper_bound: int) -> Iterable[int]:
    low = int(math.floor(value))
    high = int(math.ceil(value))
    for index in (low, high):
        if 0 <= index < upper_bound:
            yield index


def segment_is_free(free: np.ndarray, start: GridPoint, end: GridPoint) -> bool:
    """Return true when the center-to-center segment stays inside free cells.

    The check samples denser than one grid cell and tests floor/ceil neighbors
    around the sampled point. This is conservative enough to reject diagonal
    corner cuts through blocked cells.
    """
    free_mask = np.asarray(free, dtype=bool)
    h, w = free_mask.shape
    r0, c0 = start
    r1, c1 = end
    if not (0 <= r0 < h and 0 <= c0 < w and 0 <= r1 < h and 0 <= c1 < w):
        return False
    if not free_mask[r0, c0] or not free_mask[r1, c1]:
        return False

    dr = r1 - r0
    dc = c1 - c0
    steps = max(abs(dr), abs(dc)) * 4 + 1
    if steps <= 1:
        return True

    for i in range(steps + 1):
        t = i / float(steps)
        r = r0 + dr * t
        c = c0 + dc * t
        for rr in _candidate_indices(r, h):
            for cc in _candidate_indices(c, w):
                if not free_mask[rr, cc]:
                    return False
    return True


def segment_meets_minimum(
    values: np.ndarray,
    start: GridPoint,
    end: GridPoint,
    minimum: float,
    required_mask: Optional[np.ndarray] = None,
) -> bool:
    value_map = np.asarray(values, dtype=np.float32)
    h, w = value_map.shape
    r0, c0 = start
    r1, c1 = end
    if not (0 <= r0 < h and 0 <= c0 < w and 0 <= r1 < h and 0 <= c1 < w):
        return False
    dr = r1 - r0
    dc = c1 - c0
    active_mask = None if required_mask is None else np.asarray(required_mask, dtype=bool)
    if active_mask is not None and active_mask.shape != value_map.shape:
        raise ValueError("required_mask must match values shape")
    steps = max(abs(dr), abs(dc)) * 4 + 1
    for i in range(steps + 1):
        t = i / float(max(steps, 1))
        r = r0 + dr * t
        c = c0 + dc * t
        for rr in _candidate_indices(r, h):
            for cc in _candidate_indices(c, w):
                if (active_mask is None or active_mask[rr, cc]) and float(value_map[rr, cc]) < float(minimum):
                    return False
    return True


def shortcut_path(
    free: np.ndarray,
    path: Sequence[GridPoint],
    clearance_values: Optional[np.ndarray] = None,
    minimum_clearance: float = 0.0,
    required_mask: Optional[np.ndarray] = None,
) -> List[GridPoint]:
    """Greedily remove waypoints when a later waypoint is visible in free space."""
    if len(path) <= 2:
        return list(path)

    result: List[GridPoint] = [path[0]]
    i = 0
    last = len(path) - 1
    while i < last:
        j = last
        while j > i + 1:
            valid = segment_is_free(free, path[i], path[j])
            if valid and clearance_values is not None:
                valid = segment_meets_minimum(
                    clearance_values,
                    path[i],
                    path[j],
                    minimum_clearance,
                    required_mask=required_mask,
                )
            if valid:
                break
            j -= 1
        result.append(path[j])
        i = j
    return result


def turning_points(path: Sequence[GridPoint], minimum_angle_deg: float = 20.0) -> List[GridPoint]:
    """Return simplified-path vertices whose direction change exceeds a threshold."""
    if len(path) < 3:
        return []
    threshold = math.radians(float(minimum_angle_deg))
    turns: List[GridPoint] = []
    for index in range(1, len(path) - 1):
        prev_r, prev_c = path[index - 1]
        row, col = path[index]
        next_r, next_c = path[index + 1]
        first = (row - prev_r, col - prev_c)
        second = (next_r - row, next_c - col)
        first_len = math.hypot(*first)
        second_len = math.hypot(*second)
        if first_len <= 0.0 or second_len <= 0.0:
            continue
        cosine = (
            first[0] * second[0] + first[1] * second[1]
        ) / (first_len * second_len)
        angle = math.acos(min(1.0, max(-1.0, cosine)))
        if angle >= threshold:
            turns.append((row, col))
    return turns
