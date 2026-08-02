"""Line-of-sight shortcut smoothing for grid paths."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

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


def shortcut_path(free: np.ndarray, path: Sequence[GridPoint]) -> List[GridPoint]:
    """Greedily remove waypoints when a later waypoint is visible in free space."""
    if len(path) <= 2:
        return list(path)

    result: List[GridPoint] = [path[0]]
    i = 0
    last = len(path) - 1
    while i < last:
        j = last
        while j > i + 1 and not segment_is_free(free, path[i], path[j]):
            j -= 1
        result.append(path[j])
        i = j
    return result
