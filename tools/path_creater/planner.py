"""Eight-connected A* with distance / smooth / balanced cost modes."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .clearance import ClearanceMap


GridPoint = Tuple[int, int]

_NEIGHBORS = (
    (-1, 0, 1.0),
    (-1, 1, math.sqrt(2.0)),
    (0, 1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, 0, 1.0),
    (1, -1, math.sqrt(2.0)),
    (0, -1, 1.0),
    (-1, -1, math.sqrt(2.0)),
)


@dataclass
class PlannerWeights:
    w_turn: float
    w_clear: float
    w_gray: float


def _dir_id(dr: int, dc: int) -> int:
    # Map (dr,dc) to 0..7 matching _NEIGHBORS order
    table = {
        (-1, 0): 0,
        (-1, 1): 1,
        (0, 1): 2,
        (1, 1): 3,
        (1, 0): 4,
        (1, -1): 5,
        (0, -1): 6,
        (-1, -1): 7,
    }
    return table[(dr, dc)]


def clearance_penalty(dist_m: float, d_safe_m: float) -> float:
    return max(0.0, d_safe_m - float(dist_m))


def turn_penalty(prev_dir: Optional[int], new_dir: int) -> float:
    if prev_dir is None:
        return 0.0
    diff = abs(prev_dir - new_dir) % 8
    if diff > 4:
        diff = 8 - diff
    return float(diff)


def astar(
    free: np.ndarray,
    dist_m: np.ndarray,
    gray_cost: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    weights: PlannerWeights,
    d_safe_m: float,
) -> List[GridPoint]:
    h, w = free.shape
    sr, sc = start
    gr, gc = goal
    if not free[sr, sc] or not free[gr, gc]:
        raise ValueError("Start or goal not free.")

    g_score = np.full((h, w), np.inf, dtype=np.float64)
    parent_r = np.full((h, w), -1, dtype=np.int32)
    parent_c = np.full((h, w), -1, dtype=np.int32)
    came_dir = np.full((h, w), -1, dtype=np.int8)

    g_score[sr, sc] = 0.0

    def heuristic(r: int, c: int) -> float:
        return math.hypot(r - gr, c - gc)

    open_heap: List[Tuple[float, int, int, int]] = []
    counter = 0
    heapq.heappush(open_heap, (heuristic(sr, sc), counter, sr, sc))
    counter += 1
    closed = np.zeros((h, w), dtype=bool)

    while open_heap:
        _, _, r, c = heapq.heappop(open_heap)
        if closed[r, c]:
            continue
        closed[r, c] = True
        if r == gr and c == gc:
            break

        g0 = g_score[r, c]
        prev_d = int(came_dir[r, c])
        prev_dir: Optional[int] = None if prev_d < 0 else prev_d

        for dr, dc, step_len in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= h or nc < 0 or nc >= w:
                continue
            if closed[nr, nc] or not free[nr, nc]:
                continue
            if abs(dr) == 1 and abs(dc) == 1:
                if not free[r + dr, c] or not free[r, c + dc]:
                    continue

            ndir = _dir_id(dr, dc)
            tp = turn_penalty(prev_dir, ndir)
            cp = clearance_penalty(float(dist_m[nr, nc]), d_safe_m)
            gray_cell_cost = float(gray_cost[nr, nc])
            cost = step_len + weights.w_turn * tp + weights.w_clear * cp + weights.w_gray * gray_cell_cost
            ng = g0 + cost
            if ng < g_score[nr, nc]:
                g_score[nr, nc] = ng
                parent_r[nr, nc] = r
                parent_c[nr, nc] = c
                came_dir[nr, nc] = ndir
                f = ng + heuristic(nr, nc)
                heapq.heappush(open_heap, (f, counter, nr, nc))
                counter += 1

    if not np.isfinite(g_score[gr, gc]):
        raise RuntimeError("A* failed: no path between start and goal.")

    path: List[GridPoint] = []
    r, c = gr, gc
    while True:
        path.append((r, c))
        if r == sr and c == sc:
            break
        pr, pc = int(parent_r[r, c]), int(parent_c[r, c])
        if pr < 0:
            raise RuntimeError("A* path reconstruction failed.")
        r, c = pr, pc
    path.reverse()
    return path


def plan_mode(
    clearance: ClearanceMap,
    gray_cost: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    mode_weights: Dict[str, float],
    d_safe_m: float,
) -> List[GridPoint]:
    weights = PlannerWeights(
        w_turn=float(mode_weights.get("w_turn", 0.0)),
        w_clear=float(mode_weights.get("w_clear", 0.0)),
        w_gray=float(mode_weights.get("w_gray", 0.0)),
    )
    return astar(clearance.free, clearance.dist_m, gray_cost, start, goal, weights, d_safe_m)
