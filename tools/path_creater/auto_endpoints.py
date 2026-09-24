"""Resolve start/goal: config overrides or auto-pick a connected free pair."""

from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

import numpy as np

from .clearance import ClearanceMap
from .map_loader import OccupancyMap


GridPoint = Tuple[int, int]


def _bfs_farthest(free: np.ndarray, start: GridPoint) -> Tuple[GridPoint, int]:
    """Return farthest reachable free cell and hop count from start (4-connected)."""
    h, w = free.shape
    sr, sc = start
    visited = np.zeros_like(free, dtype=bool)
    q: deque = deque()
    q.append((sr, sc, 0))
    visited[sr, sc] = True
    farthest = start
    farthest_d = 0
    dirs = ((-1, 0), (1, 0), (0, -1), (0, 1))

    while q:
        r, c, d = q.popleft()
        if d >= farthest_d:
            farthest_d = d
            farthest = (r, c)
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= h or nc < 0 or nc >= w:
                continue
            if visited[nr, nc] or not free[nr, nc]:
                continue
            visited[nr, nc] = True
            q.append((nr, nc, d + 1))

    return farthest, farthest_d


def _candidate_free_cells(
    free: np.ndarray,
    dist_m: np.ndarray,
    margin: int,
    min_clear_m: float,
    max_samples: int = 2000,
) -> List[GridPoint]:
    h, w = free.shape
    if h <= 2 * margin or w <= 2 * margin:
        margin = 0
    region = np.zeros_like(free, dtype=bool)
    region[margin : h - margin, margin : w - margin] = True
    mask = free & region & (dist_m >= min_clear_m)
    rs, cs = np.where(mask)
    if len(rs) == 0:
        rs, cs = np.where(free)
    if len(rs) == 0:
        raise RuntimeError("No free cells after inflation; relax inflation_radius_m or thresh.")

    # Prefer higher clearance: sample from top percentile
    scores = dist_m[rs, cs]
    order = np.argsort(scores)[::-1]
    top_n = min(len(order), max(max_samples * 5, 5000))
    order = order[:top_n]
    if len(order) > max_samples:
        pick = np.linspace(0, len(order) - 1, max_samples, dtype=int)
        order = order[pick]
    return [(int(rs[i]), int(cs[i])) for i in order]


def auto_pick_endpoints(
    omap: OccupancyMap,
    clearance: ClearanceMap,
    seed: int = 0,
) -> Tuple[GridPoint, GridPoint, Tuple[float, float], Tuple[float, float]]:
    """
    Pick start as a high-clearance free cell, goal as BFS-farthest reachable cell.
    Returns (start_rc, goal_rc, start_xy, goal_xy).
    """
    rng = np.random.default_rng(seed)
    margin = max(5, int(2.0 / omap.resolution))
    min_clear = max(clearance.inflation_radius_m * 1.2, clearance.inflation_radius_m + 0.2)
    candidates = _candidate_free_cells(clearance.free, clearance.dist_m, margin, min_clear)

    # Try a few starts if the connected component is tiny
    best = None
    idxs = list(range(len(candidates)))
    rng.shuffle(idxs)
    for k in idxs[: min(12, len(idxs))]:
        start = candidates[k]
        goal, dist_hops = _bfs_farthest(clearance.free, start)
        if dist_hops < 20:
            continue
        if best is None or dist_hops > best[2]:
            best = (start, goal, dist_hops)
            if dist_hops > 200:
                break

    if best is None:
        start = candidates[0]
        goal, dist_hops = _bfs_farthest(clearance.free, start)
        if start == goal:
            raise RuntimeError("Could not find a traversable start/goal pair.")
        best = (start, goal, dist_hops)

    start_rc, goal_rc, _ = best
    start_xy = omap.grid_to_world(*start_rc)
    goal_xy = omap.grid_to_world(*goal_rc)
    return start_rc, goal_rc, start_xy, goal_xy


def resolve_endpoints(
    omap: OccupancyMap,
    clearance: ClearanceMap,
    start_xy: Optional[List[float]],
    goal_xy: Optional[List[float]],
    seed: int = 0,
) -> Tuple[GridPoint, GridPoint, Tuple[float, float], Tuple[float, float], bool]:
    """
    Resolve endpoints from config or auto-pick.
    Returns start_rc, goal_rc, start_xy, goal_xy, auto_picked.
    """
    if start_xy is not None and goal_xy is not None:
        sr, sc = omap.world_to_grid(float(start_xy[0]), float(start_xy[1]))
        gr, gc = omap.world_to_grid(float(goal_xy[0]), float(goal_xy[1]))
        if not omap.in_bounds(sr, sc):
            raise ValueError("Configured start outside map bounds.")
        if not omap.in_bounds(gr, gc):
            raise ValueError("Configured goal outside map bounds.")
        if not clearance.free[sr, sc]:
            raise ValueError("Configured start is in inflated obstacle / non-free cell.")
        if not clearance.free[gr, gc]:
            raise ValueError("Configured goal is in inflated obstacle / non-free cell.")
        sx, sy = omap.grid_to_world(sr, sc)
        gx, gy = omap.grid_to_world(gr, gc)
        return (sr, sc), (gr, gc), (sx, sy), (gx, gy), False

    start_rc, goal_rc, sxy, gxy = auto_pick_endpoints(omap, clearance, seed=seed)
    return start_rc, goal_rc, sxy, gxy, True
