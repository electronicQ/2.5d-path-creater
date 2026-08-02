"""Convert grid paths to world poses and write CSV / overlay."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .geo import GeoPoint
from .map_loader import OccupancyMap


GridPoint = Tuple[int, int]
Pose = Tuple[float, float, float]


def path_to_poses(omap: OccupancyMap, path: Sequence[GridPoint]) -> List[Pose]:
    if len(path) == 0:
        return []
    poses: List[Pose] = []
    for i, (r, c) in enumerate(path):
        x, y = omap.grid_to_world(r, c)
        if i + 1 < len(path):
            x2, y2 = omap.grid_to_world(*path[i + 1])
            yaw = math.atan2(y2 - y, x2 - x)
        elif poses:
            yaw = poses[-1][2]
        else:
            yaw = 0.0
        poses.append((x, y, yaw))
    return poses


def write_trajectory_csv(path: Path, poses: Iterable[Pose]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw"])
        for x, y, yaw in poses:
            writer.writerow(["{:.6f}".format(x), "{:.6f}".format(y), "{:.6f}".format(yaw)])


def write_enu_csv(path: Path, points: Iterable[Tuple[float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x, y in points:
            writer.writerow(["{:.6f}".format(x), "{:.6f}".format(y)])


def write_geo_csv(path: Path, points: Iterable[GeoPoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["latitude", "longitude"])
        for point in points:
            writer.writerow(["{:.8f}".format(point.latitude), "{:.8f}".format(point.longitude)])


def write_overlay(
    omap: OccupancyMap,
    free: np.ndarray,
    paths: Dict[str, Sequence[GridPoint]],
    out_path: Path,
    start: GridPoint,
    goal: GridPoint,
) -> None:
    """Render grayscale map with colored paths (image top = max world y)."""
    # occ visualized: white=free, black=occupied
    gray = (255.0 * (1.0 - np.clip(omap.occ, 0.0, 1.0))).astype(np.uint8)
    # Mark inflated obstacles slightly darker gray for context
    vis = np.stack([gray, gray, gray], axis=-1)
    blocked = ~free
    vis[blocked] = (vis[blocked] * 0.55).astype(np.uint8)
    # Internal maps store row 0 at minimum world-y, while image viewers expect
    # row 0 at the top. Flip back to the raw PGM orientation for overlays.
    vis = np.flipud(vis)

    colors = {
        "distance": (220, 40, 40),
        "smooth": (40, 160, 40),
        "cost_aware": (40, 80, 220),
    }
    h = omap.height
    image = Image.fromarray(vis, mode="RGB")
    draw = ImageDraw.Draw(image)
    line_width = max(1, int(round(1.5 / max(float(omap.resolution), 1e-6))))
    for name, path in paths.items():
        color = colors.get(name, (255, 255, 0))
        display_points = []
        for r, c in path:
            ir = h - 1 - r
            if 0 <= ir < h and 0 <= c < omap.width:
                display_points.append((int(c), int(ir)))
        if len(display_points) >= 2:
            draw.line(display_points, fill=color, width=line_width)
        elif len(display_points) == 1:
            draw.point(display_points[0], fill=color)

    for r, c, color in (
        (start[0], start[1], (255, 255, 0)),
        (goal[0], goal[1], (255, 0, 255)),
    ):
        ir = h - 1 - r
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                rr, cc = ir + dr, c + dc
                if 0 <= rr < h and 0 <= cc < omap.width:
                    image.putpixel((cc, rr), color)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)
