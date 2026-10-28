"""Convert grid paths to world poses and write CSV / overlay."""

from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .geo import GeoPoint
from .map_loader import OccupancyMap


GridPoint = Tuple[int, int]
Pose = Tuple[float, float, float]

LEGACY_OUTPUT_FILENAMES = (
    "traj_distance_geo.csv",
    "traj_distance_enu.csv",
    "traj_smooth_geo.csv",
    "traj_smooth_enu.csv",
    "traj_cost_aware_geo.csv",
    "traj_cost_aware_enu.csv",
    "config.resolved.yaml",
)


def remove_legacy_outputs(directory: Path) -> None:
    for filename in LEGACY_OUTPUT_FILENAMES:
        path = directory / filename
        try:
            path.unlink()
        except FileNotFoundError:
            pass


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


def write_road_map_xml(path: Path, points: Sequence[Tuple[float, float]]) -> None:
    """Write local UTM points using the project's ``road_map.xml`` schema.

    The first vertex is marked as type ``0`` (start), subsequent vertices as
    type ``2`` (ordinary path points), and consecutive vertices are connected
    by directed edges.  Yaw is expressed in radians and follows the next
    segment, with the final point reusing the previous segment direction.
    """
    root = ET.Element("map", {"generator": "path creater", "upload": "true", "version": "0.6"})
    yaws = []
    for index, point in enumerate(points):
        if len(points) < 2:
            yaw = 0.0
        elif index + 1 < len(points):
            next_x, next_y = points[index + 1]
            yaw = math.atan2(next_y - point[1], next_x - point[0])
        else:
            previous_x, previous_y = points[index - 1]
            yaw = math.atan2(point[1] - previous_y, point[0] - previous_x)
        yaws.append(yaw)

    for index, ((x, y), yaw) in enumerate(zip(points, yaws)):
        ET.SubElement(
            root,
            "vertex",
            {
                "id": "{:04d}".format(index),
                "x": "{:.6f}".format(float(x)),
                "y": "{:.6f}".format(float(y)),
                "yaw": "{:.6f}".format(yaw),
                "Name": "----",
            },
        )
    for index in range(max(0, len(points) - 1)):
        ET.SubElement(
            root,
            "edge",
            {
                "id": "{:04d}".format(index),
                "src_id": str(index),
                "dst_id": str(index + 1),
            },
        )
    for index in range(len(points)):
        ET.SubElement(
            root,
            "type",
            {
                "id": "{:04d}".format(index),
                "vertexId": "{:04d}".format(index),
                "type": "0" if index == 0 else "2",
            },
        )

    _indent_xml(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    tree.write(path, encoding="UTF-8", xml_declaration=True)


def _indent_xml(element: ET.Element, level: int = 0) -> None:
    """Pretty-print an ElementTree without requiring Python 3.9's ET.indent."""
    indent = "\n" + "  " * level
    children = list(element)
    if not children:
        return
    element.text = indent + "  "
    for index, child in enumerate(children):
        _indent_xml(child, level + 1)
        child.tail = indent + ("  " if index + 1 < len(children) else "")


def write_overlay(
    omap: OccupancyMap,
    free: np.ndarray,
    paths: Dict[str, Sequence[GridPoint]],
    out_path: Path,
    start: GridPoint,
    goal: GridPoint,
) -> None:
    """Render grayscale map with discrete colored tracking points."""
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
    point_radius = 2
    for name, points in paths.items():
        color = colors.get(name, (255, 255, 0))
        for r, c in points:
            ir = h - 1 - r
            if 0 <= ir < h and 0 <= c < omap.width:
                draw.ellipse(
                    (
                        int(c) - point_radius,
                        int(ir) - point_radius,
                        int(c) + point_radius,
                        int(ir) + point_radius,
                    ),
                    fill=color,
                )

    for r, c, color in (
        (start[0], start[1], (255, 255, 0)),
        (goal[0], goal[1], (255, 0, 255)),
    ):
        ir = h - 1 - r
        draw.ellipse(
            (int(c) - 3, int(ir) - 3, int(c) + 3, int(ir) + 3),
            fill=color,
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)
