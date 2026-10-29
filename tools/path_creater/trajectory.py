"""Convert grid paths to world poses and write CSV / overlay."""

from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

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


def _overlay_font(size: int):
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
    )
    for candidate in candidates:
        if Path(candidate).is_file():
            try:
                return ImageFont.truetype(candidate, size)
            except OSError:
                continue
    return ImageFont.load_default()


def _draw_tracking_marker(draw: ImageDraw.ImageDraw, name: str, x: int, y: int, color, scale: int = 1) -> None:
    if name == "cost_aware":
        half = 2 * scale
        draw.rectangle((x - half, y - half, x + half, y + half), fill=color)
    elif name == "smooth":
        half = 2 * scale
        draw.polygon(((x, y - half), (x - half, y + half // 2), (x + half, y + half // 2)), fill=color)
    else:
        radius = max(0, scale - 1)
        if radius == 0:
            draw.point((x, y), fill=color)
        else:
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)


def _draw_overlay_legend(image: Image.Image, colors: Dict[str, Tuple[int, int, int]]) -> Image.Image:
    font_size = max(12, min(28, int(round(min(image.size) * 0.012))))
    font = _overlay_font(font_size)
    entries = (
        ("cost_aware", "蓝色正方形  road_map1.xml  综合代价优先"),
        ("distance", "红色圆点    road_map2.xml  距离优先"),
        ("smooth", "绿色三角形  road_map3.xml  平滑优先"),
    )
    footer = "黄色圈：起点    品红圈：终点"
    padding = 8
    line_height = font_size + 8
    dummy = ImageDraw.Draw(image)
    text_width = max(dummy.textbbox((0, 0), text, font=font)[2] for _, text in entries)
    text_width = max(text_width, dummy.textbbox((0, 0), footer, font=font)[2])
    legend_width = text_width + padding * 2 + 24
    legend_height = padding * 2 + line_height * (len(entries) + 1)
    layer = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    draw.rounded_rectangle(
        (padding, padding, padding + legend_width, padding + legend_height),
        radius=5,
        fill=(255, 255, 255, 225),
        outline=(40, 40, 40, 255),
        width=1,
    )
    for index, (name, text) in enumerate(entries):
        center_y = padding + padding + line_height * index + line_height // 2
        _draw_tracking_marker(draw, name, padding + 12, center_y, colors[name], scale=2)
        draw.text((padding + 24, center_y - font_size // 2), text, fill=(20, 20, 20, 255), font=font)
    footer_y = padding + padding + line_height * len(entries)
    draw.text((padding + 8, footer_y), footer, fill=(20, 20, 20, 255), font=font)
    return Image.alpha_composite(image.convert("RGBA"), layer).convert("RGB")


def write_overlay(
    omap: OccupancyMap,
    free: np.ndarray,
    paths: Dict[str, Sequence[GridPoint]],
    out_path: Path,
    start: GridPoint,
    goal: GridPoint,
    show_legend: bool = True,
) -> None:
    """Render grayscale map with thin colored paths and tracking-point markers."""
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

    for name, points in paths.items():
        color = colors.get(name, (255, 255, 0))
        display_points = []
        for r, c in points:
            ir = h - 1 - r
            if 0 <= ir < h and 0 <= c < omap.width:
                display_points.append((int(c), int(ir)))
        if len(display_points) >= 2:
            draw.line(display_points, fill=color, width=1)

    for r, c, color in (
        (start[0], start[1], (255, 255, 0)),
        (goal[0], goal[1], (255, 0, 255)),
    ):
        ir = h - 1 - r
        draw.ellipse(
            (int(c) - 8, int(ir) - 8, int(c) + 8, int(ir) + 8),
            outline=color,
            width=2,
        )

    marker_order = ("cost_aware", "smooth", "distance")
    for name in marker_order:
        points = paths.get(name, ())
        color = colors[name]
        for r, c in points:
            ir = h - 1 - r
            if 0 <= ir < h and 0 <= c < omap.width:
                x = int(c)
                y = int(ir)
                _draw_tracking_marker(draw, name, x, y, color)

    if show_legend:
        image = _draw_overlay_legend(image, colors)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)
