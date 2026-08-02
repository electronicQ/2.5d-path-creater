"""Load ROS map_server style pgm + yaml occupancy grids."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml
from PIL import Image


@dataclass
class OccupancyMap:
    """Occupancy grid with row 0 at map origin (image flipped vs raw PGM)."""

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: int
    occupied_thresh: float
    free_thresh: float
    # occupancy probability in [0, 1], shape (H, W), row 0 = min world y
    occ: np.ndarray
    gray: np.ndarray
    image_path: Path
    yaml_path: Path

    @property
    def height(self) -> int:
        return int(self.occ.shape[0])

    @property
    def width(self) -> int:
        return int(self.occ.shape[1])

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Return (row, col) with row 0 at origin-y."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Cell center in world meters."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width


def load_map(map_yaml_path: Path, occupied_thresh_override: Optional[float] = None) -> OccupancyMap:
    map_yaml_path = Path(map_yaml_path).resolve()
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    image_name = meta["image"]
    image_path = (map_yaml_path.parent / image_name).resolve()
    if not image_path.is_file():
        raise FileNotFoundError("Map image not found: {}".format(image_path))

    resolution = float(meta["resolution"])
    origin = meta["origin"]
    origin_x, origin_y = float(origin[0]), float(origin[1])
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(
        occupied_thresh_override
        if occupied_thresh_override is not None
        else meta.get("occupied_thresh", 0.65)
    )
    free_thresh = float(meta.get("free_thresh", 0.25))

    img = Image.open(image_path)
    if img.mode != "L":
        img = img.convert("L")
    pixels = np.asarray(img, dtype=np.float64)
    # PIL: row 0 = top of image; ROS origin is bottom-left → flip vertically
    pixels = np.flipud(pixels)
    gray = pixels.astype(np.uint8)

    if negate:
        occ = pixels / 255.0
    else:
        occ = (255.0 - pixels) / 255.0

    return OccupancyMap(
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        negate=negate,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
        occ=occ.astype(np.float32),
        gray=gray,
        image_path=image_path,
        yaml_path=map_yaml_path,
    )
