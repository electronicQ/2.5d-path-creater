# Full PCD To Global Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first complete workflow from known-origin ENU point-cloud-derived static maps to three global trajectory CSVs.

**Architecture:** Keep ROS/grid-map processing, static map saving, and offline trajectory planning separated. `grid_map/` preserves unknown map cells, `map/` serializes continuous PGM maps, and `src/` performs ROS-independent planning from geodetic start/goal inputs.

**Tech Stack:** Python 3.8+, NumPy, PyYAML, Pillow, scipy, standard-library `unittest`, ROS `grid_map`, ROS `nav_msgs/OccupancyGrid`, C++ grid_map filter plugin.

## Global Constraints

- Upstream SLAM and ENU `.pcd` generation are out of scope.
- Unknown static-map cells use grayscale `205`.
- Absolute obstacles use `gray <= 90`.
- First version forbids all `gray == 205` cells; small unknown-hole repair is a later upgrade.
- Planner modes are `distance`, `smooth`, and `cost_aware`.
- Each mode writes `*_geo.csv` with `latitude,longitude` and `*_enu.csv` with `x,y`.
- Use WGS84/ECEF/ENU conversion for medium-range maps, not flat degree approximations.
- Preserve ROS independence in `src/`.

---

## File Structure

- Modify `src/map_loader.py`: keep raw grayscale pixels alongside occupancy-like data.
- Create `src/map_semantics.py`: convert grayscale maps into forbidden/free masks and normalized gray costs.
- Modify `src/clearance.py`: build clearance from the new semantic forbidden mask.
- Modify `src/planner.py`: add gray-cost weight and per-cell gray cost.
- Create `src/geo.py`: WGS84 geodetic/ECEF/ENU conversion helpers.
- Create `src/map_metadata.py`: read map metadata files such as `map/changlilou_ENU.meta.yaml`.
- Modify `src/auto_endpoints.py`: support explicit ENU start/goal produced from geodetic inputs.
- Modify `src/trajectory.py`: write ENU and geodetic CSVs without yaw.
- Modify `src/main.py`: wire map metadata, geodetic inputs, semantic config, planning, and outputs.
- Modify `config.yaml`: add first-version explicit semantic and geodetic fields.
- Refactor `map/save_continuous_map.py`: expose pure conversion from OccupancyGrid values to PGM grayscale values.
- Create `tests/`: standard-library unit tests for map semantics, geo conversion, CSV output, and map saving conversion.
- Create `grid_map/grid_map_filters/include/grid_map_filters/RestoreUnknownMaskFilter.hpp`.
- Create `grid_map/grid_map_filters/src/RestoreUnknownMaskFilter.cpp`.
- Modify `grid_map/grid_map_filters/filter_plugins.xml`, `grid_map/grid_map_filters/CMakeLists.txt`, and `grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml`.

---

### Task 1: Python Test Scaffolding And Static Map Semantics

**Files:**
- Create: `tests/__init__.py`
- Create: `tests/test_map_semantics.py`
- Create: `src/map_semantics.py`
- Modify: `src/map_loader.py`

**Interfaces:**
- Produces: `MapSemantics(unknown_gray: int, absolute_obstacle_gray: int, gray_cost_weight: float)`
- Produces: `build_semantic_layers(gray: np.ndarray, semantics: MapSemantics) -> SemanticLayers`
- Produces: `SemanticLayers.forbidden`, `SemanticLayers.free`, `SemanticLayers.gray_cost`
- Consumes: raw grayscale `OccupancyMap.gray`

- [ ] **Step 1: Write failing tests for grayscale semantics**

Create `tests/__init__.py` as an empty file.

Create `tests/test_map_semantics.py`:

```python
import unittest

import numpy as np

from src.map_semantics import MapSemantics, build_semantic_layers


class MapSemanticsTest(unittest.TestCase):
    def test_unknown_and_obstacle_are_forbidden(self):
        gray = np.array([[205, 90, 91, 255]], dtype=np.uint8)
        layers = build_semantic_layers(
            gray,
            MapSemantics(unknown_gray=205, absolute_obstacle_gray=90, gray_cost_weight=1.0),
        )
        self.assertEqual(layers.forbidden.tolist(), [[True, True, False, False]])
        self.assertEqual(layers.free.tolist(), [[False, False, True, True]])

    def test_gray_cost_is_monotonic_and_zero_for_forbidden_cells(self):
        gray = np.array([[205, 90, 128, 255]], dtype=np.uint8)
        layers = build_semantic_layers(
            gray,
            MapSemantics(unknown_gray=205, absolute_obstacle_gray=90, gray_cost_weight=1.0),
        )
        self.assertEqual(float(layers.gray_cost[0, 0]), 0.0)
        self.assertEqual(float(layers.gray_cost[0, 1]), 0.0)
        self.assertGreater(float(layers.gray_cost[0, 2]), float(layers.gray_cost[0, 3]))
        self.assertAlmostEqual(float(layers.gray_cost[0, 3]), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify they fail**

Run:

```bash
python -m unittest tests.test_map_semantics -v
```

Expected: failure because `src.map_semantics` does not exist.

- [ ] **Step 3: Implement semantic layers**

Create `src/map_semantics.py`:

```python
"""Continuous grayscale map semantics for trajectory planning."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MapSemantics:
    unknown_gray: int = 205
    absolute_obstacle_gray: int = 90
    gray_cost_weight: float = 1.0


@dataclass(frozen=True)
class SemanticLayers:
    forbidden: np.ndarray
    free: np.ndarray
    gray_cost: np.ndarray


def build_semantic_layers(gray: np.ndarray, semantics: MapSemantics) -> SemanticLayers:
    gray_u8 = np.asarray(gray, dtype=np.uint8)
    unknown = gray_u8 == int(semantics.unknown_gray)
    obstacle = gray_u8 <= int(semantics.absolute_obstacle_gray)
    forbidden = unknown | obstacle
    free = ~forbidden
    normalized = 1.0 - gray_u8.astype(np.float32) / 255.0
    gray_cost = normalized * float(semantics.gray_cost_weight)
    gray_cost[forbidden] = 0.0
    return SemanticLayers(forbidden=forbidden, free=free, gray_cost=gray_cost.astype(np.float32))
```

- [ ] **Step 4: Store raw grayscale in loaded maps**

Modify `src/map_loader.py`:

```python
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
    occ: np.ndarray
    gray: np.ndarray
    image_path: Path
    yaml_path: Path
```

In `load_map`, after flipping pixels:

```python
gray = pixels.astype(np.uint8)
```

Return it:

```python
gray=gray,
```

- [ ] **Step 5: Run tests and commit**

Run:

```bash
python -m unittest tests.test_map_semantics -v
```

Expected: `OK`.

Commit:

```bash
git add src/map_loader.py src/map_semantics.py tests/__init__.py tests/test_map_semantics.py
git commit -m "Add continuous grayscale map semantics"
```

---

### Task 2: Planner Uses Semantic Masks And Gray Cost

**Files:**
- Create: `tests/test_planner_gray_cost.py`
- Modify: `src/clearance.py`
- Modify: `src/planner.py`

**Interfaces:**
- Consumes: `SemanticLayers.free`
- Consumes: `SemanticLayers.gray_cost`
- Produces: `plan_mode(clearance, gray_cost, start, goal, mode_weights, d_safe_m)`

- [ ] **Step 1: Write failing planner tests**

Create `tests/test_planner_gray_cost.py`:

```python
import unittest

import numpy as np

from src.clearance import ClearanceMap
from src.planner import plan_mode


class PlannerGrayCostTest(unittest.TestCase):
    def test_gray_cost_discourages_middle_high_cost_cell(self):
        free = np.ones((3, 5), dtype=bool)
        dist_m = np.full((3, 5), 10.0, dtype=np.float32)
        gray_cost = np.zeros((3, 5), dtype=np.float32)
        gray_cost[1, 2] = 50.0
        clearance = ClearanceMap(free=free, dist_m=dist_m, inflation_radius_m=0.0, occupied_thresh=0.0)
        path = plan_mode(
            clearance,
            gray_cost,
            (1, 0),
            (1, 4),
            {"w_turn": 0.0, "w_clear": 0.0, "w_gray": 10.0},
            d_safe_m=0.0,
        )
        self.assertNotIn((1, 2), path)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test and verify it fails**

Run:

```bash
python -m unittest tests.test_planner_gray_cost -v
```

Expected: failure because `plan_mode` does not accept `gray_cost`.

- [ ] **Step 3: Modify planner weights and cost**

In `src/planner.py`, change:

```python
@dataclass
class PlannerWeights:
    w_turn: float
    w_clear: float
    w_gray: float
```

Change `astar` signature:

```python
def astar(
    free: np.ndarray,
    dist_m: np.ndarray,
    gray_cost: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    weights: PlannerWeights,
    d_safe_m: float,
) -> List[GridPoint]:
```

Change neighbor cost:

```python
gc = float(gray_cost[nr, nc])
cost = step_len + weights.w_turn * tp + weights.w_clear * cp + weights.w_gray * gc
```

Change `plan_mode` signature and weights:

```python
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
```

- [ ] **Step 4: Let clearance accept a semantic free mask**

In `src/clearance.py`, add:

```python
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
```

- [ ] **Step 5: Run tests and commit**

Run:

```bash
python -m unittest tests.test_map_semantics tests.test_planner_gray_cost -v
```

Expected: `OK`.

Commit:

```bash
git add src/clearance.py src/planner.py tests/test_planner_gray_cost.py
git commit -m "Add gray cost to trajectory planning"
```

---

### Task 3: WGS84/ENU Conversion And Map Metadata

**Files:**
- Create: `src/geo.py`
- Create: `src/map_metadata.py`
- Create: `tests/test_geo.py`
- Create: `tests/test_map_metadata.py`

**Interfaces:**
- Produces: `GeoPoint(latitude: float, longitude: float, altitude: float = 0.0)`
- Produces: `geodetic_to_enu(point: GeoPoint, origin: GeoPoint) -> Tuple[float, float, float]`
- Produces: `enu_to_geodetic(x: float, y: float, z: float, origin: GeoPoint) -> GeoPoint`
- Produces: `load_map_metadata(path: Path) -> MapMetadata`

- [ ] **Step 1: Write failing geo tests**

Create `tests/test_geo.py`:

```python
import unittest

from src.geo import GeoPoint, enu_to_geodetic, geodetic_to_enu


class GeoConversionTest(unittest.TestCase):
    def test_origin_converts_to_zero_enu(self):
        origin = GeoPoint(latitude=30.0, longitude=120.0, altitude=10.0)
        x, y, z = geodetic_to_enu(origin, origin)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)

    def test_round_trip_medium_range(self):
        origin = GeoPoint(latitude=30.0, longitude=120.0, altitude=10.0)
        point = GeoPoint(latitude=30.001, longitude=120.002, altitude=10.0)
        x, y, z = geodetic_to_enu(point, origin)
        restored = enu_to_geodetic(x, y, z, origin)
        self.assertAlmostEqual(restored.latitude, point.latitude, places=8)
        self.assertAlmostEqual(restored.longitude, point.longitude, places=8)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Implement `src/geo.py`**

Create `src/geo.py`:

```python
"""WGS84 geodetic, ECEF, and local ENU conversions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


@dataclass(frozen=True)
class GeoPoint:
    latitude: float
    longitude: float
    altitude: float = 0.0


def _geodetic_to_ecef(point: GeoPoint) -> Tuple[float, float, float]:
    lat = math.radians(point.latitude)
    lon = math.radians(point.longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    x = (n + point.altitude) * cos_lat * math.cos(lon)
    y = (n + point.altitude) * cos_lat * math.sin(lon)
    z = (n * (1.0 - _E2) + point.altitude) * sin_lat
    return x, y, z


def _ecef_to_geodetic(x: float, y: float, z: float) -> GeoPoint:
    b = _A * (1.0 - _F)
    ep2 = (_A * _A - b * b) / (b * b)
    p = math.hypot(x, y)
    theta = math.atan2(z * _A, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(
        z + ep2 * b * math.sin(theta) ** 3,
        p - _E2 * _A * math.cos(theta) ** 3,
    )
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return GeoPoint(latitude=math.degrees(lat), longitude=math.degrees(lon), altitude=alt)


def geodetic_to_enu(point: GeoPoint, origin: GeoPoint) -> Tuple[float, float, float]:
    x, y, z = _geodetic_to_ecef(point)
    x0, y0, z0 = _geodetic_to_ecef(origin)
    lat0 = math.radians(origin.latitude)
    lon0 = math.radians(origin.longitude)
    dx, dy, dz = x - x0, y - y0, z - z0
    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = -math.sin(lat0) * math.cos(lon0) * dx - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz
    up = math.cos(lat0) * math.cos(lon0) * dx + math.cos(lat0) * math.sin(lon0) * dy + math.sin(lat0) * dz
    return east, north, up


def enu_to_geodetic(x: float, y: float, z: float, origin: GeoPoint) -> GeoPoint:
    x0, y0, z0 = _geodetic_to_ecef(origin)
    lat0 = math.radians(origin.latitude)
    lon0 = math.radians(origin.longitude)
    dx = -math.sin(lon0) * x - math.sin(lat0) * math.cos(lon0) * y + math.cos(lat0) * math.cos(lon0) * z
    dy = math.cos(lon0) * x - math.sin(lat0) * math.sin(lon0) * y + math.cos(lat0) * math.sin(lon0) * z
    dz = math.cos(lat0) * y + math.sin(lat0) * z
    return _ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)
```

- [ ] **Step 3: Write and implement metadata loader**

Create `tests/test_map_metadata.py`:

```python
import tempfile
import unittest
from pathlib import Path

from src.map_metadata import load_map_metadata


class MapMetadataTest(unittest.TestCase):
    def test_loads_enu_origin(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "map.meta.yaml"
            p.write_text(
                "enu_origin:\n  latitude: 30.0\n  longitude: 120.0\n  altitude: 5.0\n",
                encoding="utf-8",
            )
            meta = load_map_metadata(p)
            self.assertEqual(meta.enu_origin.latitude, 30.0)
            self.assertEqual(meta.enu_origin.longitude, 120.0)
            self.assertEqual(meta.enu_origin.altitude, 5.0)


if __name__ == "__main__":
    unittest.main()
```

Create `src/map_metadata.py`:

```python
"""Load geodetic metadata associated with a static map."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from .geo import GeoPoint


@dataclass(frozen=True)
class MapMetadata:
    enu_origin: GeoPoint


def load_map_metadata(path: Path) -> MapMetadata:
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError("Map metadata not found: {}".format(path))
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    origin = data.get("enu_origin")
    if not isinstance(origin, dict):
        raise ValueError("Map metadata missing enu_origin.")
    return MapMetadata(
        enu_origin=GeoPoint(
            latitude=float(origin["latitude"]),
            longitude=float(origin["longitude"]),
            altitude=float(origin.get("altitude", 0.0)),
        )
    )
```

- [ ] **Step 4: Run tests and commit**

Run:

```bash
python -m unittest tests.test_geo tests.test_map_metadata -v
```

Expected: `OK`.

Commit:

```bash
git add src/geo.py src/map_metadata.py tests/test_geo.py tests/test_map_metadata.py
git commit -m "Add geodetic map metadata conversion"
```

---

### Task 4: Main Planner Wiring And CSV Outputs

**Files:**
- Create: `tests/test_trajectory_outputs.py`
- Modify: `src/trajectory.py`
- Modify: `src/main.py`
- Modify: `config.yaml`

**Interfaces:**
- Consumes: `GeoPoint`, `MapMetadata`, `SemanticLayers`
- Produces: `write_enu_csv(path: Path, points: Iterable[Tuple[float, float]]) -> None`
- Produces: `write_geo_csv(path: Path, points: Iterable[GeoPoint]) -> None`

- [ ] **Step 1: Write failing CSV tests**

Create `tests/test_trajectory_outputs.py`:

```python
import tempfile
import unittest
from pathlib import Path

from src.geo import GeoPoint
from src.trajectory import write_enu_csv, write_geo_csv


class TrajectoryOutputTest(unittest.TestCase):
    def test_write_enu_csv(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "traj_enu.csv"
            write_enu_csv(p, [(1.25, 2.5)])
            self.assertEqual(p.read_text(encoding="utf-8").strip(), "x,y\n1.250000,2.500000")

    def test_write_geo_csv(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "traj_geo.csv"
            write_geo_csv(p, [GeoPoint(latitude=30.1, longitude=120.2)])
            self.assertEqual(p.read_text(encoding="utf-8").strip(), "latitude,longitude\n30.10000000,120.20000000")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Implement CSV writers**

Add to `src/trajectory.py`:

```python
from .geo import GeoPoint


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
```

- [ ] **Step 3: Wire `src/main.py`**

Modify imports:

```python
from .geo import GeoPoint, enu_to_geodetic, geodetic_to_enu
from .map_metadata import load_map_metadata
from .map_semantics import MapSemantics, build_semantic_layers
from .trajectory import path_to_poses, write_enu_csv, write_geo_csv, write_overlay
```

Add config reads:

```python
map_meta_path = Path(cfg["map_meta"])
if not map_meta_path.is_absolute():
    map_meta_path = (root / map_meta_path).resolve()
map_meta = load_map_metadata(map_meta_path)

semantics_cfg = cfg.get("map_semantics", {})
gray_cfg = cfg.get("gray_cost", {})
semantics = MapSemantics(
    unknown_gray=int(semantics_cfg.get("unknown_gray", cfg.get("unknown_gray", 205))),
    absolute_obstacle_gray=int(semantics_cfg.get("absolute_obstacle_gray", cfg.get("absolute_obstacle_gray", 90))),
    gray_cost_weight=float(gray_cfg.get("weight", cfg.get("gray_cost_weight", 1.0))),
)
semantic_layers = build_semantic_layers(omap.gray, semantics)
clearance = build_clearance_from_free(
    semantic_layers.free,
    omap.resolution,
    inflation_radius_m=inflation,
    occupied_thresh=0.0,
)
```

Replace start/goal config handling with geodetic conversion:

```python
start_cfg = cfg.get("start")
goal_cfg = cfg.get("goal")
if start_cfg is None or goal_cfg is None:
    raise ValueError("Geodetic start and goal are required for global trajectory output.")
start_geo = GeoPoint(float(start_cfg["latitude"]), float(start_cfg["longitude"]), 0.0)
goal_geo = GeoPoint(float(goal_cfg["latitude"]), float(goal_cfg["longitude"]), 0.0)
sx, sy, _ = geodetic_to_enu(start_geo, map_meta.enu_origin)
gx, gy, _ = geodetic_to_enu(goal_geo, map_meta.enu_origin)
start_rc, goal_rc, start_xy, goal_xy, auto_picked = resolve_endpoints(
    omap,
    clearance,
    [sx, sy],
    [gx, gy],
    seed=int(cfg.get("seed", 0)),
)
```

Change planning call and outputs:

```python
path = plan_mode(clearance, semantic_layers.gray_cost, start_rc, goal_rc, weights, d_safe_m=d_safe)
enu_points = [omap.grid_to_world(r, c) for r, c in path]
geo_points = [enu_to_geodetic(x, y, 0.0, map_meta.enu_origin) for x, y in enu_points]
write_enu_csv(out_dir / "traj_{}_enu.csv".format(mode), enu_points)
write_geo_csv(out_dir / "traj_{}_geo.csv".format(mode), geo_points)
```

Keep `write_overlay` using `paths`.

- [ ] **Step 4: Update config schema**

Modify `config.yaml` to use explicit keys. The latitude/longitude values below demonstrate the required schema. Before running the full planner on a real map, replace them with the measured start, goal, and map-origin values for the selected map:

```yaml
map_yaml: map/changlilou_ENU.yaml
map_meta: map/changlilou_ENU.meta.yaml

start:
  latitude: 30.0
  longitude: 120.0
goal:
  latitude: 30.0001
  longitude: 120.0001

unknown_gray: 205
absolute_obstacle_gray: 90
gray_cost:
  enabled: true
  weight: 1.0

planners:
  distance:
    w_turn: 0.0
    w_clear: 1.0
    w_gray: 0.5
  smooth:
    w_turn: 4.0
    w_clear: 2.0
    w_gray: 1.0
  cost_aware:
    w_turn: 1.5
    w_clear: 4.0
    w_gray: 3.0
```

Create `map/changlilou_ENU.meta.yaml` only after the measured ENU origin for `map/changlilou_ENU.yaml` is known:

```yaml
enu_origin:
  latitude: 30.0
  longitude: 120.0
  altitude: 0.0
```

Do not commit guessed geodetic values as project defaults.

- [ ] **Step 5: Run unit tests and commit**

Run:

```bash
python -m unittest tests.test_trajectory_outputs -v
python -m unittest discover -s tests -v
```

Expected: `OK`.

Commit:

```bash
git add src/main.py src/trajectory.py config.yaml tests/test_trajectory_outputs.py
git commit -m "Write geodetic and ENU trajectory outputs"
```

---

### Task 5: Static Map Saver Preserves Unknown Cells

**Files:**
- Create: `tests/test_map_saver_conversion.py`
- Modify: `map/save_continuous_map.py`

**Interfaces:**
- Produces: `occupancy_to_pgm_array(data: Sequence[int], width: int, height: int) -> np.ndarray`

- [ ] **Step 1: Write failing conversion test**

Create `tests/test_map_saver_conversion.py`:

```python
import importlib.util
import unittest
from pathlib import Path


def load_saver_module():
    path = Path("map/save_continuous_map.py").resolve()
    spec = importlib.util.spec_from_file_location("save_continuous_map", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MapSaverConversionTest(unittest.TestCase):
    def test_occupancy_values_map_to_expected_grayscale(self):
        module = load_saver_module()
        img = module.occupancy_to_pgm_array([-1, 100, 0, 50], width=4, height=1)
        self.assertEqual(img.tolist(), [[205, 0, 254, 127]])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Refactor pure conversion**

In `map/save_continuous_map.py`, add:

```python
def occupancy_to_pgm_array(values, width, height):
    data = np.array(values).reshape((height, width))
    img = np.full((height, width), 255, dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            val = int(data[y, x])
            if val == -1:
                img[y, x] = 205
            elif val >= 100:
                img[y, x] = 0
            elif val <= 0:
                img[y, x] = 254
            else:
                img[y, x] = int(round(255 - (val / 100.0) * 255))
    return img
```

In `map_callback`, replace the loop with:

```python
img = occupancy_to_pgm_array(msg.data, width, height)
```

- [ ] **Step 3: Make imports test-safe**

Wrap ROS imports so the pure conversion test can run outside ROS:

```python
try:
    import rospy
    from nav_msgs.msg import OccupancyGrid
except ImportError:
    rospy = None
    OccupancyGrid = None
```

At runtime entry:

```python
if rospy is None:
    raise RuntimeError("ROS is required to run the map saver node.")
```

- [ ] **Step 4: Run tests and commit**

Run:

```bash
python -m unittest tests.test_map_saver_conversion -v
```

Expected: `OK`.

Commit:

```bash
git add map/save_continuous_map.py tests/test_map_saver_conversion.py
git commit -m "Preserve unknown cells in static map saver"
```

---

### Task 6: Restore Unknown Mask In Grid Map Filter Chain

**Files:**
- Create: `grid_map/grid_map_filters/include/grid_map_filters/RestoreUnknownMaskFilter.hpp`
- Create: `grid_map/grid_map_filters/src/RestoreUnknownMaskFilter.cpp`
- Modify: `grid_map/grid_map_filters/CMakeLists.txt`
- Modify: `grid_map/grid_map_filters/filter_plugins.xml`
- Modify: `grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml`

**Interfaces:**
- Produces ROS filter plugin: `gridMapFilters/RestoreUnknownMaskFilter`
- Parameters:
  - `mask_layer`: original layer whose non-finite cells define unknown areas.
  - `input_layer`: computed layer to preserve in known cells.
  - `output_layer`: layer to overwrite or create with unknown mask restored.

- [ ] **Step 1: Add filter header**

Create `grid_map/grid_map_filters/include/grid_map_filters/RestoreUnknownMaskFilter.hpp`:

```cpp
#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <string>

namespace grid_map {

class RestoreUnknownMaskFilter : public filters::FilterBase<GridMap> {
 public:
  RestoreUnknownMaskFilter();
  ~RestoreUnknownMaskFilter() override;

  bool configure() override;
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  std::string maskLayer_;
  std::string inputLayer_;
  std::string outputLayer_;
};

}  // namespace grid_map
```

- [ ] **Step 2: Add filter implementation**

Create `grid_map/grid_map_filters/src/RestoreUnknownMaskFilter.cpp`:

```cpp
#include "grid_map_filters/RestoreUnknownMaskFilter.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace grid_map {

RestoreUnknownMaskFilter::RestoreUnknownMaskFilter() = default;

RestoreUnknownMaskFilter::~RestoreUnknownMaskFilter() = default;

bool RestoreUnknownMaskFilter::configure() {
  if (!FilterBase::getParam(std::string("mask_layer"), maskLayer_)) {
    ROS_ERROR("RestoreUnknownMaskFilter did not find parameter 'mask_layer'.");
    return false;
  }
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("RestoreUnknownMaskFilter did not find parameter 'input_layer'.");
    return false;
  }
  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("RestoreUnknownMaskFilter did not find parameter 'output_layer'.");
    return false;
  }
  return true;
}

bool RestoreUnknownMaskFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  if (!mapOut.exists(maskLayer_)) {
    ROS_ERROR_STREAM("RestoreUnknownMaskFilter missing mask layer: " << maskLayer_);
    return false;
  }
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR_STREAM("RestoreUnknownMaskFilter missing input layer: " << inputLayer_);
    return false;
  }

  Matrix output = mapOut[inputLayer_];
  const Matrix& mask = mapOut[maskLayer_];
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    if (!std::isfinite(mask(index(0), index(1)))) {
      output(index(0), index(1)) = NAN;
    }
  }
  mapOut.add(outputLayer_, output);
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::RestoreUnknownMaskFilter, filters::FilterBase<grid_map::GridMap>)
```

- [ ] **Step 3: Register plugin and build target**

In `grid_map/grid_map_filters/CMakeLists.txt`, add `src/RestoreUnknownMaskFilter.cpp` to the existing `grid_map_filters` library source list.

In `grid_map/grid_map_filters/filter_plugins.xml`, add:

```xml
<class name="gridMapFilters/RestoreUnknownMaskFilter" type="grid_map::RestoreUnknownMaskFilter" base_class_type="filters::FilterBase<grid_map::GridMap>" >
  <description>Restore NaN cells in an output layer based on a mask layer.</description>
</class>
```

- [ ] **Step 4: Use filter after traversability thresholds**

In `grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml`, add this final filter after `traversability_upper_threshold`:

```yaml
  - name: restore_unknown_mask
    type: gridMapFilters/RestoreUnknownMaskFilter
    params:
      mask_layer: elevation
      input_layer: traversability
      output_layer: traversability
```

Keep `strict_inpaint` for intermediate calculations. The final filter restores unknown cells from original `elevation`.

- [ ] **Step 5: Build and verify plugin**

Run from the ROS workspace root:

```bash
catkin_make
```

Expected: build succeeds and `grid_map_filters` compiles with `RestoreUnknownMaskFilter`.

Launch the filter chain:

```bash
roslaunch grid_map_demos filters_demo_gai.launch
```

Expected: no plugin load error for `gridMapFilters/RestoreUnknownMaskFilter`.

- [ ] **Step 6: Commit**

Commit:

```bash
git add grid_map/grid_map_filters/include/grid_map_filters/RestoreUnknownMaskFilter.hpp grid_map/grid_map_filters/src/RestoreUnknownMaskFilter.cpp grid_map/grid_map_filters/CMakeLists.txt grid_map/grid_map_filters/filter_plugins.xml grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml
git commit -m "Restore unknown mask in grid map traversability"
```

---

### Task 7: End-To-End Verification

**Files:**
- Modify only if earlier verification exposes a defect in a file changed by Tasks 1-6.

**Interfaces:**
- Consumes: all implemented tasks.
- Produces: verified output files in `output/`.

- [ ] **Step 1: Run all Python unit tests**

Run:

```bash
python -m unittest discover -s tests -v
```

Expected: all tests pass.

- [ ] **Step 2: Verify static planner with measured geodetic inputs**

Before this step, confirm `config.yaml` contains measured start/goal coordinates and `map_meta` points to the metadata file with the measured ENU origin for the selected map. Run:

```bash
python -m src.main --config config.yaml
```

Expected:

```text
traj_distance_geo.csv
traj_distance_enu.csv
traj_smooth_geo.csv
traj_smooth_enu.csv
traj_cost_aware_geo.csv
traj_cost_aware_enu.csv
overlay.png
```

- [ ] **Step 3: Verify unknown cells remain forbidden**

Add a temporary diagnostic print or one-off shell check to confirm the loaded PGM contains `205` values and planned paths never include `gray == 205`.

Use this PowerShell command:

```powershell
python -c "from PIL import Image; import numpy as np; img=np.array(Image.open('map/changlilou_ENU.pgm').convert('L')); print(int((img==205).sum()))"
```

Expected: count is greater than zero for maps with unknown cells.

- [ ] **Step 4: Verify ROS map saving chain**

Run the `grid_map` launch and save script in ROS:

```bash
roslaunch grid_map_demos filters_demo_gai.launch
python map/save_continuous_map.py -n map/test_saved
```

Expected: generated `map/test_saved.pgm` contains grayscale `205` for unobserved regions.

- [ ] **Step 5: Commit final verification fixes**

If Task 7 required fixes, inspect the changed paths:

```bash
git status --short
```

Then commit only files changed to fix Task 7 verification failures:

```bash
git add src/main.py src/planner.py src/trajectory.py src/map_loader.py src/map_semantics.py src/geo.py src/map_metadata.py map/save_continuous_map.py grid_map/grid_map_filters/include/grid_map_filters/RestoreUnknownMaskFilter.hpp grid_map/grid_map_filters/src/RestoreUnknownMaskFilter.cpp grid_map/grid_map_filters/CMakeLists.txt grid_map/grid_map_filters/filter_plugins.xml grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml
git commit -m "Verify full map to trajectory workflow"
```

If no fixes were needed, do not create an empty commit.
