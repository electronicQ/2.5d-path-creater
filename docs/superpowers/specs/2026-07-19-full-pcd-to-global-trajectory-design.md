# Full PCD to Global Trajectory Design

## Scope

The first implementation target starts after upstream SLAM has already produced a local ENU `.pcd` map with a known geodetic origin. This project owns the flow from that `.pcd` map to global trajectory CSV files.

In scope:

- Use `grid_map` to convert the ENU point cloud map into a traversability grid.
- Preserve unobserved point-cloud regions as unknown areas.
- Save the traversability `OccupancyGrid` as continuous grayscale `pgm + yaml`.
- Read static maps in `src/` and plan three trajectories from latitude/longitude start and goal inputs.
- Output both geodetic and ENU CSV files for each trajectory.

Out of scope for the first version:

- SLAM map generation.
- Creating the upstream ENU `.pcd` map.
- Small unknown-hole repair.
- Speed planning, tracking control, ROS navigation stack integration, or planner plugin work.

## Data Flow

```text
Known-origin ENU .pcd map
-> grid_map elevation/traversability processing
-> OccupancyGrid traversability topic
-> map/save_continuous_map.py
-> continuous grayscale PGM + YAML
-> src/ static-map planner
-> distance / smooth / cost_aware trajectories
-> geo CSV + ENU CSV outputs
```

## Module Responsibilities

`grid_map/` converts point-cloud-derived elevation data into traversability layers. Inpainting may be used only as an intermediate calculation aid. The final traversability layer must restore the original unknown mask: cells that were `NaN` in the original elevation layer must remain `NaN` before visualization/conversion.

`map/` saves ROS `OccupancyGrid` topics to static map files. Unknown cells must be saved as grayscale `205`; lethal obstacles as black; free or low-cost cells as bright values; intermediate occupancy values as continuous grayscale costs.

`src/` is ROS-independent. It reads static `pgm + yaml` maps, map metadata, and geodetic start/goal inputs, then performs planning and writes trajectory CSVs.

## Map Metadata

Each static map should have a separate metadata file, for example:

```yaml
enu_origin:
  latitude: 30.0
  longitude: 120.0
  altitude: 0.0
```

The map YAML remains responsible for `image`, `resolution`, `origin`, and related image metadata. Traditional `occupied_thresh` and `free_thresh` are not the primary planning semantics for this continuous grayscale map.

## Static Map Semantics

Planner configuration should expose explicit grayscale thresholds:

```yaml
unknown_gray: 205
absolute_obstacle_gray: 90
gray_cost:
  enabled: true
  weight: 1.0
```

Planning interpretation:

- `gray == 205`: unknown, forbidden in the first version.
- `gray <= 90`: absolute obstacle, forbidden.
- `gray > 90` and `gray != 205`: traversable continuous-cost cell.

The first version must not repair small unknown holes. That is a later upgrade. The initial goal is to prove the full chain preserves unknown regions correctly.

## Planning Costs

The A* step cost should combine geometry, continuous grayscale cost, clearance, and turning:

```text
step_cost =
  geometry_cost
  + gray_cost_weight * normalized_gray_cost
  + clearance_weight * clearance_penalty
  + turn_weight * turn_penalty
```

Use a monotonic grayscale cost such as:

```text
normalized_gray_cost = 1.0 - gray / 255.0
```

Three planned modes:

- `distance`: favors shorter paths.
- `smooth`: favors fewer turns and smoother paths.
- `cost_aware`: favors safer, lower-cost, higher-clearance paths.

## Coordinate Conversion

The planner input start and goal are latitude/longitude values. Internally:

```text
latitude/longitude
-> current-map ENU x,y
-> grid row,col
-> planned grid path
-> ENU x,y
-> latitude/longitude
```

Use a WGS84/ENU conversion suitable for medium-range maps, not a simple flat-earth degree approximation.

## Outputs

Each trajectory mode writes two CSV files:

```text
traj_distance_geo.csv
traj_distance_enu.csv
traj_smooth_geo.csv
traj_smooth_enu.csv
traj_cost_aware_geo.csv
traj_cost_aware_enu.csv
```

CSV schemas:

```text
*_geo.csv: latitude,longitude
*_enu.csv: x,y
```

The geodetic files are the primary global trajectory files for downstream tracking systems. ENU files are for debugging, overlay checks, and validation against the current static map.

## Validation And Failure Handling

Fail early when:

- `map_yaml` or map metadata is missing.
- The map image path is invalid.
- `enu_origin` is missing.
- Start or goal is outside the map.
- Start or goal lands on `unknown_gray` or an absolute obstacle.
- Any generated path crosses `unknown_gray` or `gray <= absolute_obstacle_gray`.
- A planning mode cannot find a path.

Warn when:

- All three trajectories are identical.
- A mode produces a path with unexpectedly few points.

The grid-map-to-PGM chain must be verified as:

```text
original elevation NaN
-> final traversability NaN
-> OccupancyGrid -1
-> PGM gray 205
```

The first verification command for the static planner remains:

```bash
python -m src.main --config config.yaml
```

Manual checks should include `output/overlay.png`, all geo CSVs, and all ENU CSVs.
