# Ground Vehicle Navigation TCP Service

This is the vehicle-side TCP service for the fixed binary protocol in `docs/TCP通信协议/通信协议.xls`. The external ground-vehicle client sends local UTM coordinates and receives one final 8-byte status frame.

## Request and response frames

All multi-byte values use big-endian byte order. TCP is a byte stream, so the receiver first reads the 5-byte fixed header, checks the `length` byte, and then reads exactly the remaining bytes. A `recv()` call is not assumed to return one complete frame.

Request (`0x20`, 32 bytes):

| Offset | Size | Field | Encoding |
| ---: | ---: | --- | --- |
| 0 | 2 | Request magic `0xFFDA` | unsigned |
| 2 | 2 | Message type `0x0001` (plan navigation route) | unsigned |
| 4 | 1 | Total frame length `0x20` | unsigned |
| 5 | 4 | Start local UTM `x` | signed int32, millimeters |
| 9 | 4 | Start local UTM `y` | signed int32, millimeters |
| 13 | 4 | Start `yaw` | signed int32, milliradians; currently ignored |
| 17 | 4 | Goal local UTM `x` | signed int32, millimeters |
| 21 | 4 | Goal local UTM `y` | signed int32, millimeters |
| 25 | 4 | Goal `yaw` | signed int32, milliradians; currently ignored |
| 29 | 2 | Planning timeout | unsigned seconds |
| 31 | 1 | CRC8 | over bytes 0 through 30 |

Response (`0x08`, 8 bytes):

| Offset | Size | Field | Encoding |
| ---: | ---: | --- | --- |
| 0 | 2 | Response magic `0xFFDB` | unsigned |
| 2 | 2 | Message type `0x0001` | unsigned |
| 4 | 1 | Total frame length `0x08` | unsigned |
| 5 | 1 | Result | `0` failure, `1` success |
| 6 | 1 | Error code | `0` on success |
| 7 | 1 | CRC8 | over bytes 0 through 6 |

CRC8 uses the spreadsheet's CRC-8 algorithm: initial value `0x00`, polynomial `0x07`, no reflection, no final XOR. This service calculates it over every preceding byte, excluding the CRC byte itself.

Error codes:

| Code | Meaning |
| ---: | --- |
| `0x00` | Success |
| `0x01` | Goal lies outside the map |
| `0x02` | Goal is not traversable or no route can be found |
| `0x03` | Start lies outside the map or is not traversable |
| `0x04` | Map conversion failed |
| `0x05` | Planning phase timed out |
| `0x06` | Invalid frame, unsupported type, or CRC error |
| `0x07` | Internal/output error or another request is already running |

The frame format has no job ID or intermediate progress messages. A client should send one request and wait for its final response before sending another. The server rejects overlapping work with `0x07`.

## Coordinates and planning

Request `x/y` are signed offsets in meters (encoded as millimeters) in the local UTM projection whose origin is the UTM projection of `map_enu.meta.yaml`'s `enu_origin`. These values are not ENU values. The server adds the offsets to the origin's UTM easting/northing, inverse-projects the result to WGS84, and passes that geodetic point through the planner's existing WGS84-to-ENU conversion. This avoids mixing UTM and ENU coordinates. Request start/goal yaw are decoded and range-checked but ignored; trajectory yaw output continues to be computed from the direction between neighboring trajectory points by the path creator.

The `timeout` field limits only the planner subprocess. Map conversion has its own `conversion_timeout_s` in `config.yaml`, so the client response wait should allow for both conversion and planning (plus startup overhead).

The service handles one conversion/planning workflow at a time. On its first request it converts the configured PCD into a PGM/YAML map. Successful conversion is cached under `var/ground_vehicle_tcp/maps/` with a fingerprint including the PCD SHA-256, PCD name, ENU origin, conversion revision/configuration, and hashes of both output files. On each request, it verifies the cache manifest and artifact hashes. A matching cache survives service restarts; changes to PCD, origin, conversion revision/configuration, or cached outputs force reconversion. The planner runs with a request-specific config and writes this request's outputs to a unique directory under `var/ground_vehicle_tcp/jobs/`. After all required files pass validation, each of the nine trajectory files is atomically promoted to the planner's configured `output_dir` (default: project-root `output/`). Wait for the success response before consuming these standard filenames; the request itself does not carry a job ID or artifact paths.

A success response is sent only after these nine trajectory files exist and are non-empty and all three XML files parse with valid vertex coordinates:

```text
traj_distance_geo.csv
traj_distance_enu.csv
traj_smooth_geo.csv
traj_smooth_enu.csv
traj_cost_aware_geo.csv
traj_cost_aware_enu.csv
road_map1.xml
road_map2.xml
road_map3.xml
```

## Run on the vehicle

Adjust `tools/ground_vehicle_tcp/config.yaml` if the project paths, PCD, listener address/port, or conversion timeout differ. The default listener is `0.0.0.0:8079`. This protocol has no authentication, so restrict port 8079 to the trusted vehicle/ground-client network using the vehicle firewall or a dedicated Wi-Fi/VLAN.

Run manually from the project root after ROS has been sourced:

```bash
source devel/setup.bash
python3 -m tools.ground_vehicle_tcp.main --config tools/ground_vehicle_tcp/config.yaml
```

For systemd deployment:

```bash
sudo cp tools/ground_vehicle_tcp/ground-vehicle-tcp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ground-vehicle-tcp.service
```

Check status and logs with:

```bash
sudo systemctl status ground-vehicle-tcp.service
sudo journalctl -u ground-vehicle-tcp.service -f
```

## Same-host verification client

Once the service is running, send a request from another terminal on the vehicle. Coordinates are entered in meters/radians and encoded by the client to millimeters/milliradians:

```bash
python3 -m tools.ground_vehicle_tcp.client \
  --host 127.0.0.1 --port 8079 \
  --start-x 0 --start-y 0 --start-yaw 0 \
  --goal-x 10 --goal-y 5 --goal-yaw 0 \
  --timeout 120 --response-timeout 2400
```

The example coordinates are illustrative and must lie in the configured map's traversable area. The client prints the result and error code; it exits with status 0 on success and 1 on a reported planning failure.
