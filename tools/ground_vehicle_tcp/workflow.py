"""Serial navigation request workflow for the ground-vehicle binary protocol."""

from __future__ import annotations

import csv
import math
import os
import shutil
import struct
import tempfile
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Optional, Type

import yaml

from tools.path_creater.geo import GeoPoint, local_utm_to_geodetic
from tools.path_creater.map_metadata import load_map_metadata
from .pipeline import PipelineError, PipelineRunner

from .map_cache import MapCache
from .protocol import (
    ERROR_INTERNAL,
    ERROR_MAP_CONVERSION,
    ERROR_NO_PATH,
    ERROR_PLANNING_TIMEOUT,
    ERROR_START_NOT_PLANNABLE,
    ERROR_TARGET_OUTSIDE_MAP,
    NavigationRequest,
)


class NavigationWorkflow:
    def __init__(
        self,
        project_root: Path,
        data_root: Path,
        pcd_path: Path,
        pcd_name: str,
        metadata_path: Path,
        planner_config_path: Path,
        conversion_timeout_s: int = 1800,
        runner_factory: Type[PipelineRunner] = PipelineRunner,
        map_cache: Optional[MapCache] = None,
    ) -> None:
        self.project_root = Path(project_root).resolve()
        self.data_root = Path(data_root).resolve()
        self.pcd_path = Path(pcd_path).resolve()
        self.pcd_name = pcd_name
        self.metadata_path = Path(metadata_path).resolve()
        self.planner_config_path = Path(planner_config_path).resolve()
        self.runner_factory = runner_factory
        self.map_cache = map_cache or MapCache(
            self.data_root,
            self.project_root,
            conversion_timeout_s=conversion_timeout_s,
            runner_factory=runner_factory,
        )

    def execute(self, request: NavigationRequest) -> int:
        job_id = uuid.uuid4().hex
        job_dir = self.data_root / "jobs" / job_id
        job_dir.mkdir(parents=True, exist_ok=False)
        log_path = job_dir / "job.log"
        try:
            metadata = load_map_metadata(self.metadata_path)
            map_yaml = self.map_cache.ensure_map(self.pcd_path, self.pcd_name, metadata.enu_origin, log_path)
        except (OSError, ValueError, PipelineError) as exc:
            return self._map_error(exc)

        try:
            start_geo = self._to_geodetic(request.start_xy_m, metadata.enu_origin)
            goal_geo = self._to_geodetic(request.goal_xy_m, metadata.enu_origin)
            with self.planner_config_path.open("r", encoding="utf-8") as stream:
                base_config = yaml.safe_load(stream) or {}
            config = dict(base_config)
            config["start"] = self._point_dict(start_geo)
            config["goal"] = self._point_dict(goal_geo)
            config["map_yaml"] = str(map_yaml.resolve())
            snapshot_meta = job_dir / "map_enu.meta.yaml"
            with snapshot_meta.open("w", encoding="utf-8") as stream:
                yaml.safe_dump({
                    "enu_origin": self._point_dict(metadata.enu_origin),
                }, stream, sort_keys=False)
            config["map_meta"] = str(snapshot_meta.resolve())
            config["output_dir"] = str((job_dir / "output").resolve())
            config["write_overlay"] = True
            config_path = job_dir / "planner-config.yaml"
            with config_path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)
            runner = self.runner_factory(self.project_root, timeout_s=request.timeout_s)
            runner.plan(config_path, job_dir / "output", log_path)
            self._validate_outputs(job_dir / "output")
            self._publish_outputs(job_dir / "output", self._configured_output_dir(base_config))
            return 0
        except PipelineError as exc:
            return self._planning_error(exc, log_path)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            return self._planning_error(exc, log_path)

    @staticmethod
    def _to_geodetic(xy_m, origin: GeoPoint) -> GeoPoint:
        # The protocol's yaw fields are intentionally ignored by the planner.
        return local_utm_to_geodetic(xy_m[0], xy_m[1], origin)

    @staticmethod
    def _point_dict(point: GeoPoint) -> Dict[str, float]:
        return {
            "latitude": point.latitude,
            "longitude": point.longitude,
            "altitude": point.altitude,
        }

    @staticmethod
    def _validate_outputs(output_dir: Path) -> None:
        csv_fields = {
            "traj_distance_geo.csv": ("latitude", "longitude"),
            "traj_smooth_geo.csv": ("latitude", "longitude"),
            "traj_cost_aware_geo.csv": ("latitude", "longitude"),
            "traj_distance_enu.csv": ("x", "y"),
            "traj_smooth_enu.csv": ("x", "y"),
            "traj_cost_aware_enu.csv": ("x", "y"),
        }
        csv_names = tuple(csv_fields)
        xml_names = ("road_map1.xml", "road_map2.xml", "road_map3.xml")
        for name in csv_names + xml_names:
            path = output_dir / name
            if not path.is_file() or path.stat().st_size == 0:
                raise PipelineError("OUTPUT_INVALID", "missing or empty trajectory artifact: {}".format(name))
        for name, fields in csv_fields.items():
            try:
                with (output_dir / name).open("r", encoding="utf-8", newline="") as stream:
                    rows = list(csv.reader(stream))
                if not rows or tuple(rows[0]) != fields or len(rows) < 2:
                    raise ValueError("missing expected header or trajectory points")
                for row in rows[1:]:
                    if len(row) != len(fields) or not all(math.isfinite(float(value)) for value in row):
                        raise ValueError("invalid numeric row")
            except (OSError, UnicodeError, ValueError, csv.Error) as exc:
                raise PipelineError("OUTPUT_INVALID", "invalid trajectory CSV {}: {}".format(name, exc)) from exc
        for name in xml_names:
            try:
                root = ET.parse(str(output_dir / name)).getroot()
                vertices = root.findall("vertex") if root.tag == "map" else []
                if not vertices:
                    raise ValueError("no vertices")
                for vertex in vertices:
                    for field in ("x", "y", "yaw"):
                        if not math.isfinite(float(vertex.attrib[field])):
                            raise ValueError("non-finite {} coordinate".format(field))
            except (OSError, ValueError, KeyError, ET.ParseError) as exc:
                raise PipelineError("OUTPUT_INVALID", "invalid trajectory XML {}: {}".format(name, exc)) from exc
        overlay_path = output_dir / "overlay.png"
        try:
            with overlay_path.open("rb") as stream:
                header = stream.read(24)
            if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
                raise ValueError("invalid PNG header")
            width, height = struct.unpack(">II", header[16:24])
            if width == 0 or height == 0:
                raise ValueError("zero-sized PNG")
        except (OSError, ValueError, struct.error) as exc:
            raise PipelineError("OUTPUT_INVALID", "invalid map overlay: {}".format(exc)) from exc

    def _configured_output_dir(self, config: Dict) -> Path:
        value = Path(str(config.get("output_dir", "../../../output")))
        if not value.is_absolute():
            value = self.planner_config_path.parent / value
        return value.resolve()

    @staticmethod
    def _publish_outputs(source_dir: Path, destination_dir: Path) -> None:
        names = (
            "traj_distance_geo.csv", "traj_distance_enu.csv", "traj_smooth_geo.csv",
            "traj_smooth_enu.csv", "traj_cost_aware_geo.csv", "traj_cost_aware_enu.csv",
            "road_map1.xml", "road_map2.xml", "road_map3.xml", "overlay.png",
        )
        destination_dir.mkdir(parents=True, exist_ok=True)
        for name in names:
            source = source_dir / name
            descriptor, temporary_name = tempfile.mkstemp(
                prefix="." + name + ".", suffix=".tmp", dir=str(destination_dir)
            )
            try:
                with os.fdopen(descriptor, "wb") as target, source.open("rb") as incoming:
                    shutil.copyfileobj(incoming, target, length=1024 * 1024)
                    target.flush()
                    os.fsync(target.fileno())
                os.replace(temporary_name, destination_dir / name)
            except Exception:
                try:
                    os.unlink(temporary_name)
                except OSError:
                    pass
                raise

    @staticmethod
    def _map_error(exc: Exception) -> int:
        return ERROR_MAP_CONVERSION

    @staticmethod
    def _planning_error(exc: Exception, log_path: Optional[Path] = None) -> int:
        detail = str(exc)
        if log_path is not None:
            try:
                detail += "\n" + log_path.read_text(encoding="utf-8", errors="replace")[-16000:]
            except OSError:
                pass
        lowered = detail.lower()
        if isinstance(exc, PipelineError):
            if exc.code == "JOB_TIMEOUT":
                return ERROR_PLANNING_TIMEOUT
            if exc.code == "OUTPUT_INVALID":
                return ERROR_INTERNAL
        if "configured start outside map" in lowered or (
            "configured start" in lowered and ("non-free" in lowered or "obstacle" in lowered)
        ):
            return ERROR_START_NOT_PLANNABLE
        if "configured goal outside map" in lowered:
            return ERROR_TARGET_OUTSIDE_MAP
        if "configured goal" in lowered and ("non-free" in lowered or "obstacle" in lowered):
            return ERROR_NO_PATH
        if "no path" in lowered:
            return ERROR_NO_PATH
        return ERROR_INTERNAL


__all__ = ["NavigationWorkflow"]
