"""Fingerprint and cache converted PGM maps so restarts do not reconvert."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import uuid
from pathlib import Path
from typing import Any, Dict, Optional, Type

import yaml

from tools.path_creater.geo import GeoPoint
from .pipeline import PipelineError, PipelineRunner


CONVERSION_REVISION = "grid-map-pgm-v2"
CONVERSION_CONFIG = {
    "pcd_loader_launch": "grid_map_pcl/grid_map_pcl_loader_node.launch",
    "filters_launch": "grid_map_demos/filters_demo_gai.launch",
    "enable_rviz": False,
    "raw_grid_topic": "/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud",
    "traversability_topic": "/grid_map_visualization/traversability_grid",
    "saver": "tools/pgm_saver/save_continuous_map.py",
}
CONVERSION_SOURCE_FILES = (
    "src/grid_map/grid_map_pcl/config/parameters.yaml",
    "src/grid_map/grid_map_pcl/launch/grid_map_pcl_loader_node.launch",
    "src/grid_map/grid_map_demos/launch/filters_demo_gai.launch",
    "src/grid_map/grid_map_demos/config/filters_demo_gai.yaml",
    "src/grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml",
    "tools/pgm_saver/save_continuous_map.py",
)


class MapCache:
    def __init__(
        self,
        data_root: Path,
        project_root: Path,
        conversion_timeout_s: float = 1800.0,
        runner_factory: Type[PipelineRunner] = PipelineRunner,
    ) -> None:
        self.data_root = Path(data_root).resolve()
        self.project_root = Path(project_root).resolve()
        self.conversion_timeout_s = float(conversion_timeout_s)
        self.runner_factory = runner_factory
        self.maps_root = self.data_root / "maps"
        self.maps_root.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def file_sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with Path(path).open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def _origin_dict(origin: GeoPoint) -> Dict[str, float]:
        return {
            "latitude": float(origin.latitude),
            "longitude": float(origin.longitude),
            "altitude": float(origin.altitude),
        }

    def _conversion_source_hashes(self) -> Dict[str, str]:
        hashes: Dict[str, str] = {}
        for relative_path in CONVERSION_SOURCE_FILES:
            path = self.project_root / relative_path
            if not path.is_file():
                raise PipelineError("FILE_NOT_FOUND", "map conversion source not found: {}".format(path))
            hashes[relative_path] = self.file_sha256(path)
        return hashes

    def _fingerprint(self, pcd_sha256: str, pcd_name: str, origin: GeoPoint, source_hashes: Dict[str, str]) -> str:
        payload = {
            "pcd_sha256": pcd_sha256,
            "pcd_name": pcd_name,
            "enu_origin": self._origin_dict(origin),
            "conversion_revision": CONVERSION_REVISION,
            "conversion_config": CONVERSION_CONFIG,
            "conversion_source_sha256": source_hashes,
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _cached_map(
        self, directory: Path, fingerprint: str, pcd_sha256: str, pcd_name: str, origin: GeoPoint
    ) -> Optional[Path]:
        pgm = directory / "map_enu.pgm"
        map_yaml = directory / "map_enu.yaml"
        manifest_path = directory / "manifest.yaml"
        if not (pgm.is_file() and map_yaml.is_file() and manifest_path.is_file()):
            return None
        try:
            manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
            metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
            if manifest.get("fingerprint") != fingerprint or manifest.get("pcd_sha256") != pcd_sha256:
                return None
            if manifest.get("pcd_name") != pcd_name or manifest.get("conversion_revision") != CONVERSION_REVISION:
                return None
            if manifest.get("conversion_config") != CONVERSION_CONFIG:
                return None
            if manifest.get("enu_origin") != self._origin_dict(origin):
                return None
            if manifest.get("pgm_sha256") != self.file_sha256(pgm):
                return None
            if manifest.get("map_yaml_sha256") != self.file_sha256(map_yaml):
                return None
            if metadata.get("image") != pgm.name:
                return None
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            return None
        return map_yaml

    def ensure_map(self, pcd_path: Path, pcd_name: str, origin: GeoPoint, log_path: Path) -> Path:
        pcd_path = Path(pcd_path).resolve()
        if (
            not isinstance(pcd_name, str)
            or Path(pcd_name).name != pcd_name
            or pcd_name in (".", "..")
            or not pcd_name.lower().endswith(".pcd")
            or pcd_path.name != pcd_name
        ):
            raise PipelineError("OUTPUT_INVALID", "PCD name must be a matching safe basename")
        if not pcd_path.is_file():
            raise PipelineError("FILE_NOT_FOUND", "source PCD not found: {}".format(pcd_path))
        pcd_sha256 = self.file_sha256(pcd_path)
        source_hashes = self._conversion_source_hashes()
        fingerprint = self._fingerprint(pcd_sha256, pcd_name, origin, source_hashes)
        target_dir = self.maps_root / fingerprint
        cached = self._cached_map(target_dir, fingerprint, pcd_sha256, pcd_name, origin)
        if cached is not None:
            return cached

        temp_dir = Path(tempfile.mkdtemp(prefix="convert-", dir=str(self.maps_root)))
        try:
            temp_base = temp_dir / "map_enu"
            runner = self.runner_factory(self.project_root, timeout_s=self.conversion_timeout_s)
            runner.convert(pcd_name, temp_base, Path(log_path), rviz=False)
            if self.file_sha256(pcd_path) != pcd_sha256:
                raise PipelineError("OUTPUT_INVALID", "source PCD changed during map conversion")
            if self._conversion_source_hashes() != source_hashes:
                raise PipelineError("OUTPUT_INVALID", "map conversion configuration changed during conversion")
            temp_pgm = Path(str(temp_base) + ".pgm")
            temp_yaml = Path(str(temp_base) + ".yaml")
            if not temp_pgm.is_file() or temp_pgm.stat().st_size == 0 or not temp_yaml.is_file():
                raise PipelineError("OUTPUT_INVALID", "map conversion did not produce PGM and YAML")
            map_metadata = yaml.safe_load(temp_yaml.read_text(encoding="utf-8")) or {}
            if map_metadata.get("image") != temp_pgm.name:
                raise PipelineError("OUTPUT_INVALID", "converted map YAML references the wrong PGM")

            target_dir.mkdir(parents=True, exist_ok=True)
            manifest_path = target_dir / "manifest.yaml"
            if manifest_path.exists():
                manifest_path.unlink()
            final_pgm = target_dir / "map_enu.pgm"
            final_yaml = target_dir / "map_enu.yaml"
            os.replace(temp_pgm, final_pgm)
            os.replace(temp_yaml, final_yaml)
            manifest: Dict[str, Any] = {
                "fingerprint": fingerprint,
                "pcd_sha256": pcd_sha256,
                "pcd_name": pcd_name,
                "enu_origin": self._origin_dict(origin),
                "conversion_revision": CONVERSION_REVISION,
                "conversion_config": CONVERSION_CONFIG,
                "conversion_source_sha256": source_hashes,
                "pgm_sha256": self.file_sha256(final_pgm),
                "map_yaml_sha256": self.file_sha256(final_yaml),
            }
            manifest_temp = target_dir / ("manifest.{}.tmp".format(uuid.uuid4().hex))
            with manifest_temp.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(manifest, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(manifest_temp, manifest_path)
            return final_yaml
        finally:
            shutil.rmtree(str(temp_dir), ignore_errors=True)
