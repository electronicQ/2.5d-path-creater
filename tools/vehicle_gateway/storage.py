"""Transactional PCD and ENU metadata storage with resumable uploads."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from .validation import (
    safe_basename,
    sha256_file,
    validate_geo_point,
    validate_identifier,
    validate_pcd_header,
    validate_sha256,
)


@dataclass
class UploadInfo:
    map_id: str
    pcd_name: str
    file_size: int
    sha256: str
    enu_origin: Dict[str, float]
    offset: int


class GatewayPaths:
    def __init__(self, project_root: Path, data_root: Optional[Path] = None) -> None:
        self.project_root = Path(project_root).resolve()
        self.data_root = (Path(data_root) if data_root else self.project_root / "var" / "vehicle_gateway").resolve()
        self.jobs = self.data_root / "jobs"
        self.uploads = self.data_root / "uploads"
        self.pcd_dir = self.project_root / "map" / "pcd"
        self.pgm_dir = self.project_root / "map" / "pgm"
        self.path_config = self.project_root / "tools" / "path_creater" / "config" / "config.yaml"
        self.meta_path = self.project_root / "tools" / "path_creater" / "config" / "map_enu.meta.yaml"

    def ensure(self) -> None:
        for path in (self.jobs, self.uploads, self.pcd_dir, self.pgm_dir, self.path_config.parent):
            path.mkdir(parents=True, exist_ok=True)


class UploadStore:
    def __init__(self, paths: GatewayPaths) -> None:
        self.paths = paths
        self.paths.ensure()

    def _key(self, map_id: str, digest: str) -> str:
        return "{}-{}".format(validate_identifier(map_id, "map_id"), validate_sha256(digest))

    def begin(self, payload: Dict[str, Any]) -> UploadInfo:
        map_id = validate_identifier(payload.get("map_id"), "map_id")
        pcd_name = safe_basename(payload.get("pcd_name", "scans_Global_ENU.pcd"), ".pcd")
        try:
            file_size = int(payload["file_size"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("file_size must be an integer") from exc
        if file_size <= 0 or file_size > 16 * 1024 * 1024 * 1024:
            raise ValueError("file_size is out of range")
        digest = validate_sha256(payload.get("sha256"))
        origin = validate_geo_point(payload.get("enu_origin"), "enu_origin")
        key = self._key(map_id, digest)
        part = self.paths.uploads / (key + ".part")
        record = self.paths.uploads / (key + ".json")
        if record.exists():
            stored = json.loads(record.read_text(encoding="utf-8"))
            if stored != {
                "map_id": map_id,
                "pcd_name": pcd_name,
                "file_size": file_size,
                "sha256": digest,
                "enu_origin": origin,
            }:
                raise ValueError("existing resumable upload metadata does not match")
        else:
            record.write_text(
                json.dumps(
                    {
                        "map_id": map_id,
                        "pcd_name": pcd_name,
                        "file_size": file_size,
                        "sha256": digest,
                        "enu_origin": origin,
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        offset = part.stat().st_size if part.exists() else 0
        if offset > file_size:
            part.unlink()
            offset = 0
        return UploadInfo(map_id, pcd_name, file_size, digest, origin, offset)

    def append_chunk(self, info: UploadInfo, offset: int, chunk: bytes, chunk_sha256: Optional[str] = None) -> int:
        if offset != info.offset:
            raise ValueError("unexpected chunk offset; resume from {}".format(info.offset))
        if not chunk:
            raise ValueError("empty upload chunk")
        if chunk_sha256 and sha256_bytes(chunk) != validate_sha256(chunk_sha256):
            raise ValueError("chunk checksum mismatch")
        if offset + len(chunk) > info.file_size:
            raise ValueError("chunk exceeds declared file size")
        part = self.paths.uploads / (self._key(info.map_id, info.sha256) + ".part")
        with part.open("ab") as stream:
            stream.write(chunk)
            stream.flush()
            os.fsync(stream.fileno())
        info.offset += len(chunk)
        return info.offset

    def commit(self, info: UploadInfo) -> Dict[str, Any]:
        if info.offset != info.file_size:
            raise ValueError("upload is incomplete: {}/{} bytes".format(info.offset, info.file_size))
        key = self._key(info.map_id, info.sha256)
        part = self.paths.uploads / (key + ".part")
        if sha256_file(part) != info.sha256:
            raise ValueError("checksum mismatch")
        validate_pcd_header(part)

        metadata = {"enu_origin": info.enu_origin}
        meta_tmp = self.paths.meta_path.with_name(self.paths.meta_path.name + ".tmp")
        with meta_tmp.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())

        active_pcd = self.paths.pcd_dir / info.pcd_name
        old_pcd = active_pcd.with_name(active_pcd.name + ".previous")
        old_meta = self.paths.meta_path.with_name(self.paths.meta_path.name + ".previous")
        try:
            if active_pcd.exists():
                os.replace(active_pcd, old_pcd)
            if self.paths.meta_path.exists():
                os.replace(self.paths.meta_path, old_meta)
            os.replace(part, active_pcd)
            os.replace(meta_tmp, self.paths.meta_path)
        except Exception:
            if active_pcd.exists() and not part.exists():
                os.replace(active_pcd, part)
            if old_pcd.exists() and not active_pcd.exists():
                os.replace(old_pcd, active_pcd)
            if old_meta.exists() and not self.paths.meta_path.exists():
                os.replace(old_meta, self.paths.meta_path)
            if meta_tmp.exists():
                meta_tmp.unlink()
            raise
        for backup in (old_pcd, old_meta):
            if backup.exists():
                backup.unlink()
        record = self.paths.uploads / (key + ".json")
        if record.exists():
            record.unlink()
        result = {
            "map_id": info.map_id,
            "pcd_name": info.pcd_name,
            "pcd_path": str(active_pcd.relative_to(self.paths.project_root)),
            "meta_path": str(self.paths.meta_path.relative_to(self.paths.project_root)),
            "sha256": info.sha256,
            "file_size": info.file_size,
        }
        manifest = self.paths.data_root / "active_map.json"
        temp_manifest = manifest.with_name(manifest.name + ".tmp")
        temp_manifest.write_text(json.dumps(result, sort_keys=True), encoding="utf-8")
        os.replace(temp_manifest, manifest)
        return result


def sha256_bytes(data: bytes) -> str:
    import hashlib

    return hashlib.sha256(data).hexdigest()
