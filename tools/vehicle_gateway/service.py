"""Async TCP service and single-job vehicle workflow."""

from __future__ import annotations

import asyncio
import json
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, Optional

from .configuration import ConfigStore
from .protocol import ProtocolError, read_frame, write_frame
from .runner import PipelineError, PipelineRunner
from .storage import GatewayPaths, UploadInfo, UploadStore
from .validation import safe_basename, sha256_file, validate_identifier


@dataclass
class GatewayConfig:
    host: str = "0.0.0.0"
    port: int = 19090
    token: str = ""
    max_frame_size: int = 8 * 1024 * 1024
    project_root: Path = Path(".")
    data_root: Optional[Path] = None
    job_timeout_s: float = 1800.0


@dataclass
class Job:
    job_id: str
    map_id: str
    kind: str
    state: str
    artifacts: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    task: Optional[asyncio.Task] = None


StatusCallback = Callable[[Dict[str, Any]], Awaitable[None]]


class JobManager:
    def __init__(self, config: GatewayConfig) -> None:
        self.config = config
        self.paths = GatewayPaths(config.project_root, config.data_root)
        self.uploads = UploadStore(self.paths)
        self.config_store = ConfigStore(self.paths.path_config)
        self.active: Optional[Job] = None
        self.ready_map_id: Optional[str] = None
        self.active_map: Dict[str, Any] = {}
        active_manifest = self.paths.data_root / "active_map.json"
        if active_manifest.is_file():
            try:
                self.active_map = json.loads(active_manifest.read_text(encoding="utf-8"))
                self.ready_map_id = self.active_map.get("map_id")
            except (OSError, ValueError):
                self.active_map = {}
        self.upload_sessions: Dict[str, UploadInfo] = {}
        # Constructed both from the asyncio server and synchronous unit tests.
        # Create the loop-bound lock lazily on first async operation.
        self.lock: Optional[asyncio.Lock] = None
        self.statuses: Dict[str, Dict[str, Any]] = {}

    def _job_dir(self, job_id: str) -> Path:
        path = self.paths.jobs / validate_identifier(job_id, "job_id")
        path.mkdir(parents=True, exist_ok=True)
        return path

    async def upload_begin(self, request: Dict[str, Any]) -> Dict[str, Any]:
        async with self._lock():
            if self.active and self.active.task and not self.active.task.done():
                raise ValueError("another job is running")
        info = self.uploads.begin(request)
        key = str(request.get("job_id") or request.get("map_id"))
        self.upload_sessions[key] = info
        return {"status": "accepted", "state": "UPLOADING", "next_offset": info.offset, "file_size": info.file_size}

    async def upload_chunk(self, request: Dict[str, Any], payload: bytes) -> Dict[str, Any]:
        key = str(request.get("job_id") or request.get("map_id"))
        if key not in self.upload_sessions:
            raise ValueError("upload session not found")
        info = self.upload_sessions[key]
        new_offset = self.uploads.append_chunk(info, int(request.get("offset", -1)), payload, request.get("sha256"))
        return {"status": "progress", "state": "UPLOADING", "next_offset": new_offset, "progress": new_offset / float(info.file_size)}

    async def upload_commit(self, request: Dict[str, Any]) -> Dict[str, Any]:
        key = str(request.get("job_id") or request.get("map_id"))
        if key not in self.upload_sessions:
            raise ValueError("upload session not found")
        info = self.upload_sessions.pop(key)
        result = self.uploads.commit(info)
        self.ready_map_id = info.map_id
        self.active_map = result
        return {"status": "succeeded", "state": "READY", "artifacts": result}

    async def start_job(self, request: Dict[str, Any], kind: str, callback: StatusCallback) -> Dict[str, Any]:
        async with self._lock():
            if self.active and self.active.task and not self.active.task.done():
                return {"status": "failed", "error_code": "BUSY", "message": "another job is running"}
            map_id = validate_identifier(request.get("map_id"), "map_id")
            if self.ready_map_id != map_id:
                return {"status": "failed", "error_code": "FILE_NOT_FOUND", "message": "map has not been uploaded and committed"}
            job = Job(str(request.get("job_id") or uuid.uuid4()), map_id, kind, "CONVERTING" if kind == "convert" else "PLANNING")
            job.task = asyncio.create_task(self._run_job(job, request, callback))
            self.active = job
            return {"status": "accepted", "job_id": job.job_id, "state": job.state}

    async def cancel(self, request: Dict[str, Any]) -> Dict[str, Any]:
        async with self._lock():
            if not self.active or not self.active.task or self.active.task.done():
                return {"status": "failed", "error_code": "FILE_NOT_FOUND", "message": "job not found"}
            self.active.task.cancel()
            return {"status": "accepted", "job_id": self.active.job_id, "state": "CANCELLING"}

    def _lock(self) -> asyncio.Lock:
        if self.lock is None:
            self.lock = asyncio.Lock()
        return self.lock

    def list_artifacts(self, job_id: Optional[str]) -> Dict[str, Any]:
        if self.active and (not job_id or self.active.job_id == job_id):
            return {"status": "succeeded", "job_id": self.active.job_id, "state": self.active.state, "artifacts": self.active.artifacts}
        if job_id:
            manifest = self.paths.jobs / validate_identifier(job_id, "job_id") / "manifest.json"
            if manifest.is_file():
                return json.loads(manifest.read_text(encoding="utf-8"))
        return {"status": "failed", "error_code": "FILE_NOT_FOUND", "message": "job artifacts not found"}

    def job_status(self, job_id: str) -> Dict[str, Any]:
        job_id = validate_identifier(job_id, "job_id")
        if self.active and self.active.job_id == job_id:
            return {"status": "progress", "job_id": job_id, "state": self.active.state, "artifacts": self.active.artifacts}
        if job_id in self.statuses:
            return dict(self.statuses[job_id])
        manifest = self.paths.jobs / job_id / "manifest.json"
        if manifest.is_file():
            return json.loads(manifest.read_text(encoding="utf-8"))
        raise ValueError("job not found")

    def _manifest(self, job_id: str) -> Dict[str, Any]:
        path = self.paths.jobs / validate_identifier(job_id, "job_id") / "manifest.json"
        if not path.is_file():
            raise ValueError("job manifest not found")
        return json.loads(path.read_text(encoding="utf-8"))

    def download_artifact(self, job_id: str, artifact: str, offset: int, length: int) -> Any:
        manifest = self._manifest(job_id)
        artifacts = manifest.get("artifacts") or {}
        if artifact not in artifacts or not isinstance(artifacts[artifact], dict):
            raise ValueError("artifact is not listed in job manifest")
        if offset < 0 or length <= 0:
            raise ValueError("invalid artifact range")
        path_value = artifacts[artifact].get("path")
        if not isinstance(path_value, str):
            raise ValueError("artifact path is invalid")
        path = (self.paths.project_root / path_value).resolve()
        job_root = (self.paths.jobs / validate_identifier(job_id, "job_id")).resolve()
        if job_root not in path.parents or not path.is_file():
            raise ValueError("artifact path is invalid")
        size = path.stat().st_size
        if offset > size:
            raise ValueError("artifact offset is beyond end of file")
        with path.open("rb") as stream:
            stream.seek(offset)
            payload = stream.read(min(length, size - offset))
        return {
            "status": "progress" if offset + len(payload) < size else "succeeded",
            "job_id": job_id,
            "artifact": artifact,
            "offset": offset,
            "next_offset": offset + len(payload),
            "file_size": size,
            "sha256": artifacts[artifact].get("sha256") or sha256_file(path),
        }, payload

    async def _emit(self, job: Job, callback: StatusCallback, state: str, **fields: Any) -> None:
        self.statuses[job.job_id] = {"status": "progress", "job_id": job.job_id, "state": state, **fields}
        await callback({"status": "progress", "job_id": job.job_id, "state": state, **fields})

    async def _run_job(self, job: Job, request: Dict[str, Any], callback: StatusCallback) -> None:
        job_dir = self._job_dir(job.job_id)
        log_path = job_dir / "job.log"
        (job_dir / "request.json").write_text(json.dumps(request, ensure_ascii=False, indent=2), encoding="utf-8")
        runner = PipelineRunner(self.paths.project_root, timeout_s=self.config.job_timeout_s)
        try:
            if job.kind == "convert":
                pcd_name = safe_basename(request.get("pcd_name", "scans_Global_ENU.pcd"), ".pcd")
                if not (self.paths.pcd_dir / pcd_name).is_file():
                    raise PipelineError("FILE_NOT_FOUND", "PCD file is not committed: {}".format(pcd_name))
                pgm_name = safe_basename(request.get("pgm_name"), ".pgm") if str(request.get("pgm_name", "")).lower().endswith(".pgm") else safe_basename(request.get("pgm_name"))
                pgm_name = pgm_name[:-4] if pgm_name.lower().endswith(".pgm") else pgm_name
                temp_base = job_dir / "map" / pgm_name
                await self._emit(job, callback, "CONVERTING", message="starting ROS map conversion")
                await asyncio.to_thread(runner.convert, pcd_name, temp_base, log_path)
                final_base = self.paths.pgm_dir / pgm_name
                final_base.parent.mkdir(parents=True, exist_ok=True)
                for suffix in (".pgm", ".yaml"):
                    source = Path(str(temp_base) + suffix)
                    target = Path(str(final_base) + suffix)
                    source.replace(target)
                    job.artifacts[suffix[1:]] = self._artifact(target)
                await self._emit(job, callback, "MAP_READY", progress=1.0, artifacts=job.artifacts)
            else:
                pgm_name = safe_basename(request.get("pgm_name"), ".pgm") if str(request.get("pgm_name", "")).lower().endswith(".pgm") else safe_basename(request.get("pgm_name"))
                pgm_name = pgm_name[:-4] if pgm_name.lower().endswith(".pgm") else pgm_name
                if not (self.paths.pgm_dir / (pgm_name + ".yaml")).is_file():
                    raise PipelineError("FILE_NOT_FOUND", "PGM map is not available: {}".format(pgm_name))
                output_dir = job_dir / "output"
                self.config_store.update(request, request.get("pgm_name"), str(output_dir.resolve()))
                await self._emit(job, callback, "PLANNING", message="running path planner")
                result = await asyncio.to_thread(runner.plan, self.paths.path_config, output_dir, log_path)
                job.artifacts = {name: self._artifact(Path(path)) for name, path in result.items()}
            manifest = {"job_id": job.job_id, "map_id": job.map_id, "state": "SUCCEEDED", "artifacts": job.artifacts}
            (job_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
            self.statuses[job.job_id] = {"status": "succeeded", **manifest}
            await callback({"status": "succeeded", "job_id": job.job_id, "state": "SUCCEEDED", "progress": 1.0, "artifacts": job.artifacts})
        except asyncio.CancelledError:
            runner.cleanup()
            self.statuses[job.job_id] = {"status": "failed", "job_id": job.job_id, "state": "FAILED", "error_code": "JOB_CANCELLED", "message": "job cancelled"}
            await callback({"status": "failed", "job_id": job.job_id, "state": "FAILED", "error_code": "JOB_CANCELLED", "message": "job cancelled"})
        except (PipelineError, ValueError, OSError) as exc:
            runner.cleanup()
            self.statuses[job.job_id] = {"status": "failed", "job_id": job.job_id, "state": "FAILED", "error_code": getattr(exc, "code", "INVALID_ARGUMENT"), "message": str(exc)}
            await callback({"status": "failed", "job_id": job.job_id, "state": "FAILED", "error_code": getattr(exc, "code", "INVALID_ARGUMENT"), "message": str(exc)})
        finally:
            if self.active is job:
                self.active = None

    def _artifact(self, path: Path) -> Dict[str, Any]:
        return {"path": str(path.resolve().relative_to(self.paths.project_root)), "size": path.stat().st_size, "sha256": sha256_file(path)}


class GatewayServer:
    def __init__(self, config: GatewayConfig) -> None:
        self.config = config
        self.manager = JobManager(config)

    async def serve(self) -> asyncio.AbstractServer:
        return await asyncio.start_server(self._handle_client, self.config.host, self.config.port)

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        write_lock = asyncio.Lock()
        authenticated = not bool(self.config.token)

        async def send(header: Dict[str, Any], payload: bytes = b"") -> None:
            async with write_lock:
                await write_frame(writer, header, payload, self.config.max_frame_size)

        async def status_callback(status: Dict[str, Any]) -> None:
            try:
                await send({"type": "job.status", **status})
            except (ConnectionError, OSError):
                # The job must continue and remain queryable after a Wi-Fi drop.
                return

        try:
            while True:
                try:
                    request, payload = await read_frame(reader, self.config.max_frame_size)
                except asyncio.IncompleteReadError:
                    break
                except ProtocolError as exc:
                    await send({"type": "error", "status": "failed", "error_code": "INVALID_ARGUMENT", "message": str(exc)})
                    break
                request_id = request.get("request_id")
                response_payload = b""
                try:
                    message_type = request.get("type")
                    if message_type == "hello":
                        if self.config.token and request.get("token") != self.config.token:
                            raise PermissionError("authentication failed")
                        authenticated = True
                        response = {"status": "succeeded", "server": "vehicle_gateway", "protocol": 1}
                    elif not authenticated:
                        raise PermissionError("hello is required")
                    elif message_type == "map.upload.begin":
                        response = await self.manager.upload_begin(request)
                    elif message_type == "map.upload.chunk":
                        response = await self.manager.upload_chunk(request, payload)
                    elif message_type == "map.upload.commit":
                        response = await self.manager.upload_commit(request)
                    elif message_type == "map.convert.start":
                        response = await self.manager.start_job(request, "convert", status_callback)
                    elif message_type == "plan.start":
                        response = await self.manager.start_job(request, "plan", status_callback)
                    elif message_type == "job.cancel":
                        response = await self.manager.cancel(request)
                    elif message_type == "artifact.list":
                        response = self.manager.list_artifacts(request.get("job_id"))
                    elif message_type == "job.status":
                        response = self.manager.job_status(request.get("job_id"))
                    elif message_type == "artifact.download":
                        response, response_payload = self.manager.download_artifact(
                            request.get("job_id"), request.get("artifact"), int(request.get("offset", 0)), int(request.get("length", 4 * 1024 * 1024))
                        )
                    else:
                        raise ValueError("unknown message type")
                    await send({"type": str(message_type) + ".reply", "request_id": request_id, **response}, response_payload)
                except PermissionError as exc:
                    await send({"type": "error", "request_id": request_id, "status": "failed", "error_code": "UNAUTHORIZED", "message": str(exc)})
                except (ValueError, OSError) as exc:
                    message = str(exc)
                    error_code = "CHECKSUM_MISMATCH" if "checksum" in message.lower() else "INVALID_ARGUMENT"
                    await send({"type": "error", "request_id": request_id, "status": "failed", "error_code": error_code, "message": message})
        finally:
            writer.close()
            await writer.wait_closed()


def load_gateway_config(path: Path) -> GatewayConfig:
    import yaml

    values = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    config_path = Path(path).resolve()
    project_root = Path(values.get("project_root", config_path.parents[2]))
    if not project_root.is_absolute():
        project_root = (config_path.parent / project_root).resolve()
    data_root = Path(values["data_root"]) if values.get("data_root") else None
    if data_root is not None and not data_root.is_absolute():
        data_root = (project_root / data_root).resolve()
    return GatewayConfig(
        host=str(values.get("host", "0.0.0.0")),
        port=int(values.get("port", 19090)),
        token=str(values.get("token", "")),
        max_frame_size=int(values.get("max_frame_size", 8 * 1024 * 1024)),
        project_root=project_root,
        data_root=data_root,
        job_timeout_s=float(values.get("job_timeout_s", 1800.0)),
    )
