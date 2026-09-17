"""Qt worker objects that keep network and file work off the GUI thread."""

from __future__ import annotations

import json
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional

from .client import GatewayClient
from .models import PlannerRequest

try:
    from PySide6.QtCore import QObject, Signal, Slot
except ImportError:  # Allows protocol/model tests on machines without Qt.
    class QObject:  # type: ignore
        pass
    class Signal:  # type: ignore
        def __init__(self, *args: Any, **kwargs: Any) -> None: pass
    def Slot(*args: Any, **kwargs: Any):  # type: ignore
        def decorator(function: Any) -> Any: return function
        return decorator


class GatewayWorker(QObject):
    connect_requested = Signal()
    upload_requested = Signal(str, str, str, dict)
    convert_requested = Signal(str, str, str)
    plan_requested = Signal(object)
    status_requested = Signal(str)
    download_requested = Signal(str, str)
    connected = Signal(dict)
    progress = Signal(dict)
    completed = Signal(dict)
    failed = Signal(str)

    def __init__(self, client: GatewayClient) -> None:
        super().__init__()
        self.client = client
        self.connect_requested.connect(self.connect_gateway)
        self.upload_requested.connect(self.upload)
        self.convert_requested.connect(self.convert)
        self.plan_requested.connect(self.plan)
        self.status_requested.connect(self.poll_status)
        self.download_requested.connect(self.download_all)

    @Slot()
    def connect_gateway(self) -> None:
        try:
            self.connected.emit(self.client.connect())
        except Exception as exc:
            self.failed.emit(str(exc))

    @Slot(str, str, str, dict)
    def upload(self, path: str, map_id: str, pcd_name: str, origin: Dict[str, float]) -> None:
        try:
            result = self.client.upload_pcd(Path(path), map_id, pcd_name, origin, lambda done, total: self.progress.emit({"operation": "upload", "done": done, "total": total}))
            self.completed.emit(result)
        except Exception as exc:
            self.failed.emit(str(exc))

    @Slot(str, str, str)
    def convert(self, map_id: str, pcd_name: str, pgm_name: str) -> None:
        try:
            job_id = "convert-" + uuid.uuid4().hex
            result = self.client.start_conversion(map_id, pcd_name, pgm_name, job_id)
            self.completed.emit(result)
            self._wait_for_job(job_id)
        except Exception as exc:
            self.failed.emit(str(exc))

    @Slot(object)
    def plan(self, request: PlannerRequest) -> None:
        try:
            job_id = "plan-" + uuid.uuid4().hex
            result = self.client.start_planning(request.to_message(job_id))
            self.completed.emit(result)
            self._wait_for_job(job_id)
        except Exception as exc:
            self.failed.emit(str(exc))

    @Slot(str)
    def poll_status(self, job_id: str) -> None:
        try:
            self.progress.emit(self.client.status(job_id))
        except Exception as exc:
            self.failed.emit(str(exc))

    def _wait_for_job(self, job_id: str) -> None:
        deadline = time.monotonic() + 1800.0
        while time.monotonic() < deadline:
            status = self.client.status(job_id)
            self.progress.emit(status)
            if status.get("status") in ("succeeded", "failed") or status.get("state") in ("SUCCEEDED", "FAILED"):
                if status.get("status") == "failed" or status.get("state") == "FAILED":
                    self.failed.emit("{}: {}".format(status.get("error_code", "FAILED"), status.get("message", "job failed")))
                    return
                if job_id.startswith("plan-"):
                    self.download_all(job_id, Path.home() / "vehicle_ground_station" / "jobs")
                return
            time.sleep(1.0)
        self.failed.emit("JOB_TIMEOUT: job status polling timed out")

    def download_all(self, job_id: str, destination: Path) -> None:
        try:
            manifest = self.client.artifacts(job_id)
            local = Path(destination) / job_id
            local.mkdir(parents=True, exist_ok=True)
            for name, meta in (manifest.get("artifacts") or {}).items():
                target = local / name
                self.client.download_artifact(job_id, name, target, int(meta["size"]), str(meta["sha256"]), lambda done, total: self.progress.emit({"operation": "download", "artifact": name, "done": done, "total": total}))
            self.completed.emit({"job_id": job_id, "directory": str(local), "artifacts": manifest.get("artifacts", {})})
        except Exception as exc:
            self.failed.emit(str(exc))
