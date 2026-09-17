"""Synchronous TCP client used from QThread workers."""

from __future__ import annotations

import hashlib
import socket
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple

from tools.vehicle_protocol import DEFAULT_MAX_FRAME_SIZE, ProtocolError, encode_frame, read_frame

from .transfer import CHUNK_SIZE, resumable_file_state, sha256_file, verify_download


class GatewayClientError(RuntimeError):
    pass


class GatewayClient:
    def __init__(self, host: str, port: int = 19090, token: str = "", timeout: float = 15.0) -> None:
        self.host, self.port, self.token, self.timeout = host, int(port), token, timeout
        self.sock: Optional[socket.socket] = None
        self.events = []

    def connect(self) -> Dict[str, Any]:
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        response = self.request({"type": "hello", "token": self.token})
        if response[0].get("status") != "succeeded":
            raise GatewayClientError(response[0].get("message", "authentication failed"))
        return response[0]

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def request(self, header: Dict[str, Any], payload: bytes = b"") -> Tuple[Dict[str, Any], bytes]:
        if not self.sock:
            raise GatewayClientError("not connected")
        request = dict(header)
        request_id = request.setdefault("request_id", str(uuid.uuid4()))
        self.sock.sendall(encode_frame(request, payload))
        while True:
            response, response_payload = self._read_frame()
            if response.get("type") == "job.status" and response.get("request_id") != request_id:
                self.events.append(response)
                continue
            if response.get("request_id") not in (None, request_id):
                self.events.append(response)
                continue
            break
        if response.get("status") == "failed":
            raise GatewayClientError("{}: {}".format(response.get("error_code", "FAILED"), response.get("message", "request failed")))
        return response, response_payload

    def _read_frame(self) -> Tuple[Dict[str, Any], bytes]:
        assert self.sock is not None
        prefix = self._recv_exact(4)
        length = int.from_bytes(prefix, "big")
        if length < 4 or length > DEFAULT_MAX_FRAME_SIZE:
            raise ProtocolError("invalid response length")
        raw = prefix + self._recv_exact(length)
        from tools.vehicle_protocol import decode_frame
        return decode_frame(raw)

    def _recv_exact(self, size: int) -> bytes:
        assert self.sock is not None
        chunks = []
        remaining = size
        while remaining:
            data = self.sock.recv(remaining)
            if not data:
                raise GatewayClientError("connection closed by vehicle")
            chunks.append(data)
            remaining -= len(data)
        return b"".join(chunks)

    def upload_pcd(self, path: Path, map_id: str, pcd_name: str, enu_origin: Dict[str, float], progress: Optional[Callable[[int, int], None]] = None) -> Dict[str, Any]:
        path = Path(path)
        total = path.stat().st_size
        digest = sha256_file(path)
        begin, _ = self.request({"type": "map.upload.begin", "map_id": map_id, "job_id": "upload-" + map_id, "pcd_name": pcd_name, "file_size": total, "sha256": digest, "enu_origin": enu_origin})
        offset = int(begin.get("next_offset", 0))
        with path.open("rb") as stream:
            stream.seek(offset)
            while offset < total:
                payload = stream.read(CHUNK_SIZE)
                if not payload:
                    break
                response, _ = self.request({"type": "map.upload.chunk", "job_id": "upload-" + map_id, "offset": offset, "sha256": hashlib.sha256(payload).hexdigest()}, payload)
                offset = int(response["next_offset"])
                if progress:
                    progress(offset, total)
        response, _ = self.request({"type": "map.upload.commit", "job_id": "upload-" + map_id})
        return response

    def start_conversion(self, map_id: str, pcd_name: str, pgm_name: str, job_id: str) -> Dict[str, Any]:
        return self.request({"type": "map.convert.start", "job_id": job_id, "map_id": map_id, "pcd_name": pcd_name, "pgm_name": pgm_name})[0]

    def start_planning(self, request: Dict[str, Any]) -> Dict[str, Any]:
        return self.request(request)[0]

    def status(self, job_id: str) -> Dict[str, Any]:
        return self.request({"type": "job.status", "job_id": job_id})[0]

    def artifacts(self, job_id: str) -> Dict[str, Any]:
        return self.request({"type": "artifact.list", "job_id": job_id})[0]

    def download_artifact(self, job_id: str, artifact: str, destination: Path, expected_size: int, expected_sha256: str, progress: Optional[Callable[[int, int], None]] = None) -> Path:
        destination = Path(destination)
        part = destination.with_name(destination.name + ".part")
        offset = resumable_file_state(part, expected_size)
        with part.open("ab") as stream:
            while offset < expected_size:
                response, payload = self.request({"type": "artifact.download", "job_id": job_id, "artifact": artifact, "offset": offset, "length": CHUNK_SIZE})
                stream.write(payload)
                offset = int(response["next_offset"])
                if progress:
                    progress(offset, expected_size)
        verify_download(part, expected_size, expected_sha256)
        part.replace(destination)
        return destination
