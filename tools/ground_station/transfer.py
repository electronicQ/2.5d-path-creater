"""File hashing and resumable local transfer helpers."""

from __future__ import annotations

import hashlib
from pathlib import Path


CHUNK_SIZE = 4 * 1024 * 1024


def sha256_file(path: Path, chunk_size: int = CHUNK_SIZE) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while True:
            data = stream.read(chunk_size)
            if not data:
                break
            digest.update(data)
    return digest.hexdigest()


def resumable_file_state(part_path: Path, expected_size: int) -> int:
    size = Path(part_path).stat().st_size if Path(part_path).is_file() else 0
    return min(size, expected_size)


def verify_download(path: Path, expected_size: int, expected_sha256: str) -> None:
    path = Path(path)
    if not path.is_file() or path.stat().st_size != expected_size:
        raise ValueError("downloaded file size mismatch")
    if sha256_file(path) != expected_sha256.lower():
        raise ValueError("downloaded file checksum mismatch")
