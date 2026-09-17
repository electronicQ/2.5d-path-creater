"""Validation helpers shared by the TCP service and storage layer."""

from __future__ import annotations

import hashlib
import math
import re
from pathlib import Path
from typing import Any, Dict


_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def safe_basename(value: Any, suffix: str = "") -> str:
    name = str(value or "")
    if not name or Path(name).name != name or name in (".", "..") or not _SAFE_NAME.fullmatch(name):
        raise ValueError("unsafe file name")
    if suffix and not name.lower().endswith(suffix.lower()):
        raise ValueError("file has an invalid suffix")
    return name


def validate_identifier(value: Any, label: str = "identifier") -> str:
    name = str(value or "")
    if not _SAFE_NAME.fullmatch(name):
        raise ValueError("invalid {}".format(label))
    return name


def validate_sha256(value: Any) -> str:
    digest = str(value or "").lower()
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("sha256 must be 64 hexadecimal characters")
    return digest


def validate_geo_point(value: Any, label: str) -> Dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError("{} must be an object".format(label))
    try:
        latitude = float(value["latitude"])
        longitude = float(value["longitude"])
        altitude = float(value.get("altitude", 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("{} must contain numeric latitude/longitude/altitude".format(label)) from exc
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
        raise ValueError("{} contains a non-finite value".format(label))
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("{} latitude/longitude is out of range".format(label))
    return {"latitude": latitude, "longitude": longitude, "altitude": altitude}


def sha256_file(path: Path, chunk_size: int = 4 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def validate_pcd_header(path: Path) -> None:
    with path.open("rb") as stream:
        sample = stream.read(8192)
    if not sample.startswith(b"# .PCD") or b"DATA " not in sample:
        raise ValueError("file does not look like a PCD")
