"""Shared length-prefixed JSON/binary protocol for vehicle and ground station."""

from __future__ import annotations

import asyncio
import json
import struct
from typing import Any, Dict, Tuple


DEFAULT_MAX_FRAME_SIZE = 8 * 1024 * 1024
_PREFIX = struct.Struct("!I")
_HEADER_LENGTH = struct.Struct("!I")


class ProtocolError(ValueError):
    pass


def encode_frame(header: Dict[str, Any], payload: bytes = b"", max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> bytes:
    if not isinstance(header, dict):
        raise ProtocolError("frame header must be an object")
    try:
        header_bytes = json.dumps(header, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ProtocolError("frame header is not JSON serializable") from exc
    payload = bytes(payload)
    body_length = _HEADER_LENGTH.size + len(header_bytes) + len(payload)
    if body_length > max_frame_size:
        raise ProtocolError("frame exceeds configured maximum")
    return _PREFIX.pack(body_length) + _HEADER_LENGTH.pack(len(header_bytes)) + header_bytes + payload


def decode_frame(data: bytes, max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> Tuple[Dict[str, Any], bytes]:
    if len(data) < _PREFIX.size + _HEADER_LENGTH.size:
        raise ProtocolError("incomplete frame")
    body_length = _PREFIX.unpack(data[:4])[0]
    if body_length < _HEADER_LENGTH.size or body_length > max_frame_size or len(data) != 4 + body_length:
        raise ProtocolError("frame length mismatch")
    body = data[4:]
    header_length = _HEADER_LENGTH.unpack(body[:4])[0]
    if header_length > body_length - 4:
        raise ProtocolError("invalid header length")
    try:
        header = json.loads(body[4:4 + header_length].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid JSON header") from exc
    if not isinstance(header, dict):
        raise ProtocolError("JSON header must be an object")
    return header, body[4 + header_length:]


async def read_frame(reader: asyncio.StreamReader, max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> Tuple[Dict[str, Any], bytes]:
    prefix = await reader.readexactly(4)
    body_length = _PREFIX.unpack(prefix)[0]
    if body_length < 4 or body_length > max_frame_size:
        raise ProtocolError("invalid frame length")
    body = await reader.readexactly(body_length)
    return decode_frame(prefix + body, max_frame_size)


async def write_frame(writer: asyncio.StreamWriter, header: Dict[str, Any], payload: bytes = b"", max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> None:
    writer.write(encode_frame(header, payload, max_frame_size))
    await writer.drain()
