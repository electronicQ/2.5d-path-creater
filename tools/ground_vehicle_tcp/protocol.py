"""The fixed-size CRC8 TCP application protocol from the vehicle interface."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Tuple


MAGIC_REQUEST = 0xFFDA
MAGIC_RESPONSE = 0xFFDB
MESSAGE_TYPE_PLAN = 0x0001
REQUEST_SIZE = 0x20
RESPONSE_SIZE = 0x08

ERROR_NONE = 0x00
ERROR_TARGET_OUTSIDE_MAP = 0x01
ERROR_NO_PATH = 0x02
ERROR_START_NOT_PLANNABLE = 0x03
ERROR_MAP_CONVERSION = 0x04
ERROR_PLANNING_TIMEOUT = 0x05
ERROR_BAD_FRAME = 0x06
ERROR_INTERNAL = 0x07


class ProtocolError(ValueError):
    """Raised when a fixed protocol frame is malformed or fails CRC8."""


@dataclass(frozen=True)
class NavigationRequest:
    start_x_mm: int
    start_y_mm: int
    start_yaw_millirad: int
    goal_x_mm: int
    goal_y_mm: int
    goal_yaw_millirad: int
    timeout_s: int

    @property
    def start_xy_m(self) -> Tuple[float, float]:
        return self.start_x_mm / 1000.0, self.start_y_mm / 1000.0

    @property
    def goal_xy_m(self) -> Tuple[float, float]:
        return self.goal_x_mm / 1000.0, self.goal_y_mm / 1000.0


def crc8(data: bytes) -> int:
    """CRC-8/ATM style, init 0, polynomial 0x07, no reflection/xorout."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _check_int32(value: int, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not -(2**31) <= value < 2**31:
        raise ProtocolError("{} must be a signed int32".format(field))
    return value


def encode_request(request: NavigationRequest) -> bytes:
    values = (
        _check_int32(request.start_x_mm, "start_x_mm"),
        _check_int32(request.start_y_mm, "start_y_mm"),
        _check_int32(request.start_yaw_millirad, "start_yaw_millirad"),
        _check_int32(request.goal_x_mm, "goal_x_mm"),
        _check_int32(request.goal_y_mm, "goal_y_mm"),
        _check_int32(request.goal_yaw_millirad, "goal_yaw_millirad"),
    )
    if isinstance(request.timeout_s, bool) or not isinstance(request.timeout_s, int) or not 1 <= request.timeout_s <= 0xFFFF:
        raise ProtocolError("timeout_s must be in [1, 65535]")
    body = struct.pack(">2sHB6iH", b"\xff\xda", MESSAGE_TYPE_PLAN, REQUEST_SIZE, *(values + (request.timeout_s,)))
    return body + bytes((crc8(body),))


def decode_request(frame: bytes) -> NavigationRequest:
    if len(frame) != REQUEST_SIZE:
        raise ProtocolError("request frame must be exactly 32 bytes")
    if frame[:2] != b"\xff\xda":
        raise ProtocolError("invalid request magic")
    message_type, frame_size = struct.unpack_from(">HB", frame, 2)
    if message_type != MESSAGE_TYPE_PLAN:
        raise ProtocolError("unsupported request type")
    if frame_size != REQUEST_SIZE:
        raise ProtocolError("invalid request length")
    if crc8(frame[:-1]) != frame[-1]:
        raise ProtocolError("request CRC8 mismatch")
    values = struct.unpack_from(">6iH", frame, 5)
    if values[-1] == 0:
        raise ProtocolError("planning timeout must be greater than zero")
    return NavigationRequest(*values)


def encode_response(success: bool, error_code: int = ERROR_NONE) -> bytes:
    if not 0 <= int(error_code) <= 0xFF:
        raise ProtocolError("error_code must fit in one byte")
    if success and error_code != ERROR_NONE:
        raise ProtocolError("successful response must use error code zero")
    body = struct.pack(">2sHBBB", b"\xff\xdb", MESSAGE_TYPE_PLAN, RESPONSE_SIZE, 1 if success else 0, int(error_code))
    return body + bytes((crc8(body),))


def decode_response(frame: bytes) -> Tuple[bool, int]:
    if len(frame) != RESPONSE_SIZE:
        raise ProtocolError("response frame must be exactly 8 bytes")
    if frame[:2] != b"\xff\xdb":
        raise ProtocolError("invalid response magic")
    message_type, frame_size, result, error_code = struct.unpack_from(">HBBB", frame, 2)
    if message_type != MESSAGE_TYPE_PLAN or frame_size != RESPONSE_SIZE:
        raise ProtocolError("invalid response type or length")
    if result not in (0, 1) or crc8(frame[:-1]) != frame[-1]:
        raise ProtocolError("invalid response status or CRC8")
    if (result == 1) != (error_code == ERROR_NONE):
        raise ProtocolError("response status conflicts with error code")
    return result == 1, error_code
