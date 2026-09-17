"""Backward-compatible imports for the shared vehicle protocol."""

from tools.vehicle_protocol import DEFAULT_MAX_FRAME_SIZE, ProtocolError, decode_frame, encode_frame, read_frame, write_frame

__all__ = ["DEFAULT_MAX_FRAME_SIZE", "ProtocolError", "decode_frame", "encode_frame", "read_frame", "write_frame"]
