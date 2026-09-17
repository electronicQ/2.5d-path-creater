"""TCP gateway for vehicle-side map conversion and path planning."""

from .protocol import ProtocolError, encode_frame, read_frame, write_frame

__all__ = ["ProtocolError", "encode_frame", "read_frame", "write_frame"]
