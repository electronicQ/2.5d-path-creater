"""车端和地面端共用的 TCP 应用层帧协议。

TCP 只提供有序字节流，不提供消息边界。本模块用长度前缀把字节流
恢复为一帧一帧的应用层消息，控制信息使用 UTF-8 JSON，文件内容使用
原始二进制 payload。
"""

from __future__ import annotations

import asyncio
import json
import struct
from typing import Any, Dict, Tuple


# 单帧默认最大 body 长度为 8 MiB。PCD 等大文件必须拆成多个帧。
DEFAULT_MAX_FRAME_SIZE = 8 * 1024 * 1024

# "!I" 表示网络字节序（big-endian）的 uint32，占 4 字节。
# 协议中所有长度字段都使用这个格式，避免不同平台的字节序差异。
_PREFIX = struct.Struct("!I")
_HEADER_LENGTH = struct.Struct("!I")


class ProtocolError(ValueError):
    """协议帧格式错误，例如长度不合法或 JSON 无法解析。"""

    pass


def encode_frame(header: Dict[str, Any], payload: bytes = b"", max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> bytes:
    """把一个逻辑消息序列化为完整帧字节。

    返回值的布局是：

    ``frame_length | header_length | JSON header | binary payload``

    这里的“完整”只表示已经在内存中拼好，并不代表 TCP 会作为一个
    数据包发送；底层仍可能拆包，接收端必须按长度读取。
    """
    if not isinstance(header, dict):
        raise ProtocolError("frame header must be an object")
    try:
        header_bytes = json.dumps(header, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ProtocolError("frame header is not JSON serializable") from exc
    # bytes() 同时接受 bytearray 等类 bytes 对象，确保最终是原始字节。
    payload = bytes(payload)
    # frame_length 不包含自身的 4 字节，但包含 header_length 字段。
    body_length = _HEADER_LENGTH.size + len(header_bytes) + len(payload)
    if body_length > max_frame_size:
        raise ProtocolError("frame exceeds configured maximum")
    return _PREFIX.pack(body_length) + _HEADER_LENGTH.pack(len(header_bytes)) + header_bytes + payload


def decode_frame(data: bytes, max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> Tuple[Dict[str, Any], bytes]:
    """解析一段必须恰好包含一帧的字节。

    本函数不负责从 socket 读取；调用方应先使用 :func:`read_frame` 读满
    frame_length 指定的 body，再由本函数分离 JSON header 和 payload。
    """
    if len(data) < _PREFIX.size + _HEADER_LENGTH.size:
        raise ProtocolError("incomplete frame")
    # 第一个 4 字节定义当前帧 body 的总长度，也是恢复 TCP 消息边界的依据。
    body_length = _PREFIX.unpack(data[:4])[0]
    if body_length < _HEADER_LENGTH.size or body_length > max_frame_size or len(data) != 4 + body_length:
        raise ProtocolError("frame length mismatch")
    body = data[4:]
    # body 的前 4 字节再说明 JSON header 的长度，剩余部分就是 payload。
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
    """从 asyncio TCP 字节流读取并解析一帧。

    ``readexactly`` 是关键：一次 recv/read 不保证返回完整帧，因此必须
    先读长度，再循环等待指定数量的后续字节。它也能正确处理粘包，
    因为下一次调用会从下一帧的 4 字节长度开始读取。
    """
    prefix = await reader.readexactly(4)
    body_length = _PREFIX.unpack(prefix)[0]
    if body_length < 4 or body_length > max_frame_size:
        raise ProtocolError("invalid frame length")
    body = await reader.readexactly(body_length)
    return decode_frame(prefix + body, max_frame_size)


async def write_frame(writer: asyncio.StreamWriter, header: Dict[str, Any], payload: bytes = b"", max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> None:
    """序列化并发送一帧；``drain`` 用于等待发送缓冲区可继续写入。"""
    writer.write(encode_frame(header, payload, max_frame_size))
    await writer.drain()
