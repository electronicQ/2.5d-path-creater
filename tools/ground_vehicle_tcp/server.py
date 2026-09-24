"""Async TCP server for the vehicle's fixed-size navigation protocol."""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from .protocol import (
    ERROR_BAD_FRAME,
    ERROR_INTERNAL,
    REQUEST_SIZE,
    ProtocolError,
    decode_request,
    encode_response,
)


class NavigationServer:
    def __init__(self, workflow, host: str = "0.0.0.0", port: int = 8079) -> None:
        self.workflow = workflow
        self.host = host
        self.port = int(port)
        self._busy = False
        self._listener: Optional[asyncio.AbstractServer] = None
        self.logger = logging.getLogger("ground_vehicle_tcp")

    async def serve(self) -> asyncio.AbstractServer:
        self._listener = await asyncio.start_server(self._handle_client, self.host, self.port)
        return self._listener

    async def run_forever(self) -> None:
        listener = await self.serve()
        async with listener:
            await listener.serve_forever()

    async def close(self) -> None:
        if self._listener is not None:
            self._listener.close()
            await self._listener.wait_closed()
            self._listener = None

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        try:
            while True:
                try:
                    prefix = await reader.readexactly(5)
                except asyncio.IncompleteReadError as exc:
                    if exc.partial:
                        self.logger.warning("incomplete request header from %s", peer)
                    break

                frame_length = prefix[4]
                if frame_length != REQUEST_SIZE:
                    await self._send(writer, False, ERROR_BAD_FRAME)
                    break
                try:
                    rest = await reader.readexactly(REQUEST_SIZE - len(prefix))
                except asyncio.IncompleteReadError:
                    self.logger.warning("incomplete request frame from %s", peer)
                    break
                try:
                    request = decode_request(prefix + rest)
                except ProtocolError as exc:
                    self.logger.warning("invalid request from %s: %s", peer, exc)
                    await self._send(writer, False, ERROR_BAD_FRAME)
                    continue

                if self._busy:
                    await self._send(writer, False, ERROR_INTERNAL)
                    continue
                self._busy = True
                try:
                    loop = asyncio.get_running_loop()
                    error_code = await loop.run_in_executor(None, self.workflow.execute, request)
                except Exception:
                    self.logger.exception("unhandled navigation workflow error")
                    error_code = ERROR_INTERNAL
                finally:
                    self._busy = False
                await self._send(writer, error_code == 0, error_code)
        except (ConnectionError, asyncio.CancelledError):
            pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except ConnectionError:
                pass

    @staticmethod
    async def _send(writer: asyncio.StreamWriter, success: bool, error_code: int) -> None:
        writer.write(encode_response(success, error_code))
        await writer.drain()


__all__ = ["NavigationServer"]
