"""Run the vehicle gateway as a long-lived asyncio service."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

from .service import GatewayServer, load_gateway_config


async def _run(config_path: Path) -> None:
    config = load_gateway_config(config_path)
    server = GatewayServer(config)
    listener = await server.serve()
    sockets = listener.sockets or []
    print("vehicle_gateway listening on {}".format(", ".join(str(sock.getsockname()) for sock in sockets)), flush=True)
    async with listener:
        await listener.serve_forever()


def main() -> int:
    parser = argparse.ArgumentParser(description="Vehicle-side TCP map and path gateway")
    parser.add_argument("--config", default=str(Path(__file__).resolve().parent / "gateway.yaml"))
    args = parser.parse_args()
    try:
        asyncio.run(_run(Path(args.config)))
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
