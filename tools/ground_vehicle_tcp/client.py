"""Same-host test client for sending one navigation request and decoding reply."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from typing import Callable, Optional

from .protocol import NavigationRequest, RESPONSE_SIZE, decode_response, encode_request


ProgressCallback = Callable[[str], None]


def _recv_exact(
    sock: socket.socket,
    size: int,
    deadline: float,
    progress: Optional[ProgressCallback],
    progress_interval_s: float,
    started_at: float,
) -> bytes:
    result = bytearray()
    while len(result) < size:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0:
            raise TimeoutError("timed out waiting for the complete server response")
        if progress is None:
            sock.settimeout(remaining_s)
        else:
            sock.settimeout(min(progress_interval_s, remaining_s))
        try:
            chunk = sock.recv(size - len(result))
        except socket.timeout:
            if progress is not None:
                progress("still waiting: elapsed={:.0f}s".format(time.monotonic() - started_at))
            continue
        if not chunk:
            raise ConnectionError("server closed before sending a complete response")
        result.extend(chunk)
    return bytes(result)


def request(
    host: str,
    port: int,
    frame: bytes,
    connect_timeout_s: float = 5.0,
    response_timeout_s: float = 3600.0,
    progress: Optional[ProgressCallback] = None,
    progress_interval_s: float = 10.0,
) -> tuple:
    if progress_interval_s <= 0:
        raise ValueError("progress_interval_s must be greater than zero")
    started_at = time.monotonic()
    deadline = started_at + response_timeout_s
    if progress is not None:
        progress("connecting to {}:{}".format(host, port))
    with socket.create_connection((host, port), timeout=connect_timeout_s) as sock:
        if progress is not None:
            progress("connected; sending {} request bytes".format(len(frame)))
        sock.settimeout(connect_timeout_s)
        sock.sendall(frame)
        if progress is not None:
            progress("request sent; waiting up to {:.0f}s for the final response".format(response_timeout_s))
        prefix = _recv_exact(sock, 5, deadline, progress, progress_interval_s, started_at)
        if prefix[4] != RESPONSE_SIZE:
            raise ValueError("unexpected response frame length: {}".format(prefix[4]))
        response = decode_response(
            prefix + _recv_exact(
                sock, RESPONSE_SIZE - len(prefix), deadline, progress, progress_interval_s, started_at
            )
        )
        if progress is not None:
            progress("received complete {} response; elapsed={:.1f}s".format(
                RESPONSE_SIZE, time.monotonic() - started_at
            ))
        return response


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Send one navigation planning request")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8079)
    parser.add_argument("--response-timeout", type=float, default=3600.0,
                        help="maximum wait for conversion plus planning in seconds")
    parser.add_argument("--progress-interval", type=float, default=10.0,
                        help="seconds between wait progress messages")
    parser.add_argument("--start-x", type=float, required=True, help="local UTM offset in meters")
    parser.add_argument("--start-y", type=float, required=True, help="local UTM offset in meters")
    parser.add_argument("--start-yaw", type=float, default=0.0, help="radians; accepted but ignored for planning")
    parser.add_argument("--goal-x", type=float, required=True, help="local UTM offset in meters")
    parser.add_argument("--goal-y", type=float, required=True, help="local UTM offset in meters")
    parser.add_argument("--goal-yaw", type=float, default=0.0, help="radians; accepted but ignored for planning")
    parser.add_argument("--timeout", type=int, default=120, help="planning timeout in seconds")
    args = parser.parse_args(argv)
    values = [args.start_x, args.start_y, args.goal_x, args.goal_y, args.start_yaw, args.goal_yaw]
    if any(abs(value) > 2147483.647 for value in values[:4]):
        parser.error("coordinates exceed signed int32 millimeter range")
    navigation = NavigationRequest(
        round(args.start_x * 1000), round(args.start_y * 1000), round(args.start_yaw * 1000),
        round(args.goal_x * 1000), round(args.goal_y * 1000), round(args.goal_yaw * 1000), args.timeout,
    )
    def report(message: str) -> None:
        print("[client] {}".format(message), file=sys.stderr, flush=True)

    report("start=({:.3f}, {:.3f}) m goal=({:.3f}, {:.3f}) m planning-timeout={}s".format(
        args.start_x, args.start_y, args.goal_x, args.goal_y, args.timeout
    ))
    try:
        success, error_code = request(
            args.host,
            args.port,
            encode_request(navigation),
            response_timeout_s=args.response_timeout,
            progress=report,
            progress_interval_s=args.progress_interval,
        )
    except (OSError, ValueError) as exc:
        print("[client] request failed: {}".format(exc), file=sys.stderr, flush=True)
        return 2
    print("success={} error_code=0x{:02X}".format(success, error_code))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
