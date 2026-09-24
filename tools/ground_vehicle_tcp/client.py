"""Same-host test client for sending one navigation request and decoding reply."""

from __future__ import annotations

import argparse
import socket

from .protocol import NavigationRequest, RESPONSE_SIZE, decode_response, encode_request


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        chunk = sock.recv(size - len(result))
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
) -> tuple:
    with socket.create_connection((host, port), timeout=connect_timeout_s) as sock:
        sock.settimeout(response_timeout_s)
        sock.sendall(frame)
        prefix = _recv_exact(sock, 5)
        if prefix[4] != RESPONSE_SIZE:
            raise ValueError("unexpected response frame length: {}".format(prefix[4]))
        return decode_response(prefix + _recv_exact(sock, RESPONSE_SIZE - len(prefix)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Send one navigation planning request")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8079)
    parser.add_argument("--response-timeout", type=float, default=3600.0,
                        help="maximum wait for conversion plus planning in seconds")
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
    success, error_code = request(
        args.host, args.port, encode_request(navigation), response_timeout_s=args.response_timeout
    )
    print("success={} error_code=0x{:02X}".format(success, error_code))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
