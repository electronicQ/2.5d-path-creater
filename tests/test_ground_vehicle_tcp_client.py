import contextlib
import io
import socket
import unittest
from unittest.mock import patch

from tools.ground_vehicle_tcp import client
from tools.ground_vehicle_tcp.protocol import RESPONSE_SIZE, encode_response


class FakeSocket:
    def __init__(self):
        self.sent = bytearray()
        self.timeouts = []
        self.received = bytearray(encode_response(True))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def settimeout(self, timeout):
        self.timeouts.append(timeout)

    def sendall(self, frame):
        self.sent.extend(frame)

    def recv(self, size):
        if not self.received:
            raise socket.timeout()
        chunk = bytes(self.received[:size])
        del self.received[:size]
        return chunk


class GroundVehicleTcpClientTest(unittest.TestCase):
    def test_request_reports_connection_and_final_response(self):
        sock = FakeSocket()
        messages = []
        with patch("tools.ground_vehicle_tcp.client.socket.create_connection", return_value=sock):
            result = client.request(
                "127.0.0.1", 8079, b"x" * 32,
                response_timeout_s=2,
                progress=messages.append,
                progress_interval_s=1,
            )

        self.assertEqual(result, (True, 0))
        self.assertEqual(sock.sent, b"x" * 32)
        self.assertEqual(messages[0], "connecting to 127.0.0.1:8079")
        self.assertEqual(messages[1], "connected; sending 32 request bytes")
        self.assertIn("waiting up to 2s", messages[2])
        self.assertIn("received complete {} response".format(RESPONSE_SIZE), messages[3])

    def test_recv_exact_reports_heartbeat_while_socket_waits(self):
        sock = FakeSocket()
        messages = []

        with patch.object(sock, "recv", side_effect=[socket.timeout(), encode_response(True)]):
            response = client._recv_exact(
                sock,
                RESPONSE_SIZE,
                deadline=client.time.monotonic() + 1,
                progress=messages.append,
                progress_interval_s=0.01,
                started_at=client.time.monotonic(),
            )

        self.assertEqual(response, encode_response(True))
        self.assertEqual(messages, ["still waiting: elapsed=0s"])

    def test_cli_keeps_final_result_on_standard_output(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with patch("tools.ground_vehicle_tcp.client.request", return_value=(True, 0)):
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                status = client.main([
                    "--host", "127.0.0.1",
                    "--port", "8079",
                    "--start-x", "1", "--start-y", "2",
                    "--goal-x", "3", "--goal-y", "4",
                    "--response-timeout", "2",
                    "--progress-interval", "0.5",
                ])

        self.assertEqual(status, 0)
        self.assertEqual(stdout.getvalue(), "success=True error_code=0x00\n")
        self.assertIn("[client] start=(1.000, 2.000) m goal=(3.000, 4.000) m", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
