import hashlib
import tempfile
import unittest
from pathlib import Path

from tools.ground_station.models import GeoPoint, PlannerRequest
from tools.ground_station.transfer import resumable_file_state, verify_download
from tools.vehicle_protocol import decode_frame, encode_frame


class SharedProtocolTest(unittest.TestCase):
    def test_binary_frame_round_trip(self):
        frame = encode_frame({"type": "artifact.download.reply", "offset": 5}, b"data")
        header, payload = decode_frame(frame)
        self.assertEqual(header["offset"], 5)
        self.assertEqual(payload, b"data")


class GroundStationModelTest(unittest.TestCase):
    def test_planner_request_builds_valid_wire_message(self):
        request = PlannerRequest(
            map_id="map1",
            pgm_name="campus",
            start=GeoPoint(30.0, 120.0, 2.0),
            goal=GeoPoint(30.1, 120.1, 3.0),
            overrides={"inflation_radius_m": 1.5},
        ).to_message("job1")
        self.assertEqual(request["type"], "plan.start")
        self.assertEqual(request["start"]["latitude"], 30.0)

    def test_download_resume_and_checksum(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "result.csv.part"
            path.write_bytes(b"abc")
            self.assertEqual(resumable_file_state(path, 10), 3)
            final = Path(temp) / "result.csv"
            final.write_bytes(b"abcdef")
            verify_download(final, 6, hashlib.sha256(b"abcdef").hexdigest())
            with self.assertRaises(ValueError):
                verify_download(final, 7, hashlib.sha256(b"abcdef").hexdigest())


if __name__ == "__main__":
    unittest.main()
