import tempfile
import unittest
import asyncio
from pathlib import Path

import yaml
from PIL import Image

from tools.ground_vehicle_tcp.map_cache import MapCache
from tools.ground_vehicle_tcp.pipeline import PipelineError
from tools.ground_vehicle_tcp.server import NavigationServer
from tools.ground_vehicle_tcp.workflow import NavigationWorkflow
from tools.ground_vehicle_tcp.protocol import (
    ERROR_BAD_FRAME,
    ERROR_NONE,
    MAGIC_REQUEST,
    MAGIC_RESPONSE,
    NavigationRequest,
    ProtocolError,
    crc8,
    decode_request,
    decode_response,
    encode_request,
    encode_response,
)
from tools.path_creater.geo import GeoPoint, geodetic_to_local_utm, local_utm_to_geodetic


class ProtocolTest(unittest.TestCase):
    def test_crc8_matches_protocol_check_vector(self):
        self.assertEqual(crc8(b"123456789"), 0xF4)

    def test_request_round_trip_is_exact_32_byte_big_endian_frame(self):
        request = NavigationRequest(
            start_x_mm=123456,
            start_y_mm=-234567,
            start_yaw_millirad=-1571,
            goal_x_mm=765432,
            goal_y_mm=876543,
            goal_yaw_millirad=3142,
            timeout_s=90,
        )
        frame = encode_request(request)
        self.assertEqual(len(frame), 32)
        self.assertEqual(frame[:5], b"\xff\xda\x00\x01\x20")
        self.assertEqual(decode_request(frame), request)

    def test_request_crc_error_is_rejected(self):
        frame = bytearray(encode_request(NavigationRequest(0, 0, 0, 1000, 2000, 0, 30)))
        frame[7] ^= 0x01
        with self.assertRaises(ProtocolError):
            decode_request(bytes(frame))

    def test_response_is_exactly_8_bytes_and_validates_crc(self):
        frame = encode_response(True, ERROR_NONE)
        self.assertEqual(len(frame), 8)
        self.assertEqual(frame[:5], b"\xff\xdb\x00\x01\x08")
        self.assertEqual(decode_response(frame), (True, ERROR_NONE))
        failed = encode_response(False, ERROR_BAD_FRAME)
        self.assertEqual(decode_response(failed), (False, ERROR_BAD_FRAME))


class NavigationServerTest(unittest.TestCase):
    class FakeWriter:
        def __init__(self):
            self.data = bytearray()
            self.closed = False

        def get_extra_info(self, name):
            return ("local", 1234) if name == "peername" else None

        def write(self, data):
            self.data.extend(data)

        async def drain(self):
            return None

        def close(self):
            self.closed = True

        async def wait_closed(self):
            return None

    def test_handler_reads_exact_request_and_writes_final_response(self):
        class FakeWorkflow:
            def execute(self, request):
                self.request = request
                return ERROR_NONE

        async def run():
            reader = asyncio.StreamReader()
            request = NavigationRequest(1000, 2000, 3000, 4000, 5000, 6000, 60)
            reader.feed_data(encode_request(request))
            reader.feed_eof()
            writer = self.FakeWriter()
            workflow = FakeWorkflow()
            await NavigationServer(workflow)._handle_client(reader, writer)
            return workflow, writer

        workflow, writer = asyncio.run(run())
        self.assertEqual(workflow.request, NavigationRequest(1000, 2000, 3000, 4000, 5000, 6000, 60))
        self.assertEqual(decode_response(bytes(writer.data)), (True, ERROR_NONE))
        self.assertTrue(writer.closed)


class NavigationWorkflowErrorTest(unittest.TestCase):
    def test_planner_failures_map_to_protocol_error_codes(self):
        self.assertEqual(NavigationWorkflow._planning_error(ValueError("Configured start outside map bounds.")), 3)
        self.assertEqual(NavigationWorkflow._planning_error(ValueError("Configured goal outside map bounds.")), 1)
        self.assertEqual(NavigationWorkflow._planning_error(RuntimeError("A* failed: no path between start and goal.")), 2)

    def test_workflow_ignores_input_yaw_and_publishes_verified_trajectories(self):
        class FakeCache:
            def ensure_map(self, pcd_path, pcd_name, origin, log_path):
                return root / "cached-map.yaml"

        class FakeRunner:
            configs = []

            def __init__(self, project_root, timeout_s):
                self.timeout_s = timeout_s

            def plan(self, config_path, output_dir, log_path):
                with Path(config_path).open("r", encoding="utf-8") as stream:
                    type(self).configs.append(yaml.safe_load(stream))
                output_dir.mkdir(parents=True, exist_ok=True)
                for name in ("road_map1.xml", "road_map2.xml", "road_map3.xml"):
                    (output_dir / name).write_text(
                        '<map><vertex x="0" y="0" yaw="0" /></map>\n', encoding="utf-8"
                    )
                Image.new("RGB", (2, 2), (255, 255, 255)).save(str(output_dir / "overlay.png"))

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            metadata = root / "map_enu.meta.yaml"
            metadata.write_text(
                "enu_origin:\n  latitude: 22.39758935\n  longitude: 113.30942461\n  altitude: 25.8618\n",
                encoding="utf-8",
            )
            planner_config = root / "config.yaml"
            planner_config.write_text("output_dir: output\n", encoding="utf-8")
            FakeRunner.configs = []
            workflow = NavigationWorkflow(
                root, root / "state", root / "unused.pcd", "unused.pcd", metadata,
                planner_config, runner_factory=FakeRunner, map_cache=FakeCache(),
            )
            first = NavigationRequest(120000, -45000, 0, 130000, -40000, 1234, 90)
            second = NavigationRequest(120000, -45000, -3200, 130000, -40000, -2345, 90)
            self.assertEqual(workflow.execute(first), ERROR_NONE)
            self.assertEqual(workflow.execute(second), ERROR_NONE)
            self.assertEqual(FakeRunner.configs[0]["start"], FakeRunner.configs[1]["start"])
            self.assertEqual(FakeRunner.configs[0]["goal"], FakeRunner.configs[1]["goal"])
            self.assertEqual(FakeRunner.configs[0]["output_dir"].split("/")[-1], "output")
            self.assertTrue((root / "output" / "road_map1.xml").is_file())
            self.assertTrue((root / "output" / "overlay.png").is_file())


class NavigationOutputValidationTest(unittest.TestCase):
    @staticmethod
    def write_outputs(output_dir, include_overlay=True, include_road_map3=True):
        output_dir.mkdir(parents=True, exist_ok=True)
        road_maps = ["road_map1.xml", "road_map2.xml"]
        if include_road_map3:
            road_maps.append("road_map3.xml")
        for name in road_maps:
            (output_dir / name).write_text(
                '<map><vertex x="0" y="0" yaw="0" /></map>\n', encoding="utf-8"
            )
        if include_overlay:
            Image.new("RGB", (2, 2), (255, 255, 255)).save(str(output_dir / "overlay.png"))

    def test_validates_and_publishes_map_overlay(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            destination = root / "output"
            self.write_outputs(source)
            destination.mkdir()
            (destination / "traj_distance_geo.csv").write_text("old\n", encoding="utf-8")
            (destination / "config.resolved.yaml").write_text("old: true\n", encoding="utf-8")

            NavigationWorkflow._validate_outputs(source)
            NavigationWorkflow._publish_outputs(source, destination)

            self.assertTrue((destination / "overlay.png").is_file())
            self.assertEqual((destination / "overlay.png").read_bytes(), (source / "overlay.png").read_bytes())
            self.assertFalse((destination / "traj_distance_geo.csv").exists())
            self.assertFalse((destination / "config.resolved.yaml").exists())

    def test_rejects_missing_trajectory_xml(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "output"
            self.write_outputs(output, include_road_map3=False)

            with self.assertRaisesRegex(PipelineError, "road_map3.xml"):
                NavigationWorkflow._validate_outputs(output)


class LocalUtmConversionTest(unittest.TestCase):
    def test_local_utm_point_round_trips_through_geodetic(self):
        origin = GeoPoint(22.39758935, 113.30942461, 25.8618)
        point = local_utm_to_geodetic(120.0, -45.0, origin)
        x, y, _ = geodetic_to_local_utm(point, origin)
        self.assertAlmostEqual(x, 120.0, delta=1e-6)
        self.assertAlmostEqual(y, -45.0, delta=1e-6)


class MapCacheTest(unittest.TestCase):
    class FakeRunner:
        conversions = 0

        def __init__(self, project_root, timeout_s):
            self.project_root = project_root

        def convert(self, pcd_name, output_base, log_path, rviz=False):
            type(self).conversions += 1
            output_base.parent.mkdir(parents=True, exist_ok=True)
            Path(str(output_base) + ".pgm").write_bytes(b"P5\n1 1\n255\n\xff")
            Path(str(output_base) + ".yaml").write_text(
                "image: map_enu.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n",
                encoding="utf-8",
            )

    def test_map_fingerprint_reuses_valid_conversion_and_changes_with_origin(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            pcd = root / "scans_Global_ENU.pcd"
            pcd.write_bytes(b"pcd-content")
            log = root / "map.log"
            cache = MapCache(root / "state", root, runner_factory=self.FakeRunner)
            origin = GeoPoint(22.39758935, 113.30942461, 25.8618)
            self.FakeRunner.conversions = 0

            first = cache.ensure_map(pcd, "scans_Global_ENU.pcd", origin, log)
            again = cache.ensure_map(pcd, "scans_Global_ENU.pcd", origin, log)
            changed = GeoPoint(origin.latitude + 0.0001, origin.longitude, origin.altitude)
            changed_path = cache.ensure_map(pcd, "scans_Global_ENU.pcd", changed, log)
            pcd.write_bytes(b"updated-pcd-content")
            updated_path = cache.ensure_map(pcd, "scans_Global_ENU.pcd", origin, log)

            self.assertEqual(first, again)
            self.assertNotEqual(first, changed_path)
            self.assertNotEqual(first, updated_path)
            self.assertEqual(self.FakeRunner.conversions, 3)
            manifest = yaml.safe_load((updated_path.parent / "manifest.yaml").read_text(encoding="utf-8"))
            self.assertEqual(manifest["pcd_sha256"], cache.file_sha256(pcd))


if __name__ == "__main__":
    unittest.main()
