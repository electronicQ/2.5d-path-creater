import asyncio
import hashlib
import tempfile
import unittest
from pathlib import Path

import yaml

from tools.vehicle_gateway.configuration import apply_overrides
from tools.vehicle_gateway.protocol import decode_frame, encode_frame, read_frame
from tools.vehicle_gateway.service import GatewayConfig, GatewayServer
from tools.vehicle_gateway.storage import GatewayPaths, UploadStore
from tools.vehicle_gateway.validation import safe_basename


class GatewayProtocolTest(unittest.TestCase):
    def test_frame_round_trip_with_binary_payload(self):
        raw = encode_frame({"type": "chunk", "seq": 2}, b"abc\x00def")
        header, payload = decode_frame(raw)
        self.assertEqual(header["seq"], 2)
        self.assertEqual(payload, b"abc\x00def")

    def test_rejects_path_traversal(self):
        with self.assertRaises(ValueError):
            safe_basename("../secret.pcd", ".pcd")


class GatewayStorageTest(unittest.TestCase):
    def test_resumable_pcd_upload_commits_metadata_atomically(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = b"# .PCD v0.7 - Point Cloud Data file format\nDATA ascii\n"
            digest = hashlib.sha256(source).hexdigest()
            store = UploadStore(GatewayPaths(root, root / "var"))
            info = store.begin({
                "map_id": "m1",
                "pcd_name": "scans_Global_ENU.pcd",
                "file_size": len(source),
                "sha256": digest,
                "enu_origin": {"latitude": 30, "longitude": 120, "altitude": 4},
            })
            store.append_chunk(info, 0, source[:10], hashlib.sha256(source[:10]).hexdigest())
            resumed = store.begin({
                "map_id": "m1",
                "pcd_name": "scans_Global_ENU.pcd",
                "file_size": len(source),
                "sha256": digest,
                "enu_origin": {"latitude": 30, "longitude": 120, "altitude": 4},
            })
            self.assertEqual(resumed.offset, 10)
            store.append_chunk(resumed, 10, source[10:])
            result = store.commit(resumed)
            self.assertTrue((root / result["pcd_path"]).is_file())
            self.assertEqual(result["pcd_name"], "scans_Global_ENU.pcd")
            self.assertEqual(yaml.safe_load((root / result["meta_path"]).read_text())["enu_origin"]["latitude"], 30.0)

    def test_artifact_download_is_limited_to_manifest_entries(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config_dir = root / "tools" / "path_creater" / "config"
            config_dir.mkdir(parents=True)
            (config_dir / "config.yaml").write_text("{}\n", encoding="utf-8")
            server = GatewayServer(GatewayConfig(project_root=root, data_root=root / "data"))
            job_dir = root / "data" / "jobs" / "job1"
            output_dir = job_dir / "output"
            output_dir.mkdir(parents=True)
            artifact = output_dir / "traj.csv"
            artifact.write_bytes(b"0123456789")
            (job_dir / "manifest.json").write_text(
                '{"job_id":"job1","state":"SUCCEEDED","artifacts":{"traj.csv":{"path":"data/jobs/job1/output/traj.csv","size":10,"sha256":"x"}}}',
                encoding="utf-8",
            )
            header, payload = server.manager.download_artifact("job1", "traj.csv", 3, 4)
            self.assertEqual(payload, b"3456")
            self.assertEqual(header["next_offset"], 7)
            with self.assertRaises(ValueError):
                server.manager.download_artifact("job1", "../config.yaml", 0, 4)


class GatewayConfigTest(unittest.TestCase):
    def test_allowlisted_config_update(self):
        config = {"map_yaml": "old", "map_meta": "old", "start": {}, "goal": {}, "inflation_radius_m": 1.0}
        result = apply_overrides(config, {
            "start": {"latitude": 30, "longitude": 120, "altitude": 2},
            "goal": {"latitude": 30.1, "longitude": 120.1, "altitude": 3},
            "config_overrides": {"inflation_radius_m": 2.0},
        }, "campus", "jobs/j1/output")
        self.assertEqual(result["map_yaml"], "../../../map/pgm/campus.yaml")
        self.assertEqual(result["inflation_radius_m"], 2.0)
        with self.assertRaises(ValueError):
            apply_overrides(config, {
                "start": {"latitude": 30, "longitude": 120}, "goal": {"latitude": 30, "longitude": 120},
                "config_overrides": {"shell": "rm -rf /"},
            }, "campus", "output")


class GatewayServerTest(unittest.IsolatedAsyncioTestCase):
    async def test_hello_round_trip_over_tcp(self):
        with tempfile.TemporaryDirectory() as temp:
            server = GatewayServer(GatewayConfig(host="127.0.0.1", port=0, project_root=Path(temp), data_root=Path(temp) / "data", token="secret"))
            try:
                listener = await server.serve()
            except (OSError, PermissionError) as exc:
                self.skipTest("sandbox does not permit TCP sockets: {}".format(exc))
            if not listener.sockets:
                listener.close()
                await listener.wait_closed()
                self.skipTest("sandbox did not provide a listening socket")
            port = listener.sockets[0].getsockname()[1]
            reader, writer = await asyncio.open_connection("127.0.0.1", port)
            writer.write(encode_frame({"type": "hello", "request_id": "r1", "token": "secret"}))
            await writer.drain()
            header, payload = await read_frame(reader)
            self.assertEqual(header["status"], "succeeded")
            self.assertEqual(payload, b"")
            writer.close()
            await writer.wait_closed()
            listener.close()
            await listener.wait_closed()


if __name__ == "__main__":
    unittest.main()
