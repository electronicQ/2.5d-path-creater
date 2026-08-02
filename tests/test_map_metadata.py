import tempfile
import unittest
from pathlib import Path

from tools.path_creater.map_metadata import load_map_metadata


class MapMetadataTest(unittest.TestCase):
    def test_loads_enu_origin(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "map.meta.yaml"
            p.write_text(
                "enu_origin:\n  latitude: 30.0\n  longitude: 120.0\n  altitude: 5.0\n",
                encoding="utf-8",
            )
            meta = load_map_metadata(p)
            self.assertEqual(meta.enu_origin.latitude, 30.0)
            self.assertEqual(meta.enu_origin.longitude, 120.0)
            self.assertEqual(meta.enu_origin.altitude, 5.0)


if __name__ == "__main__":
    unittest.main()
