import tempfile
import unittest
from pathlib import Path

from tools.path_creater.geo import GeoPoint
from tools.path_creater.trajectory import write_enu_csv, write_geo_csv


class TrajectoryOutputTest(unittest.TestCase):
    def test_write_enu_csv(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "traj_enu.csv"
            write_enu_csv(p, [(1.25, 2.5)])
            self.assertEqual(p.read_text(encoding="utf-8").strip(), "x,y\n1.250000,2.500000")

    def test_write_geo_csv(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "traj_geo.csv"
            write_geo_csv(p, [GeoPoint(latitude=30.1, longitude=120.2)])
            self.assertEqual(p.read_text(encoding="utf-8").strip(), "latitude,longitude\n30.10000000,120.20000000")


if __name__ == "__main__":
    unittest.main()
