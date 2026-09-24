import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from tools.path_creater.geo import GeoPoint, geodetic_to_local_utm
from tools.path_creater.main import ROAD_MAP_FILENAMES
from tools.path_creater.trajectory import write_enu_csv, write_geo_csv, write_road_map_xml


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

    def test_write_road_map_xml_matches_sample_structure(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "road_map1.xml"
            write_road_map_xml(p, [(1.0, 2.0), (3.0, 2.0), (3.0, 5.0)])

            root = ET.parse(p).getroot()
            self.assertEqual(root.tag, "map")
            self.assertEqual(root.attrib, {"generator": "path creater", "upload": "true", "version": "0.6"})
            xml_text = p.read_text(encoding="utf-8")
            self.assertIn('Name="----" />\n  <vertex', xml_text)
            self.assertNotIn(' /><vertex', xml_text)
            vertices = root.findall("vertex")
            self.assertEqual(len(vertices), 3)
            self.assertEqual(vertices[0].attrib["id"], "0000")
            self.assertEqual(vertices[0].attrib["Name"], "----")
            self.assertAlmostEqual(float(vertices[0].attrib["x"]), 1.0)
            self.assertAlmostEqual(float(vertices[0].attrib["y"]), 2.0)
            self.assertAlmostEqual(float(vertices[0].attrib["yaw"]), 0.0)
            self.assertAlmostEqual(float(vertices[1].attrib["yaw"]), 1.570796, places=5)
            self.assertAlmostEqual(float(vertices[2].attrib["yaw"]), 1.570796, places=5)
            edges = root.findall("edge")
            self.assertEqual([edge.attrib for edge in edges], [
                {"id": "0000", "src_id": "0", "dst_id": "1"},
                {"id": "0001", "src_id": "1", "dst_id": "2"},
            ])
            types = root.findall("type")
            self.assertEqual(types[0].attrib["type"], "0")
            self.assertTrue(all(item.attrib["type"] == "2" for item in types[1:]))

    def test_local_utm_origin_is_zero(self):
        origin = GeoPoint(latitude=22.39758935, longitude=113.30942461, altitude=25.8618)
        self.assertEqual(geodetic_to_local_utm(origin, origin), (0.0, 0.0, 0.0))

    def test_local_utm_preserves_local_displacement(self):
        origin = GeoPoint(latitude=22.39758935, longitude=113.30942461, altitude=25.8618)
        point = GeoPoint(latitude=22.39759935, longitude=113.30943461, altitude=27.0)
        east, north, up = geodetic_to_local_utm(point, origin)
        self.assertGreater(east, 0.0)
        self.assertGreater(north, 0.0)
        self.assertAlmostEqual(up, 1.1382, places=4)

    def test_road_map_filename_mapping_is_stable(self):
        self.assertEqual(ROAD_MAP_FILENAMES, {
            "cost_aware": "road_map1.xml",
            "distance": "road_map2.xml",
            "smooth": "road_map3.xml",
        })


if __name__ == "__main__":
    unittest.main()
