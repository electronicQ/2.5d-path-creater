import unittest

from tools.path_creater.geo import GeoPoint, enu_to_geodetic, geodetic_to_enu


class GeoConversionTest(unittest.TestCase):
    def test_origin_converts_to_zero_enu(self):
        origin = GeoPoint(latitude=30.0, longitude=120.0, altitude=10.0)
        x, y, z = geodetic_to_enu(origin, origin)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)

    def test_round_trip_medium_range(self):
        origin = GeoPoint(latitude=30.0, longitude=120.0, altitude=10.0)
        point = GeoPoint(latitude=30.001, longitude=120.002, altitude=10.0)
        x, y, z = geodetic_to_enu(point, origin)
        restored = enu_to_geodetic(x, y, z, origin)
        self.assertAlmostEqual(restored.latitude, point.latitude, places=8)
        self.assertAlmostEqual(restored.longitude, point.longitude, places=8)


if __name__ == "__main__":
    unittest.main()
