import unittest

from tools.pgm_saver import save_continuous_map


class MapSaverConversionTest(unittest.TestCase):
    def test_occupancy_values_map_to_expected_grayscale(self):
        img = save_continuous_map.occupancy_to_pgm_array([-1, 100, 0, 50], width=4, height=1)
        self.assertEqual(img.tolist(), [[205, 0, 254, 127]])


if __name__ == "__main__":
    unittest.main()
