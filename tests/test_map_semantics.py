import unittest

import numpy as np

from tools.path_creater.map_semantics import MapSemantics, build_semantic_layers


class MapSemanticsTest(unittest.TestCase):
    def test_unknown_and_obstacle_are_forbidden(self):
        gray = np.array([[205, 90, 91, 255]], dtype=np.uint8)
        layers = build_semantic_layers(
            gray,
            MapSemantics(unknown_gray=205, absolute_obstacle_gray=90, gray_cost_weight=1.0),
        )
        self.assertEqual(layers.forbidden.tolist(), [[True, True, False, False]])
        self.assertEqual(layers.free.tolist(), [[False, False, True, True]])

    def test_gray_cost_is_monotonic_and_zero_for_forbidden_cells(self):
        gray = np.array([[205, 90, 128, 255]], dtype=np.uint8)
        layers = build_semantic_layers(
            gray,
            MapSemantics(unknown_gray=205, absolute_obstacle_gray=90, gray_cost_weight=1.0),
        )
        self.assertEqual(float(layers.gray_cost[0, 0]), 0.0)
        self.assertEqual(float(layers.gray_cost[0, 1]), 0.0)
        self.assertGreater(float(layers.gray_cost[0, 2]), float(layers.gray_cost[0, 3]))
        self.assertAlmostEqual(float(layers.gray_cost[0, 3]), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
