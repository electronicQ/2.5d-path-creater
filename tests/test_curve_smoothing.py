import math
import unittest

from tools.path_creater.curve_smoothing import CurveSmoothingConfig, smooth_and_sample_path


class CurveSmoothingTest(unittest.TestCase):
    def test_resamples_straight_path_at_fixed_interval(self):
        cfg = CurveSmoothingConfig(sample_interval_m=1.0, corner_cut_fraction=0.4, max_corner_cut_m=1.0)

        points = smooth_and_sample_path([(0.0, 0.0), (3.0, 0.0)], lambda a, b: True, cfg)

        self.assertEqual(points, [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)])

    def test_rounds_corner_when_arc_is_safe(self):
        cfg = CurveSmoothingConfig(sample_interval_m=0.5, corner_cut_fraction=0.5, max_corner_cut_m=1.0)

        points = smooth_and_sample_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], lambda a, b: True, cfg)

        self.assertEqual(points[0], (0.0, 0.0))
        self.assertEqual(points[-1], (2.0, 2.0))
        self.assertNotIn((2.0, 0.0), points)
        self.assertTrue(any(1.0 < x < 2.0 and 0.0 < y < 1.0 for x, y in points))

    def test_keeps_corner_when_rounded_arc_is_not_safe(self):
        def only_axis_aligned(a, b):
            return math.isclose(a[0], b[0], abs_tol=1e-9) or math.isclose(a[1], b[1], abs_tol=1e-9)

        cfg = CurveSmoothingConfig(sample_interval_m=1.0, corner_cut_fraction=0.5, max_corner_cut_m=1.0)

        points = smooth_and_sample_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], only_axis_aligned, cfg)

        self.assertIn((2.0, 0.0), points)


if __name__ == "__main__":
    unittest.main()
