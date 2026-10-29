import math
import unittest

from tools.path_creater.curve_smoothing import (
    CurveSmoothingConfig,
    TrackingSamplingConfig,
    adaptive_resample_path,
    ensure_path_endpoints,
    smooth_and_sample_path,
)


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


class AdaptiveTrackingSamplingTest(unittest.TestCase):
    @staticmethod
    def segment_lengths(points):
        return [math.dist(a, b) for a, b in zip(points, points[1:])]

    def test_long_straight_segments_use_sparse_points(self):
        points = adaptive_resample_path([(0.0, 0.0), (100.0, 0.0)], TrackingSamplingConfig())

        lengths = self.segment_lengths(points)
        self.assertGreater(len(lengths), 1)
        self.assertLessEqual(max(lengths), 10.0 + 1e-9)
        self.assertGreater(sum(1 for length in lengths if length > 9.0), len(lengths) / 2)

    def test_gentle_curve_uses_one_to_two_meter_spacing(self):
        points = [
            (20.0 * math.sin(math.radians(angle)), 20.0 * (1.0 - math.cos(math.radians(angle))))
            for angle in range(0, 91, 5)
        ]

        sampled = adaptive_resample_path(points, TrackingSamplingConfig())
        lengths = self.segment_lengths(sampled)
        self.assertGreaterEqual(min(lengths), 1.0 - 1e-6)
        self.assertLessEqual(max(lengths), 2.0)

    def test_sharp_curve_uses_about_one_meter_segments(self):
        points = [
            (2.0 * math.sin(math.radians(angle)), 2.0 * (1.0 - math.cos(math.radians(angle))))
            for angle in range(0, 181, 5)
        ]

        sampled = adaptive_resample_path(points, TrackingSamplingConfig())
        lengths = self.segment_lengths(sampled)
        regular_segments = lengths[:-1]
        self.assertGreaterEqual(min(regular_segments), 0.9)
        self.assertLessEqual(max(regular_segments), 1.1)

    def test_adaptive_sampling_keeps_exact_start_and_goal(self):
        start = (1.25, 2.5)
        goal = (15.75, 20.5)
        sampled = adaptive_resample_path(
            [start, (10.0, 2.5), goal],
            TrackingSamplingConfig(),
        )

        self.assertEqual(sampled[0], start)
        self.assertEqual(sampled[-1], goal)

    def test_endpoint_guard_inserts_missing_start_and_goal(self):
        start = (0.1, 0.2)
        goal = (20.3, 40.4)
        points = [(10.0, 20.0), (20.0, 40.0)]

        guarded = ensure_path_endpoints(points, start, goal)

        self.assertEqual(guarded[0], start)
        self.assertEqual(guarded[-1], goal)


if __name__ == "__main__":
    unittest.main()
