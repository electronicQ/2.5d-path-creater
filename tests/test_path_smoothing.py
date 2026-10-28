import unittest

import numpy as np

from tools.path_creater.path_smoothing import (
    segment_is_free,
    segment_meets_minimum,
    shortcut_path,
    turning_points,
)


class PathSmoothingTest(unittest.TestCase):
    def test_shortcut_removes_unneeded_collinear_waypoints(self):
        free = np.ones((3, 6), dtype=bool)
        path = [(1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5)]

        self.assertEqual(shortcut_path(free, path), [(1, 0), (1, 5)])

    def test_shortcut_keeps_waypoint_when_direct_segment_crosses_blocked_cell(self):
        free = np.ones((3, 6), dtype=bool)
        free[1, 3] = False
        path = [(1, 0), (1, 1), (1, 2), (0, 3), (1, 4), (1, 5)]

        smoothed = shortcut_path(free, path)

        self.assertIn((0, 3), smoothed)
        self.assertFalse(segment_is_free(free, (1, 0), (1, 5)))

    def test_segment_rejects_diagonal_corner_cut(self):
        free = np.ones((3, 3), dtype=bool)
        free[0, 1] = False
        free[1, 0] = False

        self.assertFalse(segment_is_free(free, (0, 0), (1, 1)))

    def test_shortcut_respects_minimum_corridor_clearance(self):
        free = np.ones((3, 6), dtype=bool)
        corridor_distance = np.full((3, 6), 10.0, dtype=np.float32)
        corridor_distance[:, 2:4] = 0.5
        path = [(1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5)]

        smoothed = shortcut_path(
            free,
            path,
            clearance_values=corridor_distance,
            minimum_clearance=2.0,
        )

        self.assertGreater(len(smoothed), 2)
        self.assertFalse(segment_meets_minimum(corridor_distance, (1, 0), (1, 5), 2.0))

    def test_clearance_constraint_only_applies_inside_required_mask(self):
        corridor_distance = np.ones((3, 6), dtype=np.float32)
        corridor_distance[:, 2:4] = 0.5
        inactive = np.zeros((3, 6), dtype=bool)

        self.assertFalse(segment_meets_minimum(corridor_distance, (1, 0), (1, 5), 2.0))
        self.assertTrue(
            segment_meets_minimum(
                corridor_distance,
                (1, 0),
                (1, 5),
                2.0,
                required_mask=inactive,
            )
        )

    def test_turning_points_ignores_straight_inner_vertices(self):
        path = [(0, 0), (0, 3), (0, 6), (4, 6)]

        self.assertEqual(turning_points(path, 20.0), [(0, 6)])


if __name__ == "__main__":
    unittest.main()
