import unittest

import numpy as np

from tools.path_creater.path_smoothing import segment_is_free, shortcut_path


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


if __name__ == "__main__":
    unittest.main()
