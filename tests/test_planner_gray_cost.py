import unittest

import numpy as np

from tools.path_creater.clearance import ClearanceMap, build_corridor_distance
from tools.path_creater.planner import clearance_penalty, plan_mode


class PlannerGrayCostTest(unittest.TestCase):
    def test_soft_clearance_decays_linearly_to_eight_meters(self):
        self.assertAlmostEqual(clearance_penalty(1.6, 8.0), 6.4)
        self.assertAlmostEqual(clearance_penalty(4.0, 8.0), 4.0)
        self.assertAlmostEqual(clearance_penalty(7.5, 8.0), 0.5)
        self.assertEqual(clearance_penalty(8.0, 8.0), 0.0)

    def test_corridor_distance_is_larger_near_road_center(self):
        gray = np.full((5, 9), 250, dtype=np.uint8)
        gray[:, 0] = 120
        gray[:, -1] = 120

        distance = build_corridor_distance(
            gray,
            resolution=0.5,
            high_cost_gray_threshold=220,
        )

        self.assertGreater(distance[2, 4], distance[2, 1])
        self.assertAlmostEqual(distance[2, 1], 0.5)

    def test_gray_cost_discourages_middle_high_cost_cell(self):
        free = np.ones((3, 5), dtype=bool)
        dist_m = np.full((3, 5), 10.0, dtype=np.float32)
        gray_cost = np.zeros((3, 5), dtype=np.float32)
        gray_cost[1, 2] = 50.0
        clearance = ClearanceMap(free=free, dist_m=dist_m, inflation_radius_m=0.0, occupied_thresh=0.0)
        path = plan_mode(
            clearance,
            gray_cost,
            (1, 0),
            (1, 4),
            {"w_turn": 0.0, "w_clear": 0.0, "w_gray": 10.0},
            d_safe_m=0.0,
        )
        self.assertNotIn((1, 2), path)

    def test_corridor_penalty_moves_path_to_wider_center_region(self):
        free = np.ones((5, 5), dtype=bool)
        dist_m = np.full((5, 5), 10.0, dtype=np.float32)
        gray_cost = np.zeros((5, 5), dtype=np.float32)
        corridor_distance = np.array([
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [3.0, 3.0, 3.0, 3.0, 3.0],
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [3.0, 3.0, 3.0, 3.0, 3.0],
            [0.5, 0.5, 0.5, 0.5, 0.5],
        ], dtype=np.float32)
        clearance = ClearanceMap(free=free, dist_m=dist_m, inflation_radius_m=0.0, occupied_thresh=0.0)

        shortest = plan_mode(
            clearance,
            gray_cost,
            (2, 0),
            (2, 4),
            {"w_turn": 0.0, "w_clear": 0.0, "w_gray": 0.0},
            d_safe_m=0.0,
        )
        centered = plan_mode(
            clearance,
            gray_cost,
            (2, 0),
            (2, 4),
            {"w_turn": 0.0, "w_clear": 0.0, "w_gray": 0.0, "w_center": 10.0},
            d_safe_m=0.0,
            corridor_distance_m=corridor_distance,
            corridor_target_m=3.0,
        )
        masked = plan_mode(
            clearance,
            gray_cost,
            (2, 0),
            (2, 4),
            {"w_turn": 0.0, "w_clear": 0.0, "w_gray": 0.0, "w_center": 10.0},
            d_safe_m=0.0,
            corridor_distance_m=corridor_distance,
            corridor_target_m=3.0,
            corridor_weight_map=np.zeros((5, 5), dtype=np.float32),
        )

        self.assertTrue(all(row == 2 for row, _ in shortest))
        self.assertTrue(any(row in (1, 3) for row, _ in centered))
        self.assertTrue(all(row == 2 for row, _ in masked))


if __name__ == "__main__":
    unittest.main()
