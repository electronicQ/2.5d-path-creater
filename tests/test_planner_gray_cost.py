import unittest

import numpy as np

from tools.path_creater.clearance import ClearanceMap
from tools.path_creater.planner import plan_mode


class PlannerGrayCostTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
