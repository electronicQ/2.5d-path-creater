import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from tools.path_creater.map_loader import OccupancyMap
from tools.path_creater.trajectory import write_overlay


class OverlayOrientationTest(unittest.TestCase):
    @staticmethod
    def make_map(size=20):
        gray = np.full((size, size), 200, dtype=np.uint8)
        return OccupancyMap(
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            negate=0,
            occupied_thresh=0.65,
            free_thresh=0.25,
            occ=((255.0 - gray) / 255.0).astype(np.float32),
            gray=gray,
            image_path=Path("dummy.pgm"),
            yaml_path=Path("dummy.yaml"),
        )

    def test_overlay_matches_pgm_orientation(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "overlay.png"
            omap = self.make_map()
            write_overlay(omap, np.ones((20, 20), dtype=bool), {"distance": [(0, 0)]}, out, (19, 0), (0, 19))

            img = Image.open(out).convert("RGB")
            self.assertEqual(img.getpixel((0, 19)), (220, 40, 40))

    def test_tracking_points_and_thin_connecting_lines_are_visible(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "points.png"
            omap = self.make_map()

            write_overlay(
                omap,
                np.ones((20, 20), dtype=bool),
                {"distance": [(0, 5), (19, 5)]},
                out,
                (19, 0),
                (19, 1),
            )

            image = Image.open(out).convert("RGB")
            self.assertEqual(image.getpixel((5, 10)), (220, 40, 40))

    def test_overlapping_markers_keep_all_three_colors_visible(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "overlap.png"
            omap = self.make_map()
            point = (10, 10)
            write_overlay(
                omap,
                np.ones((20, 20), dtype=bool),
                {
                    "distance": [point],
                    "smooth": [point],
                    "cost_aware": [point],
                },
                out,
                (19, 0),
                (19, 1),
            )

            image = Image.open(out).convert("RGB")
            colors = {
                image.getpixel((x, y))
                for x in range(5, 16)
                for y in range(4, 15)
            }
            self.assertIn((220, 40, 40), colors)
            self.assertIn((40, 160, 40), colors)
            self.assertIn((40, 80, 220), colors)


if __name__ == "__main__":
    unittest.main()
