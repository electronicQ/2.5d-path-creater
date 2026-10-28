import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from tools.path_creater.map_loader import OccupancyMap
from tools.path_creater.trajectory import write_overlay


class OverlayOrientationTest(unittest.TestCase):
    def test_overlay_matches_pgm_orientation(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "overlay.png"
            omap = OccupancyMap(
                resolution=1.0,
                origin_x=0.0,
                origin_y=0.0,
                origin_yaw=0.0,
                negate=0,
                occupied_thresh=0.65,
                free_thresh=0.25,
                occ=np.array(
                    [
                        [1.0 - 210 / 255.0, 1.0 - 220 / 255.0, 1.0 - 230 / 255.0, 1.0 - 240 / 255.0, 1.0 - 250 / 255.0],
                        [1.0 - 160 / 255.0, 1.0 - 170 / 255.0, 1.0 - 180 / 255.0, 1.0 - 190 / 255.0, 1.0 - 200 / 255.0],
                        [1.0 - 110 / 255.0, 1.0 - 120 / 255.0, 1.0 - 130 / 255.0, 1.0 - 140 / 255.0, 1.0 - 150 / 255.0],
                        [1.0 - 30 / 255.0, 1.0 - 40 / 255.0, 1.0 - 50 / 255.0, 1.0 - 60 / 255.0, 1.0 - 70 / 255.0],
                        [0.0, 1.0 - 20 / 255.0, 1.0 - 80 / 255.0, 1.0 - 90 / 255.0, 1.0 - 100 / 255.0],
                    ],
                    dtype=np.float32,
                ),
                gray=np.array(
                    [
                        [210, 220, 230, 240, 250],
                        [160, 170, 180, 190, 200],
                        [110, 120, 130, 140, 150],
                        [30, 40, 50, 60, 70],
                        [255, 20, 80, 90, 100],
                    ],
                    dtype=np.uint8,
                ),
                image_path=Path("dummy.pgm"),
                yaml_path=Path("dummy.yaml"),
            )
            free = np.ones((5, 5), dtype=bool)

            write_overlay(omap, free, {"distance": [(0, 0)]}, out, (4, 0), (0, 4))

            img = Image.open(out).convert("RGB")
            self.assertEqual(img.getpixel((0, 0)), (255, 255, 0))
            self.assertEqual(img.getpixel((0, 4)), (220, 40, 40))

    def test_tracking_points_are_not_connected_by_lines(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "points.png"
            gray = np.full((10, 10), 200, dtype=np.uint8)
            omap = OccupancyMap(
                resolution=1.0,
                origin_x=0.0,
                origin_y=0.0,
                origin_yaw=0.0,
                negate=0,
                occupied_thresh=0.65,
                free_thresh=0.25,
                occ=(255.0 - gray) / 255.0,
                gray=gray,
                image_path=Path("dummy.pgm"),
                yaml_path=Path("dummy.yaml"),
            )

            write_overlay(
                omap,
                np.ones((10, 10), dtype=bool),
                {"distance": [(0, 0), (0, 9)]},
                out,
                (9, 0),
                (9, 1),
            )

            image = Image.open(out).convert("RGB")
            self.assertNotEqual(image.getpixel((0, 4)), (220, 40, 40))


if __name__ == "__main__":
    unittest.main()
