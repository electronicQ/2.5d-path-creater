import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
GRID_MAP_DEMOS = ROOT / "src" / "grid_map" / "grid_map_demos"
FILTERS_LAUNCH = GRID_MAP_DEMOS / "launch" / "filters_demo_gai.launch"


def loaded_filter_chain_path():
    launch = ET.parse(FILTERS_LAUNCH).getroot()
    for rosparam in launch.iter("rosparam"):
        if rosparam.attrib.get("command") != "load":
            continue
        file_value = rosparam.attrib.get("file", "")
        if "filter_chain" not in file_value:
            continue
        return Path(file_value.replace("$(find grid_map_demos)", str(GRID_MAP_DEMOS)))
    raise AssertionError("filters_demo_gai.launch does not load a filter chain yaml")


class GridMapFilterChainTest(unittest.TestCase):
    def test_current_filter_chain_restores_unknown_mask_after_traversability_thresholds(self):
        chain_path = loaded_filter_chain_path()
        with open(chain_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        filters = config["grid_map_filters"]
        restore_indices = [
            i
            for i, item in enumerate(filters)
            if item.get("type") == "gridMapFilters/RestoreUnknownMaskFilter"
        ]
        self.assertTrue(
            restore_indices,
            "{} must restore original elevation NaNs before OccupancyGrid conversion".format(chain_path),
        )

        restore_index = restore_indices[-1]
        upper_index = next(
            i for i, item in enumerate(filters) if item.get("name") == "traversability_upper_threshold"
        )
        self.assertGreater(restore_index, upper_index)

        restore = filters[restore_index]
        self.assertEqual(restore.get("name"), "restore_unknown_mask")
        self.assertEqual(
            restore.get("params"),
            {
                "mask_layer": "elevation",
                "input_layer": "traversability",
                "output_layer": "traversability",
            },
        )


if __name__ == "__main__":
    unittest.main()
