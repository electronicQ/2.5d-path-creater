import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.pgm_saver import save_continuous_map
from tools.path_creater import main as path_creater_main


ROOT = Path(__file__).resolve().parents[1]
PCL_LAUNCH = ROOT / "src" / "grid_map" / "grid_map_pcl" / "launch" / "grid_map_pcl_loader_node.launch"


def launch_arg_defaults():
    launch = ET.parse(PCL_LAUNCH).getroot()
    return {arg.attrib["name"]: arg.attrib.get("default", "") for arg in launch.iter("arg")}


class ProjectPathConventionTest(unittest.TestCase):
    def test_path_creater_default_config_is_inside_tool_config_dir(self):
        expected = ROOT / "tools" / "path_creater" / "config" / "config.yaml"
        self.assertEqual(path_creater_main.default_config_path(), expected)

    def test_path_creater_banfu2_metadata_exists_in_tool_config_dir(self):
        expected = ROOT / "tools" / "path_creater" / "config" / "banfu2.meta.yaml"
        self.assertTrue(expected.is_file())

    def test_grid_map_pcl_defaults_read_from_project_map_pcd(self):
        defaults = launch_arg_defaults()
        self.assertEqual(defaults["folder_path"], "$(find grid_map_pcl)/../../../map/pcd")
        self.assertEqual(defaults["pcd_filename"], "scans_Global_ENU.pcd")
        self.assertIn("../../../map/pcd", defaults["output_grid_map"])

    def test_map_saver_default_output_base_is_project_map_pgm(self):
        expected = ROOT / "map" / "pgm" / "map"
        self.assertEqual(save_continuous_map.default_output_base(), expected)

    def test_map_saver_resolves_bare_names_under_project_map_pgm(self):
        expected = ROOT / "map" / "pgm" / "campus"
        self.assertEqual(save_continuous_map.resolve_output_base("campus"), expected)

    def test_map_saver_preserves_explicit_relative_paths(self):
        expected = (ROOT / "custom" / "campus").resolve()
        self.assertEqual(save_continuous_map.resolve_output_base("custom/campus"), expected)


if __name__ == "__main__":
    unittest.main()
