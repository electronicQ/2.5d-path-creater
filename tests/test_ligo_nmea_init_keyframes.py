import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LIGO = ROOT / "src" / "LIGO."


class LigoNmeaInitKeyframeTest(unittest.TestCase):
    def test_avia_handsfree_config_declares_init_keyframe_distance(self):
        with open(LIGO / "config" / "avia_handsfree.yaml", "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.assertIn("init_svd_min_distance", config["nmea"])
        self.assertGreater(config["nmea"]["init_svd_min_distance"], 0.0)

    def test_ligo_reads_init_keyframe_distance_parameter(self):
        parameters = (LIGO / "src" / "parameters.cpp").read_text(encoding="utf-8")
        self.assertIn("nmea/init_svd_min_distance", parameters)
        self.assertIn("init_svd_min_distance", parameters)

    def test_nmea_initialization_skips_non_keyframes_before_svd(self):
        source = (LIGO / "src" / "NMEA_Processing_fg.cpp").read_text(encoding="utf-8")
        self.assertIn("shouldAcceptInitKeyframe", source)
        self.assertIn("Skip non-keyframe", source)


if __name__ == "__main__":
    unittest.main()
