import unittest

from src.gnss_to_enu.scripts.gnss_timestamp_compensator import compensated_stamp_seconds


class GNSSTimestampCompensatorTest(unittest.TestCase):
    def test_subtracts_epoch_offset_from_original_stamp(self):
        adjusted = compensated_stamp_seconds(1782701507.671617, 1782701065.9716315)
        self.assertAlmostEqual(adjusted, 441.6999855, places=6)

    def test_rejects_negative_adjusted_stamp(self):
        with self.assertRaises(ValueError):
            compensated_stamp_seconds(10.0, 20.0)


if __name__ == "__main__":
    unittest.main()
