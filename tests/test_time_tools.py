import unittest

from tools.path_creater.timing import format_duration


class TimeToolsTest(unittest.TestCase):
    def test_format_duration_uses_milliseconds_below_one_second(self):
        self.assertEqual(format_duration(0.123456), "123.5 ms")

    def test_format_duration_uses_seconds_at_one_second_or_above(self):
        self.assertEqual(format_duration(1.23456), "1.235 s")


if __name__ == "__main__":
    unittest.main()
