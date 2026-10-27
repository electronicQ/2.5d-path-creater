import subprocess
import tempfile
import unittest
from pathlib import Path


HELPER = Path(__file__).resolve().parents[1] / 'scripts/lib/python_cache.sh'


class DeploymentPythonCacheTest(unittest.TestCase):
    def same_binary(self, cached, expected):
        result = subprocess.run(
            ['bash', '-c', 'source "$1"; vehicle_same_python_binary "$2" "$3"',
             'cache-test', str(HELPER), str(cached), str(expected)])
        return result.returncode == 0

    def test_python2_aliases_and_missing_or_different_python(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            python27 = root / 'python2.7'
            python27.write_text('#!/bin/sh\nexit 0\n')
            python27.chmod(0o755)
            alias = root / 'python'
            alias.symlink_to(python27)
            chained = root / 'python2'
            chained.symlink_to(alias)
            python3 = root / 'python3'
            python3.write_text('#!/bin/sh\nexit 0\n')
            python3.chmod(0o755)
            self.assertTrue(self.same_binary(python27, python27))
            self.assertTrue(self.same_binary(alias, python27))
            self.assertTrue(self.same_binary(chained, python27))
            self.assertFalse(self.same_binary(python3, python27))
            self.assertFalse(self.same_binary(root / 'missing', python27))
            self.assertFalse(self.same_binary(alias, root / 'missing'))


if __name__ == '__main__':
    unittest.main()
