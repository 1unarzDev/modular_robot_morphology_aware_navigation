import unittest

import modular_robot_benchmarks


class PackageSmokeTest(unittest.TestCase):
    def test_imports(self):
        self.assertIsNotNone(modular_robot_benchmarks)
