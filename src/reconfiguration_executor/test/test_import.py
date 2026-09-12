import unittest

from reconfiguration_executor.control import wrap_angle


class PackageSmokeTest(unittest.TestCase):
    def test_control_imports(self):
        self.assertEqual(wrap_angle(0.0), 0.0)
