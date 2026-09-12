import unittest

from modular_robot_bringup.kinematics import BodyTwist


class PackageSmokeTest(unittest.TestCase):
    def test_kinematics_imports(self):
        self.assertEqual(BodyTwist().x, 0.0)
