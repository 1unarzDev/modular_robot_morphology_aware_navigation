import unittest

from morphology_planner import OccupancyGrid


class PackageSmokeTest(unittest.TestCase):
    def test_constructs_grid(self):
        self.assertEqual(OccupancyGrid(2, 2, 0.1).width, 2)
