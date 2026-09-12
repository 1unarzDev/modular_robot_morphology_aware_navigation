import unittest

import morphology_manager


class PackageSmokeTest(unittest.TestCase):
    def test_imports(self):
        self.assertIsNotNone(morphology_manager)
