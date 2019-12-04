
import unittest

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_classes.exceptions.general_exceptions import *


class TestBasicGaitSelection(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        self.actual_map = {"walking": {
            "right_open": "test_a_bit_higher", "left_swing": "test", "right_close": "right_close"}}

    def test_gait_selection_creation_from_yaml(self):
        self.assertEquals(self.actual_map, self.gait_selection.gait_version_map)

    def test_gait_selection_wrong_package(self):
        self.assertRaises(PackageNotFoundError, GaitSelection, "definitely_a_wrong_package", "wrong_directory")

    def test_gait_selection_wrong_directory(self):
        self.assertRaises(FileNotFoundError, GaitSelection, "march_gait_selection", "test/wrong_directory")

