import unittest

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_classes.exceptions.gait_exceptions import *
from march_shared_classes.exceptions.general_exceptions import *
from march_shared_classes.gait.subgait import Subgait


class TestGetSubgait(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        self.actual_map = {"walking": {
            "right_open": "test_a_bit_higher", "left_swing": "test", "right_close": "right_close"}}

    def test_get_correct_subgait(self):
        self.assertEqual(Subgait, type(self.gait_selection.get_subgait_from_loaded_subgaits('walking', 'left_swing')))

    def test_wrong_names(self):
        self.assertRaises(KeyError, self.gait_selection.get_subgait_from_loaded_subgaits, 'walking', 'wrong')
        self.assertRaises(KeyError, self.gait_selection.get_subgait_from_loaded_subgaits, 'wrong', 'left_swing')

        self.assertRaises(GaitNameNotFound, self.gait_selection.validate_gaits_in_map, 'wrong', 'left_swing')
        self.assertRaises(SubgaitNameNotFound, self.gait_selection.validate_gaits_in_map, 'walking', 'wrong')

    def test_get_subgait_incorrect_version(self):
        self.gait_selection.gait_version_map["walking"]["right_close"] = "wrong_version"
        self.assertRaises(FileNotFoundError, self.gait_selection.get_subgait_path, "walking", "right_close")

    def test_set_subgait(self):
        subgait = self.gait_selection.get_subgait_from_loaded_subgaits('walking', 'right_close')
        self.assertEquals('right_close', subgait.version)

        self.gait_selection.set_subgait_version('walking', 'right_close', 'not_the_default')
        subgait = self.gait_selection.get_subgait_from_loaded_subgaits('walking', 'right_close')
        self.assertEquals('not_the_default', subgait.version)

    def test_set_subgait_wrong(self):
        subgait = self.gait_selection.get_subgait_from_loaded_subgaits('walking', 'right_close')
        self.assertEquals('right_close', subgait.version)

        self.gait_selection.set_subgait_version('walking', 'right_close', 'a_wrong_subgait')
        subgait = self.gait_selection.get_subgait_from_loaded_subgaits('walking', 'right_close')

        self.assertEquals('right_close', subgait.version)
