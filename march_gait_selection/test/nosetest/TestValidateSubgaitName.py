#!/usr/bin/env python
import unittest

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_classes.exceptions.gait_exceptions import *


class TestValidateSubgaitName(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        self.actual_map = {"walking": {
            "right_open": "test_a_bit_higher", "left_swing": "test", "right_close": "right_close"}}

    def test_validate_subgait_name_correct(self):
        self.assertTrue(self.gait_selection.validate_gaits_in_map("walking", "right_open"))
        self.assertTrue(self.gait_selection.validate_gaits_in_map("walking", "right_close"))
        self.assertTrue(self.gait_selection.validate_gaits_in_map("walking", "left_swing"))

    def test_validate_subgait_name_wrong_subgait(self):
        self.assertRaises(SubgaitNameNotFound, self.gait_selection.validate_gaits_in_map, "walking", "very_wrong_indeed")

    def test_validate_subgait_name_wrong_gait(self):
        self.assertRaises(GaitNameNotFound, self.gait_selection.validate_gaits_in_map, "not_even_close", "very_wrong_indeed")
