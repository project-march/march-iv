#!/usr/bin/env python

import unittest

from march_gait_selection.GaitSelection import GaitSelection


class TestValidateSubgaitTransition(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        self.actual_map = {"walking": {
            "right_open": "test_a_bit_higher", "left_swing": "test", "right_close": "right_close"}}

    def test_transition_correct(self):
        self.assertTrue(self.gait_selection._validate_trajectory_transition("walking", "left_swing", "right_close"))

    def test_transition_wrong_from(self):
        self.assertRaises(KeyError, self.gait_selection._validate_trajectory_transition, "walking", "wrong", "right_close")

    def test_transition_wrong_to(self):
        self.assertRaises(KeyError, self.gait_selection._validate_trajectory_transition, "walking", "left_swing", "wrong")

