#!/usr/bin/env python
import os
import unittest

import rospkg
from rospy_message_converter import message_converter
import yaml

from march_gait_selection.GaitSelection import GaitSelection


class TestValidateSubgaitTransition(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')

        gait_path = os.path.join(rospkg.RosPack().get_path('march_gait_selection'),
                                 'test/correct_walking_gait/walking/walking.gait')
        gait_yaml = yaml.load(open(gait_path), Loader=yaml.SafeLoader)
        self.gait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/GaitGoal',
                                                                        gait_yaml,
                                                                        kind='message')
        self.assertTrue(self.gait_selection.validate_gait(self.gait))

    def test_transition_correct(self):
        self.assertTrue(self.gait_selection.validate_subgait_transition('walking', 'left_swing', 'right_close'))

    def test_transition_wrong_from(self):
        self.assertFalse(self.gait_selection.validate_subgait_transition('walking', 'wrong', 'right_close'))

    def test_transition_wrong_to(self):
        self.assertFalse(self.gait_selection.validate_subgait_transition('walking', 'left_swing', 'wrong'))

    def test_transition_wrong_to_from_start(self):
        self.assertFalse(self.gait_selection.validate_subgait_transition('walking', 'start', 'wrong'))

    def test_transition_wrong_from_to_end(self):
        self.assertFalse(self.gait_selection.validate_subgait_transition('walking', 'wrong', 'end'))
