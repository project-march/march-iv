#!/usr/bin/env python

import unittest

import os
import rospkg
import yaml

from rospy_message_converter import message_converter

from march_gait_selection.GaitSelection import GaitSelection


class TestValidateTrajectoryTransition(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")

        gait_path = os.path.join(rospkg.RosPack().get_path('march_gait_selection'),
                                 "test/correct_walking_gait/walking/walking.gait")
        gait_yaml = yaml.load(open(gait_path), Loader=yaml.SafeLoader)
        self.gait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/GaitGoal',
                                                                        gait_yaml,
                                                                        kind="message")
        self.assertTrue(self.gait_selection.validate_gait(self.gait))



    def test_gait_correct(self):
        self.assertTrue(self.gait_selection.validate_gait(self.gait))

    def test_gait_none(self):
        self.assertFalse(self.gait_selection.validate_gait(None))

    def test_gait_different_graph_lengths(self):
        self.gait.graph.from_subgait.append("start")
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_empty_from_subgait(self):
        self.gait.graph.from_subgait = []
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_empty_to_subgait(self):
        self.gait.graph.to_subgait = []
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_missing_start(self):
        # Equalize lengths so we don't fail that check instead
        self.gait.graph.from_subgait = ["right_open", "left_swing", "right_close"]
        self.gait.graph.to_subgait.pop()
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_missing_end(self):
        # Equalize lengths so we don't fail that check instead
        self.gait.graph.from_subgait.pop()
        self.gait.graph.to_subgait = ["right_open", "left_swing", "right_close"]
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_start_in_to_subgait(self):
        self.gait.graph.from_subgait.append("start")
        self.gait.graph.to_subgait.append("start")
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_end_in_from_subgait(self):
        self.gait.graph.from_subgait.append("end")
        self.gait.graph.to_subgait.append("end")
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_wrong_transition(self):
        self.gait_selection.set_subgait_version("walking", "left_swing", "incompatible_with_right_open_test_a_bit_higher")
        self.assertFalse(self.gait_selection.validate_gait(self.gait))

    def test_gait_by_name_wrong(self):
        self.assertFalse(self.gait_selection.validate_gait_by_name("Wrong name"))

    def test_gait_by_name_correct(self):
        self.assertTrue(self.gait_selection.validate_gait_by_name("walking"))

