#!/usr/bin/env python

from copy import deepcopy
import unittest

from march_gait_selection.GaitSelection import GaitSelection
from march_gait_selection.transition_gait.transition_subgait import TransitionSubgait
from march_shared_classes.exceptions.gait_exceptions import GaitError


PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'

walk_medium = 'walk_medium'
walk_small = 'walk_small'
stairs_up = 'stairs_up'

right_open = 'right_open'
right_swing = 'right_swing'
right_close = 'right_close'

wrong_name = 'wrong'

gait_selection = GaitSelection(PKG, DIR)


class TestTransitionTrajectory(unittest.TestCase):
    def test_invalid_old_gait_name(self):
        # check if wrong gait name causes right error
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(gait_selection, wrong_name, walk_medium, right_swing)

    def test_invalid_new_gait_name(self):
        # check if wrong gait name causes right error
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(gait_selection, walk_small, wrong_name, right_swing)

    def test_invalid_subgait_name(self):
        # check if wrong subgait name causes right error
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, wrong_name)

    def test_invalid_gait_selection(self):
        # check if wrong gait selection module
        with self.assertRaises(GaitError):
            TransitionSubgait.from_subgait_names(wrong_name, walk_small, walk_medium, right_swing)

    def test_scale_setpoints_when_new_duration_is_larger(self):
        # check if the scaling of the setpoints works with a large scaling factor
        subgait_walk_medium = deepcopy(gait_selection[walk_medium][right_swing])
        scaling_factor = 45.453

        scaled_subgait_walk_medium = TransitionSubgait._scale_timestamps_subgaits(subgait_walk_medium, scaling_factor)

        for joint in scaled_subgait_walk_medium.joints:
            self.assertEqual(joint.setpoints[-1].time, scaling_factor,
                             msg='Scaled end point {ep} does not match scaling factor {sf} after scaling'
                             .format(ep=joint.setpoints[-1].time, sf=scaling_factor))

    def test_scale_setpoints_when_new_duration_is_smaller(self):
        # check if the scaling of the setpoints works with a small scaling factor
        subgait_walk_medium = deepcopy(gait_selection[walk_medium][right_swing])
        scaling_factor = 0.123

        scaled_subgait_walk_medium = TransitionSubgait._scale_timestamps_subgaits(subgait_walk_medium, scaling_factor)

        for joint in scaled_subgait_walk_medium.joints:
            self.assertEqual(joint.setpoints[-1].time, scaling_factor,
                             msg='Scaled end point {ep} does not match scaling factor {sf} after scaling'
                             .format(ep=joint.setpoints[-1].time, sf=scaling_factor))

    def test_equalize_amount_of_setpoints_with_lower_duration_new_gait(self):
        # test if interpolation works with higher duration and different setpoints
        subgait_walk_medium = deepcopy(gait_selection[walk_medium][right_swing])
        subgait_stairs_up = deepcopy(gait_selection[stairs_up][right_swing])

        subgait_walk_medium = TransitionSubgait._scale_timestamps_subgaits(subgait_stairs_up,
                                                                           subgait_walk_medium.duration)

        TransitionSubgait._equalize_amount_of_setpoints(subgait_stairs_up, subgait_walk_medium)

    def test_equalize_amount_of_setpoints_with_higher_duration_new_gait(self):
        # test if interpolation works with higher duration and different setpoints
        subgait_walk_medium = deepcopy(gait_selection[walk_medium][right_swing])
        subgait_stairs_up = deepcopy(gait_selection[stairs_up][right_swing])

        subgait_walk_medium = TransitionSubgait._scale_timestamps_subgaits(subgait_walk_medium,
                                                                           subgait_stairs_up.duration)

        TransitionSubgait._equalize_amount_of_setpoints(subgait_walk_medium, subgait_stairs_up)

    def test_walk_transition_small_to_medium_right_open(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, right_open)

    def test_walk_transition_medium_to_small_right_open(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_medium, walk_small, right_open)

    def test_walk_transition_small_to_medium_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, right_swing)

    def test_walk_transition_medium_to_small_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_medium, walk_small, right_swing)

    def test_walk_transition_small_to_medium_right_close(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, right_close)

    def test_walk_transition_medium_to_small_right_close(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_medium, walk_small, right_close)

    def test_transition_walk_small_stairs_up_right_open(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, stairs_up, right_open)

    def test_transition_walk_small_stairs_up_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, stairs_up, right_swing)

    def test_transition_walk_small_stairs_up_right_close(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, stairs_up, right_close)
