#!/usr/bin/env python

import unittest

import rosunit

from march_gait_selection.GaitSelection import GaitSelection
from march_gait_selection.transition_gait.transition_subgait import TransitionSubgait
from march_shared_classes.exceptions.gait_exceptions import GaitError


PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'

walk_medium = 'walk_medium'
walk_small = 'walk_small'

right_open = 'right_open'
right_swing = 'right_swing'
right_close = 'right_close'

wrong_name = 'wrong'

gait_selection = GaitSelection(PKG, DIR)


class TestTransitionTrajectory(unittest.TestCase):
    def test_valid_walk_small_gait(self):
        # check if gait name exists
        is_walk_small_gait = False if gait_selection[walk_small] is None else True
        self.assertTrue(is_walk_small_gait, msg='{gn} gait could not be found in folder'.format(gn=walk_small))

    def test_valid_walk_medium_gait(self):
        # check if gait name exists
        is_walk_medium_medium = False if gait_selection[walk_medium] is None else True
        self.assertTrue(is_walk_medium_medium, msg='{gn} gait could not be found in folder'.format(gn=walk_medium))

    def test_right_open_walk_small(self):
        # check if subgait name exists
        is_right_open_subgait = False if gait_selection[walk_small][right_open] is None else True
        self.assertTrue(is_right_open_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=right_open, gn=walk_small))

    def test_right_open_walk_medium(self):
        # check if subgait name exists
        is_right_open_subgait = False if gait_selection[walk_medium][right_open] is None else True
        self.assertTrue(is_right_open_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=right_open, gn=walk_medium))

    def test_right_swing_in_walk_small(self):
        # check if subgait name exists
        is_right_swing_subgait = False if gait_selection[walk_small][right_swing] is None else True
        self.assertTrue(is_right_swing_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=right_swing, gn=walk_small))

    def test_right_swing_in_walk_medium(self):
        # check if subgait name exists
        is_right_swing_subgait = False if gait_selection[walk_medium][right_swing] is None else True
        self.assertTrue(is_right_swing_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=right_swing, gn=walk_medium))

    def test_right_close_walk_small(self):
        # check if subgait name exists
        is_right_close_subgait = False if gait_selection[walk_small][right_close] is None else True
        self.assertTrue(is_right_close_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=right_close, gn=walk_small))

    def test_right_close_walk_medium(self):
        # check if subgait name exists
        is_right_close_subgait = False if gait_selection[walk_medium][right_close] is None else True
        self.assertTrue(is_right_close_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=right_close, gn=walk_medium))

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

    def test_valid_transition_small_to_medium_right_open(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, right_open)

    def test_valid_transition_medium_to_small_right_open(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_medium, walk_small, right_open)

    def test_valid_transition_small_to_medium_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, right_swing)

    def test_valid_transition_medium_to_small_right_swing(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_medium, walk_small, right_swing)

    def test_valid_transition_small_to_medium_right_close(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_small, walk_medium, right_close)

    def test_valid_transition_medium_to_small_right_close(self):
        #  Test if the TransitionSubgait is created without an error
        TransitionSubgait.from_subgait_names(gait_selection, walk_medium, walk_small, right_close)


if __name__ == '__main__':
    rosunit.unitrun('march_gait_selection', 'transition_gait_test', TestTransitionTrajectory)
