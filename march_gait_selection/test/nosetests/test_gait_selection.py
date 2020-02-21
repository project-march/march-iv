#!/usr/bin/env python

import unittest

from march_gait_selection.GaitSelection import GaitSelection


PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'

walk_medium = 'walk_medium'
walk_small = 'walk_small'

right_open = 'right_open'
right_swing = 'right_swing'
right_close = 'right_close'

wrong_name = 'wrong'

gait_selection = GaitSelection(PKG, DIR)


class TestGaitSelection(unittest.TestCase):
    def test_valid_walk_small_gait(self):
        # check if gait name exists
        is_walk_small_gait = False if gait_selection[walk_small] is None else True
        self.assertTrue(is_walk_small_gait, msg='{gn} gait could not be found in folder'.format(gn=walk_small))

    def test_valid_walk_medium_gait(self):
        # check if gait name exists
        is_walk_medium_medium = False if gait_selection[walk_medium] is None else True
        self.assertTrue(is_walk_medium_medium, msg='{gn} gait could not be found in folder'.format(gn=walk_medium))

    def test_invalid_gait_name(self):
        # check if wrong gait name
        self.assertEqual(None, gait_selection[wrong_name], msg='Gait selection class does not return None when given '
                                                               'a false gait name')

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

    def test_invalid_subgait_name(self):
        # check if wrong gait name
        self.assertEqual(None, gait_selection[walk_medium][wrong_name], msg='Gait selection class does not return None '
                                                                            'when given a false subgait name')

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
