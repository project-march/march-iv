#!/usr/bin/env python

import unittest

import rostest

from march_gait_selection.GaitSelection import GaitSelection
from march_gait_selection.transition_gait.create_transition import TransitionSubgait
from march_shared_classes.gait.gait import Gait

PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'

old_gait_name = 'walk_medium'
new_gait_name = 'walk_small'

before_subgait_name = 'left_swing'
subgait_name = 'right_swing'
after_subgait_name = 'left_close'


class TestTransitionTrajectory(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection(PKG, DIR)
        self.transition_gait = TransitionSubgait(self.gait_selection)

    def test_valid_gait_information(self):
        last_gait = False if self.gait_selection[old_gait_name] is None else True
        new_gait = False if self.gait_selection[new_gait_name] is None else True

        self.assertTrue(last_gait, msg='{gn} gait could not be found in map'.format(gn=old_gait_name))
        self.assertTrue(new_gait, msg='{gn} gait could not be found in map'.format(gn=new_gait_name))

        before_subgait = False if self.gait_selection[old_gait_name][before_subgait_name] is None else True
        after_subgait = False if self.gait_selection[new_gait_name][after_subgait_name] is None else True

        self.assertTrue(before_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=before_subgait_name, gn=old_gait_name))
        self.assertTrue(after_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=after_subgait_name, gn=new_gait_name))

        subgait_small = False if self.gait_selection[old_gait_name][subgait_name] is None else True
        subgait_medium = False if self.gait_selection[new_gait_name][subgait_name] is None else True

        self.assertTrue(subgait_small, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=subgait_name, gn=old_gait_name))
        self.assertTrue(subgait_medium, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=subgait_name, gn=new_gait_name))

    def test_valid_transition_medium_to_small(self):
        subgait_ms = self.transition_gait.create_transition_subgait(old_gait_name, new_gait_name, subgait_name)

        before_subgait = self.gait_selection[old_gait_name][before_subgait_name]
        after_subgait = self.gait_selection[new_gait_name][after_subgait_name]

        subgaits = [before_subgait, subgait_ms, after_subgait]

        from_subgait_names = ['start', before_subgait_name, 'transition_subgait', after_subgait_name]
        to_subgait_names = [before_subgait_name, 'transition_subgait', after_subgait_name, 'end']

        Gait('transition', subgaits, from_subgait_names, to_subgait_names)


if __name__ == '__main__':
    rostest.rosrun('march_gait_selection', 'transition_gait_test', TestTransitionTrajectory)
