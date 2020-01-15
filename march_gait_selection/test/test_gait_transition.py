#!/usr/bin/env python

import unittest

import rostest

from march_gait_selection.create_transition import TransitionSubgait
from march_gait_selection.GaitSelection import GaitSelection
from march_shared_classes.gait.gait import Gait

PKG = 'march_gait_selection'
DIR = 'test/testing_gait_files'

last_gait_name = 'walk_medium'
new_gait_name = 'walk_small'

last_subgait_name = 'left_swing'
new_subgait_name = 'right_swing'


class TestTransitionTrajectory(unittest.TestCase):
    def setUp(self):
        self.gait_selection = GaitSelection(PKG, DIR)

    def test_valid_gait_information(self):
        last_gait = False if self.gait_selection[last_gait_name] is None else True
        new_gait = False if self.gait_selection[new_gait_name] is None else True

        self.assertTrue(last_gait, msg='{gn} gait could not be found in map'.format(gn=last_gait_name))
        self.assertTrue(new_gait, msg='{gn} gait could not be found in map'.format(gn=new_gait_name))

        last_subgait = False if self.gait_selection[last_gait_name][last_subgait_name] is None else True
        new_subgait = False if self.gait_selection[new_gait_name][new_subgait_name] is None else True

        self.assertTrue(last_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=last_subgait_name, gn=last_gait_name))
        self.assertTrue(new_subgait, msg='{sg} subgait could not be found in gait {gn}'
                        .format(sg=new_subgait_name, gn=new_gait_name))

    def test_valid_transition_medium_to_small(self):
        last_gait = self.gait_selection[last_gait_name]
        new_gait = self.gait_selection[new_gait_name]

        last_subgait = last_gait[last_subgait_name]
        new_subgait = new_gait[new_subgait_name]

        transition_subgait_ms = TransitionSubgait.create_transition_subgait(self.gait_selection, last_gait_name,
                                                                            new_gait_name, last_subgait_name)

        subgaits = [last_subgait, transition_subgait_ms, new_subgait]

        from_subgait_names = ['start', last_subgait_name, 'transition_subgait', new_subgait_name]
        to_subgait_names = [last_subgait_name, 'transition_subgait', new_subgait_name, 'end']

        Gait('transition', subgaits, from_subgait_names, to_subgait_names)


if __name__ == '__main__':
    rostest.rosrun('march_gait_selection', 'transition_gait_test', TestTransitionTrajectory)
