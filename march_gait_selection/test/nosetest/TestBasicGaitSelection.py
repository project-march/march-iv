#!/usr/bin/env python
import os
import unittest

import rospkg

from march_gait_selection.GaitSelection import GaitSelection


PKG = 'march_gait_selection'


class TestBasicGaitSelection(unittest.TestCase):
    def test_gait_selection_creation_from_yaml(self):
        gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')

        actual_map = {
            'walking': {
                'right_open': 'test_a_bit_higher',
                'left_swing': 'test',
                'right_close': 'right_close',
            },
        }

        self.assertEquals(actual_map, gait_selection.gait_version_map)
        self.assertEquals(gait_selection.gait_directory, os.path.join(
            rospkg.RosPack().get_path('march_gait_selection'), 'test/correct_walking_gait'))

    def test_gait_selection_wrong_package(self):
        with self.assertRaises(Exception) as context:
            GaitSelection('definitely_a_wrong_package', 'wrong_directory')
        self.assertEqual('Could not find package definitely_a_wrong_package, Shutting down.', str(context.exception))

    def test_gait_selection_wrong_directory(self):
        with self.assertRaises(Exception) as context:
            GaitSelection('march_gait_selection', 'test/wrong_directory')
        self.assertEqual('Could not find march_gait_selection/test/wrong_directory/default.yaml, Shutting down.',
                         str(context.exception))
