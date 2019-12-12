#!/usr/bin/env python
import os
import unittest

import rospkg

from march_gait_selection.GaitSelection import GaitSelection


class TestScanDirectory(unittest.TestCase):

    gait_directory = 'test/correct_walking_gait'

    gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')
    absolute_gait_directory = os.path.join(rospkg.RosPack().get_path('march_gait_selection'), gait_directory)

    def test_validate_scan_directory(self):
        self.maxDiff = None

        directory = {
            'walking': {
                'image': self.absolute_gait_directory + '/walking/walking.png',
                'subgaits': {
                    'right_open': ['test_a_bit_higher'],
                    'right_close': ['not_the_default', 'right_close'],
                    'left_swing': ['incompatible_with_right_open_test_a_bit_higher', 'test'],
                },
            },
        }
        self.assertEquals(directory, self.gait_selection.scan_directory())
