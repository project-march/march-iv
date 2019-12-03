#!/usr/bin/env python
import unittest

from march_gait_selection.GaitSelection import GaitSelection


class TestValidateVersionMap(unittest.TestCase):

    gait_directory = 'test/gaits_correct_walking_gait'

    def test_validate_version_map_correct(self):
        gait_map = \
            {
                'walking':
                    {
                        'right_open': 'test_a_bit_higher',
                        'left_swing': 'test',
                        'right_close': 'right_close',
                    },
            }
        gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')
        self.assertTrue(gait_selection.validate_version_map(gait_map))

    def test_validate_version_map_wrong_gait(self):
        gait_map = \
            {
                'walking':
                    {
                        'right_open': 'test_a_bit_higher',
                        'left_swing': 'test',
                        'right_close': 'right_close',
                    },
                'wrong':
                    {
                        'right_open': 'test_a_bit_higher',
                    },
            }
        gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')
        self.assertFalse(gait_selection.validate_version_map(gait_map))

    def test_validate_version_map_wrong_subgait(self):
        gait_map = \
            {
                'walking':
                    {
                        'right_open': 'test_a_bit_higher',
                        'wrong': 'test',
                        'right_close': 'right_close',
                    },
            }
        gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')
        self.assertFalse(gait_selection.validate_version_map(gait_map))

    def test_validate_version_map_wrong_version(self):
        gait_map = \
            {
                'walking':
                    {
                        'right_open': 'non_existant',
                        'left_swing': 'test',
                        'right_close': 'right_close',
                    },
            }
        gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')
        self.assertFalse(gait_selection.validate_version_map(gait_map))
