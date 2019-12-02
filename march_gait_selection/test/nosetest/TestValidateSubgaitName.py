#!/usr/bin/env python
import unittest

from march_gait_selection.GaitSelection import GaitSelection


class TestValidateSubgaitName(unittest.TestCase):

    gait_directory = 'test/gaits_correct_walking_gait'

    actual_map_correct = \
        {
            'walking':
                {
                    'right_open': 'test_a_bit_higher',
                    'left_swing': 'test',
                    'right_close': 'right_close',
                },
        }

    gait_selection = GaitSelection('march_gait_selection', 'test/correct_walking_gait')

    def test_validate_subgait_name_correct(self):
        self.assertTrue(self.gait_selection.validate_subgait_name('walking', 'right_open'))
        self.assertTrue(self.gait_selection.validate_subgait_name('walking', 'right_close'))
        self.assertTrue(self.gait_selection.validate_subgait_name('walking', 'left_swing'))

    def test_validate_subgait_name_wrong_subgait(self):
        self.assertFalse(self.gait_selection.validate_subgait_name('walking', 'very_wrong_indeed'))

    def test_validate_subgait_name_wrong_gait(self):
        self.assertFalse(self.gait_selection.validate_subgait_name('not_even_close', 'very_wrong_indeed'))
