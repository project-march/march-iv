#!/usr/bin/env python

import unittest
from march_gait_selection.GaitSelection import GaitSelection


class TestGetSubgait(unittest.TestCase):

    def setUp(self):
        self.gait_directory = "test/gaits_correct_walking_gait"

        self.actual_map_correct = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "left_swing": "test",
                        "right_close": "right_close"
                    }
            }

        self.actual_map_wrong = \
            {
                "walking":
                    {
                        "right_open": "test_a_bit_higher",
                        "left_swing": "test",
                    }
            }

    def test_get_subgait_correct(self):
        gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        subgait = gait_selection.get_subgait('walking', 'left_swing')
        self.assertEquals('left_swing', subgait.name)
        self.assertEquals('test', subgait.version)

    def test_get_subgait_wrong(self):
        gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        del gait_selection.gait_version_map["walking"]["right_close"]
        gait_selection.load_subgait_files()
        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals(None, subgait)

    def test_get_subgait_incorrect_version(self):
        gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        gait_selection.gait_version_map["walking"]["right_close"] = "wrong_version"
        self.assertRaises(KeyError, gait_selection.get_subgait_path, "walking", "right_close")

    def test_set_subgait(self):
        gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")

        subgait = gait_selection.get_subgait('walking', 'right_close')

        self.assertEquals('right_close', subgait.version)
        gait_selection.set_subgait_version('walking', 'right_close', 'not_the_default')

        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals('not_the_default', subgait.version)

    def test_set_subgait_wrong(self):
        gait_selection = GaitSelection('march_gait_selection', "test/correct_walking_gait")
        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals('right_close', subgait.version)
        gait_selection.set_subgait_version('walking', 'right_close', 'a_wrong_subgait')

        # Check the version hasn't changed
        subgait = gait_selection.get_subgait('walking', 'right_close')
        self.assertEquals('right_close', subgait.version)
