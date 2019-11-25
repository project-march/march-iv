#!/usr/bin/env python

import unittest
import rosunit

from march_gain_scheduling.one_step_linear_interpolation import interpolate
from march_gain_scheduling.unequal_length_error import UnequalLengthError

PKG = 'march_gain_scheduling'
TEST = 'test_interpolation'

class OneStepLinearInterpolationTest(unittest.TestCase):
    def test_interpolate_up(self):
        current = [0, 0, 0]
        needed = [1, 1, 1]
        result = interpolate(current, needed, 1, 0.1)
        self.assertEqual(result, [0.1, 0.1, 0.1])

    def test_interpolate_down(self):
        current = [5, 5, 5]
        needed = [3, 3, 3]
        result = interpolate(current, needed, 1, 0.1)
        self.assertEqual(result, [4.9, 4.9, 4.9])

    def test_interpolate_up_and_down(self):
        current = [2, 5, 6]
        needed = [4, 3, 4]
        for i in range(25):
            current = interpolate(current, needed, 3, 0.1)
        self.assertEqual(current, [4, 3, 4])

    def test_interpolate_list_length(self):
        current = [2, 5, 6, 8]
        needed = [4, 3, 4]
        with self.assertRaises(UnequalLengthError):
            interpolate(current, needed, 1, 0.1)

    def test_interpolate_different_paths(self):
        current = [1, 1, 1]
        needed = [2, 5, 4]
        for i in range(30):
            current = interpolate(current, needed, 1, 0.2)
        self.assertEqual(current, [2, 5, 4])


if __name__ == '__main__':
    rosunit.unitrun(PKG, TEST, OneStepLinearInterpolationTest)
