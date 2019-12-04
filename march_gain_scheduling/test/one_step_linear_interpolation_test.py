#!/usr/bin/env python
import unittest

import rosunit

from march_gain_scheduling.interpolation_errors import NegativeValueError, UnequalLengthError
from march_gain_scheduling.one_step_linear_interpolation import interpolate

PKG = 'march_gain_scheduling'


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
        result = interpolate(current, needed, 3, 0.1)
        self.assertEqual(result, [2.3, 4.7, 5.7])

    def test_interpolate_list_length(self):
        current = [2, 5, 6, 8]
        needed = [4, 3, 4]
        with self.assertRaises(UnequalLengthError):
            interpolate(current, needed, 1, 0.1)

    def test_negative_value_error(self):
        current = [2, 5, 6]
        needed = [4, 3, 4]
        with self.assertRaises(NegativeValueError):
            interpolate(current, needed, -1, 0.1)

    def test_interpolate_different_paths(self):
        current = [1, 1, 1]
        needed = [2, 5, 4]
        for i in range(26):  # Loop until needed is reached and a bit further
            current = interpolate(current, needed, 1, 0.2)
        self.assertEqual(current, [2, 5, 4])

    def test_interpolate_empty_lists(self):
        current = []
        needed = []
        result = interpolate(current, needed, 1, 0.1)
        self.assertEqual(result, [])


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_multiply', OneStepLinearInterpolationTest)
