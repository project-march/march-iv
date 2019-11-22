#!/usr/bin/env python

import unittest
import rosunit

from march_gain_scheduling.interpolate_pid_values import interpolate

class InterpolatePidValuesTest(unittest.TestCase):
    def test_interpolate_up(self):
        input = [0, 0, 0]
        output = [1, 1, 1]
        result = interpolate(input, output, 1, 0.1)
        self.assertEqual(result, [0.1, 0.1, 0.1])

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_multiply', InterpolatePidValuesTest)