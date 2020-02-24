import unittest

from march_shared_classes.gait.limits import Limits


class LimitsTest(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(0, 1, 2, 3, 4, 5)

    def test_lower_limit(self):
        self.assertEqual(self.limits.lower, 0, 'Lower limit not initialised correctly.')

    def test_upper_limit(self):
        self.assertEqual(self.limits.upper, 1, 'Upper limit not initialised correctly.')

    def test_velocity_limit(self):
        self.assertEqual(self.limits.velocity, 2, 'Velocity limit not initialised correctly.')

    def test_effort(self):
        self.assertEqual(self.limits.effort, 3, 'Effort limit not initialised correctly.')

    def test_k_position(self):
        self.assertEqual(self.limits.k_position, 4, 'k_position not initialised correctly.')

    def test_k_velocity(self):
        self.assertEqual(self.limits.k_velocity, 5, 'k_velocity not initialised correctly.')
