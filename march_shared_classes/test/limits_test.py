import unittest
import rosunit

from march_shared_classes.gait.limits import Limits


class LimitsTest(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(0, 1, 2)

    def test_lower_limit(self):
        self.assertEqual(self.limits.lower, 0, "Lower limit not initialised correctly.")

    def test_upper_limit(self):
        self.assertEqual(self.limits.upper, 1, "Upper limit not initialised correctly.")

    def test_velocity_limit(self):
        self.assertEqual(self.limits.velocity, 2, "Velocity limit not initialised correctly.")


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_limits', LimitsTest)
