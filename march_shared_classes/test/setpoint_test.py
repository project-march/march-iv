import unittest
import rosunit

from march_shared_classes.gait.setpoint import Setpoint


class SetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = Setpoint(0, 1, 2)

    def test_lower_limit(self):
        self.assertEqual(self.limits.lower, 0, "Lower limit not initialised correctly.")

    def test_upper_limit(self):
        self.assertEqual(self.limits.upper, 1, "Upper limit not initialised correctly.")

    def test_velocity_limit(self):
        self.assertEqual(self.limits.velocity, 2, "Velocity limit not initialised correctly.")

    def test_velocity_repr(self):
        self.assertEqual(str(self.setpoint),
                         "Time: 0, Position: 1, Velocity: 2",
                         "Velocity limit not initialised correctly.")


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_setpoint', SetpointTest)
