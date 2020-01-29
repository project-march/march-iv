import unittest

from march_shared_classes.gait.setpoint import Setpoint


class SetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = Setpoint(1.123412541, 0.0343412512, 123.162124)

    def test_time_rounding(self):
        self.assertEqual(self.setpoint.time, 1.1234)

    def test_position_rounding(self):
        self.assertEqual(self.setpoint.position, 0.0343)

    def test_velocity_rounding(self):
        self.assertEqual(self.setpoint.velocity, 123.1621)

    def test_string_output(self):
        self.assertEqual(str(self.setpoint), 'Time: %s, Position: %s, Velocity: %s' % (1.1234, 0.0343, 123.1621))
