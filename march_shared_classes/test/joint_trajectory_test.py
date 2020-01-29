import unittest
import rosunit

from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.setpoint import Setpoint


class JointTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.joint_name = "test_joint"
        self.limits = Limits(-1, 1, 2)
        self.duration = 2
        self.times = [0, 1, 2]
        self.setpoints = [Setpoint(t, 2 * t, t / 2) for t in self.times]
        self.joint_trajectory = JointTrajectory(self.joint_name, self.limits, self.setpoints, self.duration)

    def test_get_setpoints_unzipped_time(self):
        time, position, velocity = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(time, [t for t in self.times])

    def test_get_setpoints_unzipped_position(self):
        time, position, velocity = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(position, [2 * t for t in self.times])

    def test_get_setpoints_unzipped_velocity(self):
        time, position, velocity = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(velocity, [t / 2 for t in self.times])

    def test_valid_joint_transition(self):
        next_setpoints = [Setpoint(t, 2 * t, t / 2) for t in reversed(self.times)]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertTrue(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_valid_joint_transition_unequal_time_zero_speed(self):
        next_setpoints = [Setpoint(t, 2 * t, t / 2) for t in reversed(self.times)]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertTrue(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_position(self):
        next_setpoints = [Setpoint(t, t, t / 2) for t in reversed(self.times)]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_velocity(self):
        next_setpoints = [Setpoint(t, 2 * t, t) for t in reversed(self.times)]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_unequal_time_nonzero_speed(self):
        next_setpoints = [Setpoint(0.5 * t + 1, 2 * t, t / 2) for t in reversed(self.times)]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertTrue(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))


