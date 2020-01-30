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

    # get_setpoints_unzipped tests
    def test_get_setpoints_unzipped_time(self):
        time, position, velocity = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(time, [t for t in self.times])

    def test_get_setpoints_unzipped_position(self):
        time, position, velocity = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(position, [2 * t for t in self.times])

    def test_get_setpoints_unzipped_velocity(self):
        time, position, velocity = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(velocity, [t / 2 for t in self.times])

     # validate_joint_transition() tests
    def test_valid_joint_transition(self):
        next_setpoints = [Setpoint(t, 2 * (max(self.times) - t), (max(self.times) - t) / 2) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertTrue(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_position(self):
        next_setpoints = [Setpoint(t, (max(self.times) - t), (max(self.times) - t) / 2) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_velocity(self):
        next_setpoints = [Setpoint(t, 2 * (max(self.times) - t), (max(self.times) - t)) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    # _validate_boundary_points tests
    def test_valid_joint_transition_nonzero_start_zero_speed(self):
        # First setpoint at t = 0.5 and last setpoint at t = 1.5 =/= duration have zero speed.
        setpoints = [Setpoint(0.5*t + 0.5, (max(self.times) - t), t * 2 - 2 * t) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertTrue(joint_trajectory._validate_boundary_points())

    def test_invalid_joint_transition_nonzero_start_nonzero_speed(self):
        # First setpoint at t = 1 has nonzero speed.
        setpoints = [Setpoint(0.5*t + 1, (max(self.times) - t), t / 2) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertFalse(joint_trajectory._validate_boundary_points())

    def test_invalid_joint_transition_nonzero_start_nonzero_speed(self):
        # Last setpoint at t = 1 =/= duration has nonzero speed.
        setpoints = [Setpoint(0.5*t, (max(self.times) - t), (max(self.times) - t) / 2) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertFalse(joint_trajectory._validate_boundary_points())

    # get_interpolated_setpoint tests

