import unittest

import numpy as np

from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.setpoint import Setpoint


class JointTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.joint_name = 'test_joint'
        self.limits = Limits(-1, 1, 2)
        self.duration = 2.0
        self.times = [0, self.duration / 2.0, self.duration]
        self.setpoints = [Setpoint(t, 2 * t, t / 2.0) for t in self.times]
        self.joint_trajectory = JointTrajectory(self.joint_name, self.limits, self.setpoints, self.duration)

    # get_setpoints_unzipped tests
    def test_get_setpoints_unzipped_time(self):
        output = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(output, (self.times, [2 * t for t in self.times], [t / 2.0 for t in self.times]))

    # validate_joint_transition() tests
    def test_valid_joint_transition(self):
        next_setpoints = [Setpoint(t, 2 * (self.duration - t), (self.duration - t) / 2.0) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertTrue(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_position(self):
        next_setpoints = [Setpoint(t, (self.duration - t), (self.duration - t) / 2.0) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    def test_invalid_joint_transition_velocity(self):
        next_setpoints = [Setpoint(t, 2 * (self.duration - t), (self.duration - t)) for t in self.times]
        next_joint_trajectory = JointTrajectory(self.joint_name, self.limits, next_setpoints, self.duration)
        self.assertFalse(self.joint_trajectory.validate_joint_transition(next_joint_trajectory))

    # _validate_boundary_points tests
    def test_valid_boundary_points_nonzero_start_end_zero_speed(self):
        # First setpoint at t = 0.5 and last setpoint at t = 1.5 =/= duration have zero speed.
        setpoints = [Setpoint(0.5 * t + 0.5, (self.duration - t), t * 2 - t**2) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertTrue(joint_trajectory._validate_boundary_points())

    def test_invalid_boundary_points_nonzero_start_nonzero_speed(self):
        # First setpoint at t = 1 has nonzero speed.
        setpoints = [Setpoint(0.5 * t + 1, (self.duration - t), (self.duration - t) * 2) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertFalse(joint_trajectory._validate_boundary_points())

    def test_invalid_boundary_points_nonzero_end_nonzero_speed(self):
        # Last setpoint at t = 1 =/= duration has nonzero speed.
        setpoints = [Setpoint(0.5 * t, (self.duration - t), (self.duration - t) / 2.0) for t in self.times]
        joint_trajectory = JointTrajectory(self.joint_name, self.limits, setpoints, self.duration)
        self.assertFalse(joint_trajectory._validate_boundary_points())

    # interpolate_setpoints tests
    def test_interpolation_time_points(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        time_points = np.linspace(0, self.duration, len(interpolated_list[0]))
        self.assertTrue((interpolated_list[0] == time_points).all())

    def test_interpolation_start_point_position(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        self.assertEqual(interpolated_list[1][0], self.setpoints[0].position)

    def test_interpolation_start_point_velocity(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        inter_velocity = (interpolated_list[1][1] - interpolated_list[1][0]) / \
                         (interpolated_list[0][1] - interpolated_list[0][0])
        self.assertTrue(abs(inter_velocity - self.setpoints[0].velocity) <= 0.1,
                        msg='Interpolated start velocity {inter_v} was too far from actual start velocity '
                            '{actual_v}'.format(inter_v=inter_velocity, actual_v=self.setpoints[0].velocity))

    def test_interpolation_end_point_position(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        self.assertEqual(interpolated_list[1][-1], self.setpoints[-1].position)

    def test_interpolation_end_point_velocity(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        inter_velocity = (interpolated_list[1][-2] - interpolated_list[1][-1]) / \
                         (interpolated_list[0][-2] - interpolated_list[0][-1])
        self.assertTrue(abs(inter_velocity - self.setpoints[-1].velocity) <= 0.1,
                        msg='Interpolated end velocity {inter_v} was too far from actual end velocity '
                            '{actual_v}'.format(inter_v=inter_velocity, actual_v=self.setpoints[-1].velocity))

    def test_interpolation_mid_point_position(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        mid_index = len(interpolated_list[1]) / 2
        inter_position = (interpolated_list[1][mid_index] + interpolated_list[1][mid_index - 1]) / 2
        self.assertTrue(abs(inter_position - self.setpoints[1].position) <= 0.001,
                        msg='Interpolated midpoint position {inter_x} was too far from actual midpoint position '
                            '{actual_x}'.format(inter_x=inter_position, actual_x=self.setpoints[1].position))

    def test_interpolation_mid_point_velocity(self):
        interpolated_list = self.joint_trajectory.interpolate_setpoints()
        mid_index = len(interpolated_list[1]) / 2
        inter_velocity = (interpolated_list[1][mid_index] - interpolated_list[1][mid_index - 1]) / \
                         (interpolated_list[0][mid_index] - interpolated_list[0][mid_index - 1])
        self.assertTrue(abs(inter_velocity - self.setpoints[1].velocity) <= 0.1,
                        msg='Interpolated midpoint velocity {inter_v} was too far from actual midpoint velocity '
                            '{actual_v}'.format(inter_v=inter_velocity, actual_v=self.setpoints[1].velocity))

    # get_interpolated_setpoint tests
    def test_get_interpolated_setpoints_invalid_time(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(self.duration + 1)
        self.assertEqual(setpoint, Setpoint(0, 0, 0))

    def test_get_interpolated_setpoints_on_setpoint(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(self.times[1])
        self.assertEqual(setpoint, self.setpoints[1])

    def test_get_interpolated_setpoints(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(self.duration / 3.0)
        self.assertIsInstance(setpoint, Setpoint)
