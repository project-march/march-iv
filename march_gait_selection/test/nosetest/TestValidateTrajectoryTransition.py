#!/usr/bin/env python

import unittest
import rospy

from march_gait_selection.GaitSelection import GaitSelection
from march_shared_resources.msg import Subgait

from trajectory_msgs.msg import JointTrajectoryPoint


class TestValidateTrajectoryTransition(unittest.TestCase):

    def test_transition_correct_2_points(self):
        from_subgait = Subgait()
        from_subgait.trajectory.joint_names = ["joint1", "joint2"]
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(0)))
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(1)))

        to_subgait = Subgait()
        to_subgait.trajectory.joint_names = ["joint1", "joint2"]
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(0)))
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(2)))

        self.assertTrue(GaitSelection.validate_trajectory_transition(from_subgait.trajectory, to_subgait.trajectory) == "")
        self.assertTrue(GaitSelection.validate_trajectory_transition(to_subgait.trajectory, from_subgait.trajectory) == "")

    def test_transition_correct_3_points(self):
        from_subgait = Subgait()
        from_subgait.trajectory.joint_names = ["joint1", "joint2"]
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(0)))
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0.5, 0], velocities=[0, 1], time_from_start=rospy.Duration.from_sec(0.5)))
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(1)))

        to_subgait = Subgait()
        to_subgait.trajectory.joint_names = ["joint1", "joint2"]
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(0)))
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(2)))
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0.5, 0], velocities=[0, 1], time_from_start=rospy.Duration.from_sec(3)))

        self.assertTrue(GaitSelection.validate_trajectory_transition(from_subgait.trajectory, to_subgait.trajectory) == "")
        self.assertFalse(GaitSelection.validate_trajectory_transition(to_subgait.trajectory, to_subgait.trajectory) == "")

    def test_transition_position_mismatch(self):
        from_subgait = Subgait()
        from_subgait.trajectory.joint_names = ["joint1", "joint2"]
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(0)))
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(1)))

        to_subgait = Subgait()
        to_subgait.trajectory.joint_names = ["joint1", "joint2"]
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.3], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(0)))
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(2)))

        self.assertFalse(GaitSelection.validate_trajectory_transition(from_subgait.trajectory, to_subgait.trajectory) == "")

    def test_transition_joint_mismatch(self):
        from_subgait = Subgait()
        from_subgait.trajectory.joint_names = ["joint_wrong", "joint2"]
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(0)))
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(1)))

        to_subgait = Subgait()
        to_subgait.trajectory.joint_names = ["joint1", "joint2"]
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(0)))
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(2)))

        self.assertFalse(GaitSelection.validate_trajectory_transition(from_subgait.trajectory, to_subgait.trajectory)
                         == "")

    def test_transition_velocity_mismatch(self):
        from_subgait = Subgait()
        from_subgait.trajectory.joint_names = ["joint1", "joint2"]
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(0)))
        from_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[-0.5, -0.2], time_from_start=rospy.Duration.from_sec(1)))

        to_subgait = Subgait()
        to_subgait.trajectory.joint_names = ["joint1", "joint2"]
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[1, 0.5], velocities=[0, -0.2], time_from_start=rospy.Duration.from_sec(0)))
        to_subgait.trajectory.points.append(JointTrajectoryPoint(
            positions=[0, 0], velocities=[0, 0], time_from_start=rospy.Duration.from_sec(2)))

        self.assertFalse(GaitSelection.validate_trajectory_transition(from_subgait.trajectory, to_subgait.trajectory)
                         == "")
