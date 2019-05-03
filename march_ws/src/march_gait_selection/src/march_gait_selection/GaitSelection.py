#!/usr/bin/env python
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryResult
from march_shared_resources.msg import GaitAction
from trajectory_msgs.msg import JointTrajectoryPoint


class PoseToTrajectoryAction(object):
    _target_gait_action_server = None
    _trajectory_execution_client = None

    def __init__(self):
        self._target_gait_action_server = actionlib.SimpleActionServer("march/gait/perform", GaitNameAction,
                                                                       execute_cb=self.target_gait_callback,
                                                                       auto_start=False)
        self._target_gait_action_server.start()
        self._trajectory_execution_client = actionlib.SimpleActionClient("march/gait/schedule",
                                                                         GaitAction)
        self._trajectory_execution_client.wait_for_server()

    def target_gait_callback(self, goal):
        rospy.loginfo(" %s pose requested", goal.subgait_name)
        trajectory_result = self.execute_trajectory(goal.subgait_name)
        if trajectory_result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo("set_succeeded")
            self._target_gait_action_server.set_succeeded()
        else:
            rospy.loginfo("set_aborted")
            self._target_gait_action_server.set_aborted()

    def execute_trajectory(self, gait_name):
        trajectory_message = FollowJointTrajectoryActionGoal()
        trajectory_message.goal.trajectory.joint_names = ["left_hip", "left_knee", "left_ankle", "right_hip",
                                                          "right_knee", "right_ankle"]
        # For now we handle everything with 2 gaits sit and stand.
        # @TODO(Isha) implement proper gait selection
        if gait_name == "sit":
            point = JointTrajectoryPoint()
            point.positions = [1.3, 1.3, 0.349065850399, 1.3, 1.3, 0.349065850399]
            point.velocities = [0, 0, 0, 0, 0, 0]
            point.time_from_start = rospy.Duration.from_sec(3)
        else:
            point = JointTrajectoryPoint()
            point.positions = [0, 0, 0, 0, 0, 0]
            point.velocities = [0, 0, 0, 0, 0, 0]
            point.time_from_start = rospy.Duration.from_sec(3)
        trajectory_message.goal.trajectory.points.append(point)
        self._trajectory_execution_client.send_goal(trajectory_message.goal)
        rospy.logdebug("wait_for_result:")
        self._trajectory_execution_client.wait_for_result()
        return self._trajectory_execution_client.get_result()


if __name__ == '__main__':
    rospy.init_node("gait_selection")
    server = PoseToTrajectoryAction()
    rate = rospy.Rate(10)
    rospy.spin()
