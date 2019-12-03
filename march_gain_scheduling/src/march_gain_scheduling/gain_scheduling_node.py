#!/usr/bin/env python
import rospy
from dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    # The parameters Joint_list and gait_list should be obtained from the parameter server of the workspace.
    joint_list = ["left_hip_aa", "right_hip_aa", "left_ankle", "right_ankle", "left_hip_fe", "right_hip_fe",
                  "left_knee", "right_knee"]

    rospy.init_node("march_gain_scheduling_node")

    DynamicPIDReconfigurer(joint_list=joint_list)

    rospy.spin()
