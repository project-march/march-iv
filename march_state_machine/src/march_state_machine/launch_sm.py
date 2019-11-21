#!/usr/bin/env python
import rospy
import smach

from march_state_machine.states.wait_for_gait_server_state import WaitForGaitServerState
from march_state_machine.states.wait_for_ros_control_state import WaitForRosControlState


##
# @brief Create the launch state machine.
# @details
# @return The launch state machine object.
def create():
    sm_launch = smach.Sequence(
        outcomes=['succeeded', 'failed'],
        connector_outcome='succeeded')

    # Open the container
    with sm_launch:
        # Add states to the container
        smach.Sequence.add('WAIT FOR GAIT SERVER', WaitForGaitServerState())
        if rospy.get_param('~unpause', False):
            smach.Sequence.add('WAIT FOR ROS_CONTROL', WaitForRosControlState())

    return sm_launch
