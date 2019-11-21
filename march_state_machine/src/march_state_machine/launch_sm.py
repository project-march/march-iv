#!/usr/bin/env python
import smach

from march_state_machine.states.wait_for_gait_server_state import WaitForGaitServerState


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

    return sm_launch
