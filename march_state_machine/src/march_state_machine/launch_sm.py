import smach

from march_state_machine.states.wait_for_ros_control_state import WaitForRosControlState


def create():
    """Creates the launch state machine.

    :return The launch sequence machine object
    """
    sm_launch = smach.Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded')

    with sm_launch:
        smach.Sequence.add('WAIT FOR ROS_CONTROL', WaitForRosControlState())

    return sm_launch
