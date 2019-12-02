import smach

from .states.wait_for_gait_server_state import WaitForGaitServerState


def create():
    """Creates the launch state machine.

    :return The launch sequence machine object
    """
    sm_launch = smach.Sequence(
        outcomes=['succeeded', 'preempted', 'failed'],
        connector_outcome='succeeded')

    with sm_launch:
        smach.Sequence.add('WAIT FOR GAIT SERVER', WaitForGaitServerState())

    return sm_launch
