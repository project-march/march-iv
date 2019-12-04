import smach

from .states.gait_state import GaitState


def create():
    sm_sit = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_sit:
        smach.StateMachine.add('SIT DOWN', GaitState('sit', 'sit_down'),
                               transitions={'succeeded': 'SIT HOME', 'aborted': 'failed'})
        smach.StateMachine.add('SIT HOME', GaitState('sit', 'sit_home'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_sit
