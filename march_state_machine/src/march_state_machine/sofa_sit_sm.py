import smach

from .states.gait_state import GaitState


def create():
    sm_sofa_sit = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_sofa_sit:
        smach.StateMachine.add('SIT DOWN', GaitState('sofa_sit', 'sit_down'),
                               transitions={'succeeded': 'SIT HOME', 'aborted': 'failed'})
        smach.StateMachine.add('SIT HOME', GaitState('sofa_sit', 'sit_home'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_sofa_sit
