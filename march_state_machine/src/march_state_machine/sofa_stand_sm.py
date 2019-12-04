import smach

from .states.gait_state import GaitState


def create():
    sm_sofa_stand = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_sofa_stand:
        smach.StateMachine.add('PREPARE STAND UP', GaitState('sofa_stand', 'prepare_stand_up'),
                               transitions={'succeeded': 'STAND UP', 'aborted': 'failed'})
        smach.StateMachine.add('STAND UP', GaitState('sofa_stand', 'stand_up'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_sofa_stand
