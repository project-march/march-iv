import smach

from .states.gait_state import GaitState


def create():
    sm_single_step_small = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_single_step_small:
        smach.StateMachine.add('RIGHT OPEN', GaitState('single_step_small', 'right_open'),
                               transitions={'succeeded': 'LEFT CLOSE', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState('single_step_small', 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_single_step_small
