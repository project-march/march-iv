import smach

from .states.gait_state import GaitState


def create():
    sm_tilted_path_middle_step = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_tilted_path_middle_step:
        smach.StateMachine.add('LEFT OPEN', GaitState('tilted_path_middle_step', 'left_open'),
                               transitions={'succeeded': 'RIGHT CLOSE', 'aborted': 'failed'})
        smach.StateMachine.add('RIGHT CLOSE', GaitState('tilted_path_middle_step', 'right_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_tilted_path_middle_step
