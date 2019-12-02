import smach

from .states.gait_state import GaitState


def create():
    sm_rough_terrain_high_step = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_rough_terrain_high_step:
        smach.StateMachine.add('RIGHT OPEN', GaitState('rough_terrain_high_step', 'right_open'),
                               transitions={'succeeded': 'LEFT CLOSE', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState('rough_terrain_high_step', 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_rough_terrain_high_step
