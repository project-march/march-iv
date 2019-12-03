import smach

from .states.gait_state import GaitState


def create():
    sm_ramp_door_last_step = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_ramp_door_last_step:
        smach.StateMachine.add('RIGHT OPEN', GaitState('ramp_door_last_step', 'right_open'),
                               transitions={'succeeded': 'LEFT CLOSE', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState('ramp_door_last_step', 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_ramp_door_last_step
