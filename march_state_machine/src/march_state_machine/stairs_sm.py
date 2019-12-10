import smach

from .states.gait_state import GaitState
from .states.stoppable_state import StoppableState


def create(gait_name):
    sm_stairs_up = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])

    with sm_stairs_up:
        smach.StateMachine.add('RIGHT OPEN', GaitState(gait_name, 'right_open'),
                               transitions={'succeeded': 'LEFT SWING', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT SWING', StoppableState(gait_name, 'left_swing'),
                               transitions={'succeeded': 'RIGHT SWING',
                                            'stopped': 'RIGHT CLOSE',
                                            'aborted': 'failed'})
        smach.StateMachine.add('RIGHT SWING', StoppableState(gait_name, 'right_swing'),
                               transitions={'succeeded': 'LEFT SWING',
                                            'stopped': 'LEFT CLOSE',
                                            'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState(gait_name, 'right_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState(gait_name, 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_stairs_up
