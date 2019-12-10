import smach

from .states.gait_state import GaitState
from .states.stoppable_state import StoppableState


def create():
    sm_walk_small = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_walk_small:
        smach.StateMachine.add('RIGHT_OPEN', GaitState('walk_small', 'right_open'),
                               transitions={'succeeded': 'LEFT SWING', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT SWING', StoppableState('walk_small', 'right_swing'),
                               transitions={'succeeded': 'LEFT SWING',
                                            'stopped': 'LEFT CLOSE',
                                            'aborted': 'failed'})

        smach.StateMachine.add('LEFT SWING', StoppableState('walk_small', 'left_swing'),
                               transitions={'succeeded': 'RIGHT SWING',
                                            'stopped': 'RIGHT CLOSE', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState('walk_small', 'right_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT CLOSE', GaitState('walk_small', 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_walk_small
