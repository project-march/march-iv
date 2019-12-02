import smach

from .states.gait_state import GaitState
from .states.stoppable_state import StoppableState


def create():
    sm_walk = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    # Open the container
    sm_walk.userdata.stop_pressed = False
    with sm_walk:
        # Movement states
        smach.StateMachine.add('RIGHT_OPEN', GaitState('walk', 'right_open'),
                               transitions={'succeeded': 'LEFT SWING', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT SWING', StoppableState('walk', 'right_swing'),
                               transitions={'succeeded': 'LEFT SWING',
                                            'stopped': 'LEFT CLOSE',
                                            'aborted': 'failed'})

        smach.StateMachine.add('LEFT SWING', StoppableState('walk', 'left_swing'),
                               transitions={'succeeded': 'RIGHT SWING',
                                            'stopped': 'RIGHT CLOSE', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState('walk', 'right_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT CLOSE', GaitState('walk', 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_walk
