import smach

from .states.gait_state import GaitState
from .states.stoppable_state import StoppableState


def create():
    sm_ramp_door_slope_down = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    # Open the container
    sm_ramp_door_slope_down.userdata.stop_pressed = False
    with sm_ramp_door_slope_down:
        # Movement states
        smach.StateMachine.add('RIGHT OPEN', GaitState('ramp_door_slope_down', 'right_open'),
                               transitions={'succeeded': 'LEFT OPEN', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT OPEN', GaitState('ramp_door_slope_down', 'left_open'),
                               transitions={'succeeded': 'RIGHT SWING', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT SWING', StoppableState('ramp_door_slope_down', 'right_swing'),
                               transitions={'succeeded': 'LEFT SWING', 'stopped': 'LEFT CLOSE', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT SWING', StoppableState('ramp_door_slope_down', 'left_swing'),
                               transitions={'succeeded': 'RIGHT SWING', 'stopped': 'RIGHT CLOSE', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState('ramp_door_slope_down', 'right_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT CLOSE', GaitState('ramp_door_slope_down', 'left_close'),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_ramp_door_slope_down
