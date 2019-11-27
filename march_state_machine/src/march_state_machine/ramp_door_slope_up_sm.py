import smach

from march_state_machine.states.gait_state import GaitState
from march_state_machine.states.stoppable_state import StoppableState


def create():
    sm_ramp_door_slope_up = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    # Open the container
    sm_ramp_door_slope_up.userdata.stop_pressed = False
    with sm_ramp_door_slope_up:
        # Movement states
        smach.StateMachine.add('RIGHT OPEN', GaitState("ramp_door_slope_up", "right_open"),
                               transitions={'succeeded': 'LEFT OPEN', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT OPEN', GaitState("ramp_door_slope_up", "left_open"),
                               transitions={'succeeded': 'RIGHT SWING', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT SWING', StoppableState("ramp_door_slope_up", "right_swing"),
                               transitions={'succeeded': 'LEFT SWING',
                                            'stopped': 'LEFT CLOSE',
                                            'aborted': 'failed'})

        smach.StateMachine.add('LEFT SWING', StoppableState("ramp_door_slope_up", "left_swing"),
                               transitions={'succeeded': 'RIGHT SWING',
                                            'stopped': 'RIGHT CLOSE', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState("ramp_door_slope_up", "right_close"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT CLOSE', GaitState("ramp_door_slope_up", "left_close"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_ramp_door_slope_up
