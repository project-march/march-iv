#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState
from march_state_machine.states.StoppableState import StoppableState


def create():
    sm_walk = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    # Open the container
    sm_walk.userdata.stop_pressed = False
    with sm_walk:
        # Movement states
        smach.StateMachine.add('RIGHT_OPEN', GaitState("walk", "right_open"),
                               transitions={'succeeded': 'LEFT SWING', 'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT SWING', StoppableState("walk", "right_swing"),
                               transitions={'succeeded': 'LEFT SWING',
                                            'stopped': 'LEFT CLOSE',
                                            'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT SWING', StoppableState("walk", "left_swing"),
                               transitions={'succeeded': 'RIGHT SWING',
                                            'stopped': 'RIGHT CLOSE', 'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState("walk", "right_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT CLOSE', GaitState("walk", "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})

    return sm_walk
