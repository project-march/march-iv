#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState
from march_state_machine.states.StoppableState import StoppableState


def create(gait_name):
    sm_stairs_up = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])

    sm_stairs_up.userdata.stop_pressed = False
    with sm_stairs_up:
        smach.StateMachine.add('RIGHT OPEN', GaitState(gait_name, "right_open"),
                               transitions={'succeeded': 'LEFT SWING', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT SWING', StoppableState(gait_name, "left_swing"),
                               transitions={'succeeded': 'RIGHT SWING',
                                            'stopped': 'RIGHT CLOSE',
                                            'preempted': 'preempted',
                                            'aborted': 'failed'})
        smach.StateMachine.add('RIGHT SWING', StoppableState(gait_name, "right_swing"),
                               transitions={'succeeded': 'LEFT SWING',
                                            'stopped': 'LEFT CLOSE',
                                            'preempted': 'preempted',
                                            'aborted': 'failed'})

        smach.StateMachine.add('RIGHT CLOSE', GaitState(gait_name, "right_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState(gait_name, "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})

    return sm_stairs_up
