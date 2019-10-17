#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState


def create():
    sm_side_step_left_small = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_side_step_left_small:
        smach.StateMachine.add('LEFT OPEN', GaitState("side_step_left_small", "left_open"),
                               transitions={'succeeded': 'RIGHT CLOSE', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('RIGHT CLOSE', GaitState("side_step_left_small", "right_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_side_step_left_small
