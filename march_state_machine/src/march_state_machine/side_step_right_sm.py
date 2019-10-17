#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState


def create():
    sm_side_step_right = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_side_step_right:
        smach.StateMachine.add('RIGHT OPEN', GaitState("side_step_right", "right_open"),
                               transitions={'succeeded': 'LEFT CLOSE', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState("side_step_right", "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_side_step_right
