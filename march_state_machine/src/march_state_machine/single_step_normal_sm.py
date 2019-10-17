#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState


def create():
    sm_single_step_normal = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_single_step_normal:
        smach.StateMachine.add('RIGHT OPEN', GaitState("single_step_normal", "right_open"),
                               transitions={'succeeded': 'LEFT CLOSE', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState("single_step_normal", "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_single_step_normal
