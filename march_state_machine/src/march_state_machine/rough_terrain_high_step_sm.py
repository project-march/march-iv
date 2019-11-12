#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState


def create():
    sm_rough_terrain_high_step = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_rough_terrain_high_step:
        smach.StateMachine.add('RIGHT OPEN', GaitState("rough_terrain_high_step", "right_open"),
                               transitions={'succeeded': 'LEFT CLOSE', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState("rough_terrain_high_step", "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_rough_terrain_high_step