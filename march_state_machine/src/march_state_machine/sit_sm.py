#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState


def create():
    sm_sit = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_sit:
        smach.StateMachine.add('SIT DOWN', GaitState("sit", "sit_down"),
                               transitions={'succeeded': 'SIT HOME', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('SIT HOME', GaitState("sit", "sit_home"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_sit
