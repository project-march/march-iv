#!/usr/bin/env python
import smach

import ready_sm.ready_sm as ready_sm
import moving_sm.moving_sm as moving_sm


def create():
    sm_healthy = smach.StateMachine(outcomes=['succeeded', 'error', 'failed'])
    # Open the container
    with sm_healthy:
        # Add states to the container
        smach.StateMachine.add('READY', ready_sm.create(),
                              transitions={'succeeded': 'READY',
                                           'failed': 'failed'})
        smach.StateMachine.add('MOVING', moving_sm.create(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})



    return sm_healthy
