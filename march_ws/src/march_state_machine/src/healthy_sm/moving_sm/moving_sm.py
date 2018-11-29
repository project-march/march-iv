#!/usr/bin/env python
import smach

from Walking import Walking

import walking_sm.walking_sm as walking_sm


def create():
    sm_moving = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_moving:
        # Add states to the container
        smach.StateMachine.add('WALKING', walking_sm.create(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm_moving
