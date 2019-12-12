import smach

from . import ramp_door_last_step_sm
from . import ramp_door_slope_down_sm
from .states.idle_state import IdleState


def create():
    sm_ramp_down = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_ramp_down:

        smach.StateMachine.add('GAIT RD SLOPE DOWN', ramp_door_slope_down_sm.create(),
                               transitions={'succeeded': 'STANDING SLOPE DOWN', 'failed': 'failed'})

        smach.StateMachine.add('GAIT RD LAST STEP', ramp_door_last_step_sm.create(),
                               transitions={'succeeded': 'succeeded', 'failed': 'failed'})

        smach.StateMachine.add('STANDING SLOPE DOWN', IdleState(outcomes=['gait_ramp_door_last_step', 'preempted']),
                               transitions={'gait_ramp_door_last_step': 'GAIT RD LAST STEP'})

    return sm_ramp_down
