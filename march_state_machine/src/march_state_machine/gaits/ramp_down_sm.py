import smach

from march_state_machine.state_machines.slope_state_machine import SlopeStateMachine
from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_ramp_down = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_ramp_down:
        smach.StateMachine.add('GAIT RD SLOPE DOWN', SlopeStateMachine('ramp_door_slope_down'),
                               transitions={'succeeded': 'STANDING SLOPE DOWN', 'failed': 'failed'})

        smach.StateMachine.add('STANDING SLOPE DOWN', IdleState(outcomes=['gait_ramp_door_last_step', 'preempted']),
                               transitions={'gait_ramp_door_last_step': 'GAIT RD LAST STEP'})

        smach.StateMachine.add('GAIT RD LAST STEP', StepStateMachine('ramp_door_last_step'))

    return sm_ramp_down
