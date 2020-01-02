import smach

from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path_sideways = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_tilted_path_sideways:
        smach.StateMachine.add('GAIT TP STRAIGHT FIRST', StepStateMachine('tilted_path_first_start',
                                                                          subgaits=['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING TP SIDEWAYS', 'failed': 'failed'})

        smach.StateMachine.add('STANDING TP SIDEWAYS', IdleState(outcomes=['gait_tilted_path_second_start',
                                                                           'preempted']),
                               transitions={'gait_tilted_path_second_start': 'GAIT TP STRAIGHT SECOND'})

        smach.StateMachine.add('GAIT TP STRAIGHT SECOND', StepStateMachine('tilted_path_second_start',
                                                                           subgaits=['left_open', 'right_close']))

    return sm_tilted_path_sideways
