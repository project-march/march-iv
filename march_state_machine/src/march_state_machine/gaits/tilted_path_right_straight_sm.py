import smach

from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path_right_straight = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    sm_tilted_path_right_straight.register_io_keys(['sounds'])
    with sm_tilted_path_right_straight:
        smach.StateMachine.add('GAIT TP RIGHT STRAIGHT START', StepStateMachine('tilted_path_right_straight_start',
                               subgaits=['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING TP RIGHT STRAIGHT', 'failed': 'failed'})

        smach.StateMachine.add('GAIT TP RIGHT SINGLE STEP', StepStateMachine('tilted_path_right_single_step',
                               subgaits=['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING TP RIGHT STRAIGHT', 'failed': 'failed'})

        smach.StateMachine.add('STANDING TP RIGHT STRAIGHT', IdleState(outcomes=['gait_tilted_path_right_single_step',
                                                                                 'gait_tilted_path_right_straight_end',
                                                                                 'preempted']),
                               transitions={'gait_tilted_path_right_single_step': 'GAIT TP RIGHT SINGLE STEP',
                                            'gait_tilted_path_right_straight_end': 'GAIT TP RIGHT STRAIGHT END'})

        smach.StateMachine.add('GAIT TP RIGHT STRAIGHT END', StepStateMachine('tilted_path_right_straight_end',
                                                                              subgaits=['right_open', 'left_close']))

    return sm_tilted_path_right_straight
