import smach

from march_state_machine.state_machines.step_state_machine import StepStateMachine
from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path_flexed_knee_step = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_tilted_path_flexed_knee_step:
        smach.StateMachine.add('GAIT TP FLEXED KNEE STEP', StepStateMachine('tilted_path_flexed_knee_step',
                               subgaits=['right_open', 'left_close']),
                               transitions={'succeeded': 'STANDING TP STRAIGHT', 'failed': 'failed'})

        smach.StateMachine.add('GAIT TP SINGLE STEP', StepStateMachine('tilted_path_single_step',
                               subgaits=['right_open', 'left_close']),
                               transitions={'succeeded': 'STANDING TP STRAIGHT', 'failed': 'failed'})

        smach.StateMachine.add('STANDING TP STRAIGHT', IdleState(outcomes=['gait_tilted_path_single_step',
                                                                           'gait_tilted_path_straight_end',
                                                                           'preempted']),
                               transitions={'gait_tilted_path_single_step': 'GAIT TP SINGLE STEP',
                                            'gait_tilted_path_straight_end': 'GAIT TP STRAIGHT END'})

        smach.StateMachine.add('GAIT TP STRAIGHT END', StepStateMachine('tilted_path_straight_end',
                                                                        subgaits=['left_open', 'right_close']))

    return sm_tilted_path_flexed_knee_step
