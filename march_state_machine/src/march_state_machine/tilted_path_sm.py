import smach

from march_state_machine import set_ankle_from_2_5_to_min5_sm
from march_state_machine import tilted_path_first_starting_step_sm
from march_state_machine import tilted_path_second_starting_step_sm
from march_state_machine import set_ankle_from_min5_to_min10_sm
from march_state_machine import tilted_path_middle_step_sm
from march_state_machine import set_ankle_from_min10_to_min5_sm
from march_state_machine import tilted_path_first_ending_step_sm
from march_state_machine import tilted_path_second_ending_step_sm
from march_state_machine import set_ankle_from_min5_to_2_5_sm

from march_state_machine.states.idle_state import IdleState


def create():
    sm_tilted_path = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_tilted_path:

        smach.StateMachine.add('GAIT SET ANKLE FROM 2 5 TO MIN5', set_ankle_from_2_5_to_min5_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH START FIRST START STEP',
                                            'preempted': 'failed', 'failed': 'failed'})

        smach.StateMachine.add('GAIT SET ANKLE FROM MIN5 TO MIN10', set_ankle_from_min5_to_min10_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH MIDDLE STEP', 'preempted': 'failed',
                                            'failed': 'failed'})

        smach.StateMachine.add('GAIT SET ANKLE FROM MIN10 TO MIN5', set_ankle_from_min10_to_min5_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH START FIRST END STEP',
                                            'preempted': 'failed', 'failed': 'failed'})

        smach.StateMachine.add('GAIT SET ANKLE FROM MIN5 TO 2 5', set_ankle_from_min5_to_2_5_sm.create(),
                               transitions={'succeeded': 'succeeded', 'preempted': 'failed', 'failed': 'failed'})

        smach.StateMachine.add('GAIT TILTED PATH FIRST STARTING STEP',
                               tilted_path_first_starting_step_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH START SECOND START STEP',
                                            'preempted': 'failed', 'failed': 'failed'})

        smach.StateMachine.add('GAIT TILTED PATH SECOND STARTING STEP',
                               tilted_path_second_starting_step_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH START ANKLES MIN5',
                                            'preempted': 'failed',
                                            'failed': 'failed'})

        smach.StateMachine.add('GAIT TILTED PATH MIDDLE STEP', tilted_path_middle_step_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH MIDDLE STEP', 'preempted': 'failed',
                                            'failed': 'failed'})

        smach.StateMachine.add('GAIT TILTED PATH FIRST ENDING STEP', tilted_path_first_ending_step_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH START SECOND END STEP',
                                            'preempted': 'failed', 'failed': 'failed'})

        smach.StateMachine.add('GAIT TILTED PATH SECOND ENDING STEP',
                               tilted_path_second_ending_step_sm.create(),
                               transitions={'succeeded': 'STANDING TILTED PATH END ANKLES MIN5', 'preempted': 'failed',
                                            'failed': 'failed'})

        smach.StateMachine.add('STANDING TILTED PATH START FIRST START STEP',
                               IdleState(outcomes=['gait_tilted_path_first_starting_step', 'preempted']),
                               transitions={
                                   'gait_tilted_path_first_starting_step': 'GAIT TILTED PATH FIRST STARTING STEP',
                                   'preempted': 'failed'})
        smach.StateMachine.add('STANDING TILTED PATH START SECOND START STEP',
                               IdleState(outcomes=['gait_tilted_path_second_starting_step', 'preempted']),
                               transitions={
                                   'gait_tilted_path_second_starting_step': 'GAIT TILTED PATH SECOND STARTING STEP',
                                   'preempted': 'failed'})
        smach.StateMachine.add('STANDING TILTED PATH START ANKLES MIN5',
                               IdleState(outcomes=['gait_set_ankle_from_min5_to_min10', 'preempted']),
                               transitions={'gait_set_ankle_from_min5_to_min10': 'GAIT SET ANKLE FROM MIN5 TO MIN10',
                                            'preempted': 'failed'})
        smach.StateMachine.add('STANDING TILTED PATH MIDDLE STEP', IdleState(
            outcomes=['gait_tilted_path_middle_step', 'gait_set_ankle_from_min10_to_min5', 'preempted']),
                               transitions={'gait_tilted_path_middle_step': 'GAIT TILTED PATH MIDDLE STEP',
                                            'gait_set_ankle_from_min10_to_min5': 'GAIT SET ANKLE FROM MIN10 TO MIN5',
                                            'preempted': 'failed'})
        smach.StateMachine.add('STANDING TILTED PATH START FIRST END STEP',
                               IdleState(outcomes=['gait_tilted_path_first_ending_step', 'preempted']),
                               transitions={'gait_tilted_path_first_ending_step': 'GAIT TILTED PATH FIRST ENDING STEP',
                                            'preempted': 'failed'})
        smach.StateMachine.add('STANDING TILTED PATH START SECOND END STEP',
                               IdleState(outcomes=['gait_tilted_path_second_ending_step', 'preempted']),
                               transitions={
                                   'gait_tilted_path_second_ending_step': 'GAIT TILTED PATH SECOND ENDING STEP',
                                   'preempted': 'failed'})
        smach.StateMachine.add('STANDING TILTED PATH END ANKLES MIN5',
                               IdleState(outcomes=['gait_set_ankle_from_min5_to_2_5', 'preempted']),
                               transitions={'gait_set_ankle_from_min5_to_2_5': 'GAIT SET ANKLE FROM MIN5 TO 2 5',
                                            'preempted': 'failed'})

    return sm_tilted_path
