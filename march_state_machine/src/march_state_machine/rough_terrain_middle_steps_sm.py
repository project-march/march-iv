import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_rough_terrain_middle_steps = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_rough_terrain_middle_steps:

        smach.StateMachine.add('RIGHT_OPEN', GaitState("rough_terrain_middle_steps", "right_open"),
                               transitions={'succeeded': 'LEFT SWING', 'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT SWING', GaitState("rough_terrain_middle_steps", "right_swing"),
                               transitions={'succeeded': 'LEFT CLOSE', 'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT SWING', GaitState("rough_terrain_middle_steps", "left_swing"),
                               transitions={'succeeded': 'RIGHT SWING', 'preempted': 'preempted', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT CLOSE', GaitState("rough_terrain_middle_steps", "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})

    return sm_rough_terrain_middle_steps
