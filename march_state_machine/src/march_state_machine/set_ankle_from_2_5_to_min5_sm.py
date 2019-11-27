import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_set_ankle_from_2_5_to_min5 = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_set_ankle_from_2_5_to_min5:
        smach.StateMachine.add('SET ANKLE FROM 2 5 TO MIN5',
                               GaitState("set_ankle_from_2_5_to_min5", "set_ankle_from_2_5_to_min5"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_set_ankle_from_2_5_to_min5
