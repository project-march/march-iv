import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_set_ankle_from_min10_to_min5 = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_set_ankle_from_min10_to_min5:
        smach.StateMachine.add('SET ANKLE FROM MIN10 TO MIN5',
                               GaitState("set_ankle_from_min10_to_min5", "set_ankle_from_min10_to_min5"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})
    return sm_set_ankle_from_min10_to_min5
