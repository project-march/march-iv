import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_set_ankle_from_min5_to_min10 = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_set_ankle_from_min5_to_min10:
        smach.StateMachine.add('SET ANKLE FROM MIN5 TO MIN10',
                               GaitState("set_ankle_from_min5_to_min10", "set_ankle_from_min5_to_min10"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_set_ankle_from_min5_to_min10
