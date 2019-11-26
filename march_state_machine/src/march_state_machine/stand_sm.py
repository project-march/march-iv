import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_stand = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_stand:
        smach.StateMachine.add('PREPARE STAND UP', GaitState("stand", "prepare_stand_up"),
                               transitions={'succeeded': 'STAND UP', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('STAND UP', GaitState("stand", "stand_up"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_stand
