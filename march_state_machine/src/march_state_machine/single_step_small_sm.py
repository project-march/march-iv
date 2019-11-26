import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_single_step_small = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_single_step_small:
        smach.StateMachine.add('RIGHT OPEN', GaitState("single_step_small", "right_open"),
                               transitions={'succeeded': 'LEFT CLOSE', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('LEFT CLOSE', GaitState("single_step_small", "left_close"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_single_step_small
