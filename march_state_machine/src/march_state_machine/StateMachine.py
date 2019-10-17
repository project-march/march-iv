#!/usr/bin/env python
import rospy
import smach
import smach_ros
import threading

from multiprocessing.pool import ThreadPool
from march_state_machine import launch_sm, healthy_sm
from march_state_machine.SafetyState import SafetyState
from march_state_machine.states.EmptyState import EmptyState


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'


def main():
    rospy.init_node("state_machine")

    sm = create_sm()
    if rospy.get_param("state_machine_viewer", "true"):
        sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
        sis.start()
    print sm.check_consistency()

    pool = ThreadPool(processes=1)
    async_result = pool.apply_async(sm.execute)

    rospy.spin()
    if rospy.get_param("state_machine_viewer", "true"):
        sis.stop()


def create_sm():
    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('START', Start(),
                               transitions={'succeeded': 'LAUNCH'})
        smach.StateMachine.add('LAUNCH', launch_sm.create(),
                               transitions={'succeeded': 'HEALTHY', 'failed': 'SHUTDOWN'})

        safety_concurrence = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                               default_outcome='failed', child_termination_cb=lambda outcome_map: True)

        with safety_concurrence:
            smach.Concurrence.add("SAFETY", SafetyState(outcomes=['error']))
            smach.Concurrence.add("STATE MACHINE", healthy_sm.create())

        smach.StateMachine.add('HEALTHY', safety_concurrence,
                               transitions={'succeeded': 'SHUTDOWN', 'failed': 'ERROR'})
        smach.StateMachine.add('ERROR', EmptyState(),
                               transitions={'succeeded': 'HEALTHY', 'failed': 'SHUTDOWN'})
        smach.StateMachine.add('SHUTDOWN', EmptyState(),
                               transitions={'succeeded': 'DONE', 'failed': 'DONE'})

    return sm
