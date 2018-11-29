#!/usr/bin/env python
import rospy
import smach
import smach_ros
import launch_sm.launch_sm as launch_sm
import healthy_sm.healthy_sm as healthy_sm
import error_sm.error_sm as error_sm
import shutdown_sm.shutdown_sm as shutdown_sm

import time

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        time.sleep(10)
        return 'succeeded'


def main():
    rospy.init_node("state_machine")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('START', Start(), transitions={'succeeded': 'LAUNCH'})

        smach.StateMachine.add('LAUNCH', launch_sm.create(), transitions={'succeeded': 'HEALTHY', 'failed': 'SHUTDOWN'})
        smach.StateMachine.add('HEALTHY', healthy_sm.create(), transitions={'succeeded': 'HEALTHY', 'error': 'ERROR', 'failed': 'SHUTDOWN'})
        smach.StateMachine.add('ERROR', error_sm.create(), transitions={'succeeded': 'HEALTHY', 'failed': 'SHUTDOWN'})
        smach.StateMachine.add('SHUTDOWN', shutdown_sm.create(), transitions={'succeeded': 'DONE', 'failed':'DONE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
