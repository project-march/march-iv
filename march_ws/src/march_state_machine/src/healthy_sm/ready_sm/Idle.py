import rospy
import smach

import time

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Idle')
        time.sleep(5)
        return 'succeeded'
