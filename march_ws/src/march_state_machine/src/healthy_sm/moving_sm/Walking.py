import rospy
import smach

import time

class Walking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Walking')
        return 'succeeded'
