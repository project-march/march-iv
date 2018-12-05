import rospy
import smach

import time

class Walking(smach.GaitMonitorState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Walking'
        time.sleep(5)
        return 'succeeded'
