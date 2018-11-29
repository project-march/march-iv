import rospy
import smach
from march_custom_msgs.srv import Trigger


class UrdfState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking config')
        return 'succeeded'
