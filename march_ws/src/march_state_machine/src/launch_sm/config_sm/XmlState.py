import rospy
import smach
from march_custom_msgs.srv import Trigger


class XmlState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking xml')
        return 'succeeded'

    def service_preempt(self):
        rospy.logwarn("SERVICE PREEMPT")
        return 'failed'
