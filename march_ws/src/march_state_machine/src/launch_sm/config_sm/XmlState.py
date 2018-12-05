import rospy
import smach
from march_custom_msgs.srv import Trigger

class XmlState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Checking config')
        checkConfig = rospy.ServiceProxy('march/config_validation', Trigger)
        result = checkConfig()
        self.request_preempt()
        rospy.loginfo(result)
        if result.success:
            return 'succeeded'
        else:
            return 'failed'

    def service_preempt(self):
        rospy.logwarn("SERVICE PREEMPT")
        return 'failed'
