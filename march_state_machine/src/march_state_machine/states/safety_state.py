import smach_ros
import rospy
from march_shared_resources.msg import Error


class SafetyState(smach_ros.MonitorState):
    """State which monitors if errors have occurred.

    This state always runs concurrent to the healthy state machine.
    Listens to the topic '/march/error' and returns 'invalid' when
    an error occurs and 'valid' when no error has occurred.
    """

    def __init__(self):
        super(SafetyState, self).__init__('/march/error', Error, self.error_callback)

    @staticmethod
    def error_callback(userdata, msg):
        """
        Callback method for messages published on the '/march/error' topic.
        :param userdata: Userdata passed to the state
        :param msg: Error message published on the topic
        :return: False if the error is fatal, True otherwise
        """
        if msg.type == Error.NON_FATAL:
            rospy.logerr('SafetyState has noticed a non fatal error: ' + msg.error_message)
            return True
        rospy.logfatal('SafetyState has noticed a fatal error: ' + msg.error_message)
        return False
