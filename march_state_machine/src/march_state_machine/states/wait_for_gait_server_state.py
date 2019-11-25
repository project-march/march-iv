import actionlib
import rospy
import smach

from march_shared_resources.msg import GaitNameAction
from . import DEFAULT_TIMEOUT_SECS


class WaitForGaitServerState(smach.State):
    """State which waits for the gait server to be available."""

    def __init__(self, timeout=rospy.Duration(secs=DEFAULT_TIMEOUT_SECS)):
        self._timeout = timeout
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failed'])

    def execute(self, _userdata):
        rospy.logdebug('Waiting for /march/gait/perform action server')
        gait_client = actionlib.SimpleActionClient('/march/gait/perform', GaitNameAction)

        if gait_client.wait_for_server(self._timeout):
            return 'succeeded'
        elif rospy.is_shutdown():
            return 'preempted'
        else:
            rospy.logerr('Waiting for gait server timed out')
            return 'failed'
