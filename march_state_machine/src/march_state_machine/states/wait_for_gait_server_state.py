import actionlib
import rospy
import smach

from march_shared_resources.msg import GaitNameAction


class WaitForGaitServerState(smach.State):
    """State which waits for the gait server to be available."""

    def __init__(self, timeout=rospy.Duration.from_sec(60)):
        self._timeout = timeout
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, _userdata):
        rospy.logdebug('Waiting for /march/gait/perform action server')
        gait_client = actionlib.SimpleActionClient('/march/gait/perform', GaitNameAction)

        if gait_client.wait_for_server(self._timeout):
            return 'succeeded'
        else:
            return 'failed'
