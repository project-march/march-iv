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

        end_time = rospy.Time.now() + self._timeout
        while not gait_client.wait_for_server(rospy.Duration(0.1)):
            if rospy.Time.now() > end_time:
                rospy.logerr('Waiting for gait server timed out after {}s'.format(self._timeout.to_sec()))
                return 'failed'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

        if rospy.is_shutdown():
            return 'failed'
        return 'succeeded'
