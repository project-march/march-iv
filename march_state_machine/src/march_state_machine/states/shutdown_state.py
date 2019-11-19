import rospy
import smach


class ShutdownState(smach.State):
    """Shutdown state that will request shutdown from ROS. Will always succeed."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, _userdata):
        rospy.signal_shutdown('State machine shutdown')
        return 'succeeded'
