import smach
import rospy


class WaitForParameterState(smach.State):
    """State which waits for the parameter server to contain a certain value."""

    def __init__(self, parameter, timeout=60):
        self.parameter = parameter
        self.timeout = rospy.Duration.from_sec(timeout)
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    """Sleep so the statemachine stays in this state longer."""
    def execute(self, userdata):
        end = rospy.get_rostime() + self.timeout

        while not rospy.has_param(self.parameter) and not rospy.core.is_shutdown():
            if rospy.get_rostime() > end:
                return 'failed'

        return 'succeeded'
