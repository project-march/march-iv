import psutil
import rospy
import smach


class ShutdownState(smach.State):
    """Shutdown state that will request shutdown from ROS. Will always succeed."""

    def __init__(self):
        super(ShutdownState, self).__init__(outcomes=['succeeded'])

    def execute(self, _userdata):
        rospy.signal_shutdown('State machine shutdown')

        # Please forgive me for my sins
        # This is an issue with gazebo that will not respond to SIGINT
        # So we send a SIGTERM instead to kill it
        # See https://github.com/ros-simulation/gazebo_ros_pkgs/issues/751
        for p in psutil.process_iter():
            if p.name() == 'gzserver':
                p.kill()

        return 'succeeded'
