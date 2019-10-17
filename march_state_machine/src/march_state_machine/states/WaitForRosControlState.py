import smach
import rospy
from controller_manager_msgs.srv import *


class WaitForRosControlState(smach.State):
    """State which waits for the parameter server to contain a certain value."""

    def __init__(self, timeout=5):
        self.timeout = rospy.Duration.from_sec(timeout)
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    """Sleep so the statemachine stays in this state longer."""

    def execute(self, userdata):
        found = False
        end = rospy.get_rostime() + self.timeout
        rospy.loginfo("trying to find controller manager")
        rospy.wait_for_service('march/controller_manager/list_controllers', 5)
        list_controllers = rospy.ServiceProxy(
            '/march/controller_manager/list_controllers', ListControllers)
        rospy.loginfo("controller manager found")

        req = ListControllersRequest()
        while not found:
            if rospy.get_rostime() > end:
                rospy.logwarn("unable to find a JointTrajectoryController")
                return 'failed'
            try:
                controller_list = list_controllers.call(req)
                for c in controller_list.controller:
                    if c.type == "effort_controllers/JointTrajectoryController" or \
                            c.type == "position_controllers/JointTrajectoryController":
                        rospy.loginfo("found gazebo ros control")
                        found = True
                        return 'succeeded'
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s" % e)
                return 'failed'
