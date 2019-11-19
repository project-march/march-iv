import smach
import rospy
from controller_manager_msgs.srv import *


class WaitForRosControlState(smach.State):
    """State which waits for the ros controller manager to load the controllers."""

    def __init__(self, timeout=60):
        self._timeout = rospy.Duration.from_sec(timeout)
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failed'])

    def execute(self, userdata):
        end = rospy.get_rostime() + self._timeout
        rospy.logdebug('trying to find controller manager')
        list_controllers = rospy.ServiceProxy('/march/controller_manager/list_controllers', ListControllers)
        try:
            list_controllers.wait_for_service()
        except rospy.exceptions.ROSInterruptException:
            rospy.logdebug('ROS was shutdown while waiting for controller service')
            return 'preempted'

        rospy.logdebug('controller manager found')

        req = ListControllersRequest()
        while not rospy.is_shutdown():
            if rospy.get_rostime() > end:
                rospy.logwarn('unable to find a JointTrajectoryController')
                return 'failed'
            try:
                controller_list = list_controllers.call(req)
                for c in controller_list.controller:
                    if c.type == 'effort_controllers/JointTrajectoryController' or \
                            c.type == 'position_controllers/JointTrajectoryController':
                        rospy.logdebug('found ros control')
                        return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn('Service call failed: %s' % e)
                return 'failed'
