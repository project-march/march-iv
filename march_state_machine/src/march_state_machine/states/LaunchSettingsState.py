import rospy
import smach
import time
import subprocess
from std_msgs.msg import Empty


##
# Launch settings State this starts a configuration view to decide what should start-up.
class LaunchSettingsState(smach.State):

    ##
    # @brief The Constructor.
    # @details
    # @param [in] outcomes All possible outcomes of this state
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['exoskeleton', 'simulation', 'failed'])

    ##
    # @brief This state only launches the the `rqt_launch_menu`.
    # @details
    # @param [in] userdate This data is passed onto this state.
    # In the launchState the userdata is expected to be empty.
    # @param [in] self This is the object pointer.
    # @return The outcome of this state:
    #     `exoskeleton`, `failed` or `simulation`.
    def execute(self, userdata):
        if self.should_execute_rqt_launch():
            self.start_rqt_launch_menu()

        # Wait until the run configuration is chosen in the rqt_launch_menu
        # TODO(Tim) determine the rate we use for these cases
        # 100 is fast enough, but there is not specific reason to make this 100
        rate = rospy.Rate(100)
        while not rospy.has_param("/march/launch_settings/run_configuration") and not rospy.core.is_shutdown():
            rate.sleep()

        return rospy.get_param("/march/launch_settings/run_configuration")

    ##
    # @brief Check if the `rqt_launch_menu` should be launched or not.
    # @details
    # @return Boolean that indicates if the `rqt_launch_menu` should be started.
    @staticmethod
    def should_execute_rqt_launch():
        if rospy.has_param("march/launch_settings/rqt_launch_menu"):
            return rospy.get_param("march/launch_settings/rqt_launch_menu")
        # On default do not skip this state
        return True

    ##
    # @brief Create and execute the command to launch rqt_launch_menu
    # @details
    # @return The return state of executing the launch command.
    @staticmethod
    def start_rqt_launch_menu():
        command = "roslaunch march_rqt_launch_menu " \
                  "march_rqt_launch_menu.launch"
        popen = subprocess.Popen(command, shell=True)
        state = popen.poll()
        if state is None:
            return 'succeeded'
        else:
            rospy.logerr("Process terminated")
            return 'failed'
