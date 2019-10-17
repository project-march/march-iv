import rospy
import smach
import time
import roslaunch
import subprocess


##
# The Launch State.
# This state starts a launch file with the needed arguments.
class LaunchState(smach.State):

    ##
    # @brief The Constructor.
    #
    # @details
    # @param [in] package_name The name of the package where the launch file belong to.
    #
    # @param [in] launch_file_name The name of the launch file that should be executed.
    #
    # @param [in] launch_is_optional_key This key means that the package is optional.
    # When the key is set, that key should also be on the ros parameter server.
    # If the value of this parameter is true the package is executed.
    # When the value is false the package is skipped.
    # This allows the system on run-time to choose which packages to start.
    # Not only in advance based on the run-configuration,
    # but also based on whether the corresponding checkbox is selected.
    #
    # @param [in] launch_arguments Additional parameters for the target launch file.
    # For example a gazebo_ui argument can be used to enable the ui in gazebo.
    def __init__(self, package_name, launch_file_name,
                 launch_is_optional_key='', launch_arguments={}):
        self.package_name = package_name
        self.launch_file_name = launch_file_name
        self.launch_arguments = launch_arguments
        self.launch_is_optional_key = launch_is_optional_key
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    ##
    # @brief This state is only launching a launch file.
    #
    # @details
    # @param [in] userdate This data is passed onto this state.
    # In the launchState the userdata is expected to be empty.
    # @param [in] self This is the object pointer.
    # @return The outcome of this state: `succeeded` or `failed`.
    def execute(self, userdata):
        if not self.should_execute_launch_file():
            rospy.loginfo("Do not launch " + self.launch_file_name)
            return 'succeeded'

        rospy.loginfo(
            "Launch " + self.package_name + " " + self.launch_file_name)
        return self.execute_launch_file()

    ##
    # @brief Add launch parameters to the command.
    # Only when there are arguments defined in launch_arguments.
    # With the param_name the value is obtained from the ros param server.
    # The param name should be the same as defined in the launch file.
    # @param [in] command This is the command to start a launch file.
    # @param [in] self This is the object pointer.
    # @return The new command with the parameters added.
    def add_parameters_to_command(self, command):
        for param_name, param_path in self.launch_arguments.iteritems():
            if rospy.has_param("march/launch_settings" + param_path):
                command += " {0}:={1}".format(param_name, rospy.get_param(
                    "march/launch_settings" + param_path))
        return command

    ##
    # @brief Check if this launch file should be skipped
    #     or not depending on the ros params.
    # @param [in] self This is the object pointer.
    # @return A boolean indicating if the launch file should be launched.
    def should_execute_launch_file(self):
        if self.launch_is_optional_key == "":
            # If there is no key, this launch file is not optional
            return True
        if not rospy.has_param(
                "march/launch_settings" + self.launch_is_optional_key):
            # If the parameter is not set do not execute
            # the launch file and give a warning.
            rospy.logwarn(
                "Expected ros parameter not found: march/launch_settings" +
                self.launch_is_optional_key)
            return False
        # Simply return the value of the parameter as result.
        return rospy.get_param(
            "march/launch_settings" + self.launch_is_optional_key)

    ##
    # @brief Create a command that launches the launch file,
    #     based on the parameters passed in the constructor.
    # @param [in] self This is the object pointer.
    # @return The outcome of this state: `succeeded` or `failed`.
    def execute_launch_file(self):
        command = "roslaunch {0} {1}".format(self.package_name,
                                             self.launch_file_name)
        command = self.add_parameters_to_command(command)

        # Execute the command with shell features enabled
        popen = subprocess.Popen(command, shell=True)
        # Check if the process has terminated
        return_state = popen.poll()
        if return_state is None:
            return 'succeeded'
        else:
            rospy.loginfo("Process terminated")
            return 'failed'
