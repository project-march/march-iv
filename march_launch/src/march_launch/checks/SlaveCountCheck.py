import rospy
from march_launch.Color import Color
from LaunchCheck import LaunchCheck


class SlaveCountCheck(LaunchCheck):

    def __init__(self):
        LaunchCheck.__init__(self, "SlaveCount", "", "march_launch", "slave_count.launch", manual_confirmation=True)

    def perform(self):
        self.launch()

        rospy.sleep(rospy.Duration.from_sec(5))
        self.stop_launch_process()
        slave_count = self.get_key_from_parameter_server("/check/slave_count")

        self.log("Ethercat found " + str(slave_count) + " slave(s)", Color.Info)
        self.passed = slave_count > 0
        self.done = True
