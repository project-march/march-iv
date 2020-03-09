import rospy

from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    rospy.init_node('march_gain_scheduling_node')

    while not rospy.is_shutdown() and not rospy.has_param('/march/joint_names'):
        rospy.sleep(0.5)
        rospy.logdebug('Waiting on /march/joint_names to be available')

    if rospy.is_shutdown():
        return

    joint_list = rospy.get_param('/march/joint_names')
    DynamicPIDReconfigurer(joint_list)

    rospy.spin()
