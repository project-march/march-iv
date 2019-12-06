import rospy

from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    # The parameters Joint_list and gait_list should be obtained from the parameter server of the workspace.
    rospy.init_node('march_gain_scheduling_node')
    joint_list = rospy.get_param('/march/joint_names')
    DynamicPIDReconfigurer(joint_list=joint_list)

    rospy.spin()
