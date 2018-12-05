import rospy
import smach

from march_custom_msgs.msg import Gait
from march_custom_msgs.msg import StepSize
from std_msgs.msg import Empty


class GaitMonitorState():
    def __init__(self):
        topic_name_gait = "input_device/instruction/gait"
        topic_name_stop = "input_device/instruction/stop"
        topic_name_trigger = "input_device/instruction/trigger"
        topic_name_step_size = "input_device/instruction/step_size"

        self._sub_gait = rospy.Subscriber(topic_name_gait, Gait, callback_gait)
        self._sub_stop = rospy.Subscriber(topic_name_stop, Empty, callback_stop)
        self._sub_trigger = rospy.Subscriber(topic_name_trigger, Empty, callback_trigger)
        self._sub_step_size = rospy.Subscriber(topic_name_step_size, StepSize, callback_step_size)


    def callback_gait(data):
        rospy.loginfo(data)

    def callback_stop(data):
        rospy.loginfo(data)

    def callback_trigger(data):
        rospy.loginfo(data)

    def callback_step_size(data):
        rospy.loginfo(data)
