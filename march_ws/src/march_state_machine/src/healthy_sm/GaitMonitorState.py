import rospy
import smach

from march_custom_msgs.msg import Gait
from march_custom_msgs.msg import StepSize
from std_msgs.msg import Empty

class GaitMonitorState():
    _pub_input_gait_denied = None
    _dictionary_callbacks = {"":""}

    def __init__(self):
        _pub_input_gait_denied = rospy.Publisher("input_device/denied/gait", Empty)

        self._sub_gait = rospy.Subscriber("input_device/instruction/gait", Gait, callback_gait)
        self._sub_stop = rospy.Subscriber("input_device/instruction/stop", Empty, callback_stop)
        self._sub_trigger = rospy.Subscriber("input_device/instruction/trigger", Empty, callback_trigger)
        self._sub_step_size = rospy.Subscriber("input_device/instruction/step_size", StepSize, callback_step_size)

    def callback_gait(data):
        _pub_input_gait_denied.publish(Empty())

    def callback_stop(data):
        rospy.loginfo(data)

    def callback_trigger(data):
        rospy.loginfo(data)

    def callback_step_size(data):
        rospy.loginfo(data)
