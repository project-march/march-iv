#!/usr/bin/env python
import rospy
from dynamic_reconfigure.client import Client
from march_shared_resources.msg import GaitNameActionGoal


class DynamicPIDReconfigurer:
    def __init__(self, gait_name=None, joint_list=None):
        self._gait_name = gait_name
        self._joint_list = joint_list
        self._clients = []
        for i in range(len(self._joint_list)):
            self._clients.append(Client("/march/controller/trajectory/gains/" + self._joint_list[i], timeout=30))
        rospy.Subscriber("march/gait/perform/goal", GaitNameActionGoal, callback=self.gait_selection_callback)

    def gait_selection_callback(self, data):
        rospy.logdebug("This is the gait name: %s", data.goal.name)
        if self._gait_name != data.goal.name:
            rospy.logdebug("The selected gait: {0} is not the same as the previous gait: {1}".format(
                data.goal.name, self._gait_name))
            self._gait_name = data.goal.name
            self.client_update()

    def client_update(self):
        for i in range(len(self._joint_list)):
            p_value, i_value, d_value = self.look_up_table(i)
            self._clients[i].update_configuration({"p": p_value,
                                                   "i": i_value,
                                                   "d": d_value})
            rospy.logdebug("Config set to {0}, {1}, {2}".format(p_value, i_value, d_value))

    # Method that functions as a lookup table. The implementation now only functions as a test and should be adjusted
    # for working with a config file which stores all the tuned values. Having a lookup table within your node is very
    # untidy.
    def look_up_table(self, i):
        if self._gait_name == "sit":
            return 10001, 0, 10
        elif self._gait_name == "walk":
            return 10002, 0, 10
        elif self._gait_name == "stand":
            return 10003, 0, 10
        elif self._joint_list[i] == "right_ankle" and self._gait_name == "sit_down":
            return 10004, 0, 10
        elif self._joint_list[i] == "left_hip_fe" and self._gait_name == "sit_down":
            return 10005, 0, 10
        elif self._joint_list[i] == "right_hip_fe" and self._gait_name == "sit_down":
            return 10006, 0, 10
        elif self._joint_list[i] == "left_knee" and self._gait_name == "sit_down":
            return 10007, 0, 10
        elif self._joint_list[i] == "right_knee" and self._gait_name == "sit_down":
            return 10008, 0, 10
        else:
            return 1000, 0, 10
