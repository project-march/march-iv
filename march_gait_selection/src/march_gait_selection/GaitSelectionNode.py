#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib
import ast

from std_srvs.srv import Trigger
from march_shared_resources.srv import StringTrigger

from march_shared_resources.msg import GaitNameAction, GaitAction, GaitGoal

from GaitSelection import GaitSelection


class PerformGaitAction(object):

    def __init__(self, gait_selection):
        self.gait_selection = gait_selection
        self.action_server = actionlib.SimpleActionServer("march/gait/perform", GaitNameAction,
                                                          execute_cb=self.target_gait_callback,
                                                          auto_start=False)
        self.action_server.start()
        self.schedule_gait_client = actionlib.SimpleActionClient("march/gait/schedule", GaitAction)

    def target_gait_callback(self, goal):
        rospy.loginfo("Trying to schedule subgait %s/%s", goal.name, goal.subgait_name)
        if not self.gait_selection.validate_gait_by_name(goal.name):
            rospy.logerr("Gait %s is invalid", goal.name)
            self.action_server.set_succeeded(False)
            return False
        subgait = self.gait_selection.get_subgait(goal.name, goal.subgait_name)
        trajectory_result = self.schedule_gait(goal.name, subgait)
        # @TODO(Isha, Tim) monitor the scheduled gait and pass feedback to the state machine.
        self.action_server.set_succeeded(True)

    def schedule_gait(self, gait_name, subgait):
        gait_action_goal = GaitGoal()
        gait_action_goal.name = gait_name
        gait_action_goal.current_subgait = subgait

        self.schedule_gait_client.send_goal(gait_action_goal)

        self.schedule_gait_client.wait_for_result(timeout=subgait.duration + rospy.Duration(1))
        return self.schedule_gait_client.get_result()


def set_selected_version_callback(msg, gait_selection):
    try:
        string = msg.string.replace(".subgait", "")
        gait_name, subgait_name, version_name = string.split("/")
    except ValueError:
        return [False, "Could not split gait " + msg.string + "."]

    if gait_selection.set_subgait_version(gait_name, subgait_name, version_name):
        return [True, "Subgait " + gait_name + "/" + subgait_name + " now uses version " + version_name + "."]
    return [False, "Version " + gait_name + "/" + subgait_name + "/" + version_name + ".subgait is not valid."]


def set_gait_version_map(msg, gait_selection):
    try:
        map = ast.literal_eval(msg.string)
    except ValueError:
        return [False, "Not a valid dictionary " + str(msg.string)]

    if gait_selection.validate_version_map(map):
        return [True, "Gait version map set to " + str(gait_selection.gait_version_map)]
    return [False, "Gait version map is not valid " + str(map)]


if __name__ == '__main__':
    rospy.init_node("gait_selection")

    default_yaml = os.path.join(rospkg.RosPack().get_path('march_gait_selection'), 'gait', 'default.yaml')
    gait_selection = GaitSelection(default_yaml=default_yaml)
    perform_gait_server = PerformGaitAction(gait_selection)

    # Use lambdas to process service calls inline
    get_gait_version_map_service = rospy.Service('march/gait_selection/get_version_map', Trigger,
                                                 lambda msg: [True,
                                                              str(perform_gait_server.gait_selection.gait_version_map)])

    get_all_gait_files_service = rospy.Service('march/gait_selection/get_directory_structure', Trigger,
                                               lambda msg: [True,
                                                            str(perform_gait_server.gait_selection.scan_directory())])

    set_selected_version_service = rospy.Service('march/gait_selection/set_version', StringTrigger,
                                                 lambda msg: set_selected_version_callback(
                                                     msg, perform_gait_server.gait_selection))

    set_gait_version_map_service = rospy.Service('march/gait_selection/set_version_map', StringTrigger,
                                                 lambda msg: set_gait_version_map(
                                                    msg, perform_gait_server.gait_selection))

    perform_gait_server.schedule_gait_client.wait_for_server()

    rate = rospy.Rate(10)
    rospy.spin()