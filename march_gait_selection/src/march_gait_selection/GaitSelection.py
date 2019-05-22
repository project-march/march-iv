#!/usr/bin/env python
import os

import yaml
import rospkg
import rospy

from march_shared_resources.msg import Subgait
from trajectory_msgs.msg import JointTrajectory
from rospy_message_converter import message_converter


class GaitSelection(object):

    def __init__(self, gait_directory=None, gait_version_map=None, default_yaml=None):
        if gait_directory is None and gait_version_map is None:
            if default_yaml is None:
                rospy.logfatal("Cannot instantiate GaitSelection without parameters, shutting down.")
                raise ValueError('Cannot instantiate GaitSelection without parameters, shutting down.')
            else:
                default_config = yaml.load(open(default_yaml), Loader=yaml.SafeLoader)
                self.gait_directory = os.path.join(
                    rospkg.RosPack().get_path('march_gait_selection'), default_config["directory"])
                self.gait_version_map = default_config["gaits"]
        else:
            self.gait_directory = os.path.join(
                rospkg.RosPack().get_path('march_gait_selection'), gait_directory)
            self.gait_version_map = gait_version_map
        rospy.logdebug("GaitSelection initialized with gait_directory " + self.gait_directory)
        rospy.logdebug("GaitSelection initialized with gait_version_map " + str(self.gait_version_map))

    def set_subgait_version(self, gait_name, subgait_name, version):
        if self.validate_version_name(gait_name, subgait_name, version):
            self.gait_version_map[gait_name][subgait_name] = version
            return True
        return False

    def get_subgait(self, gait_name, subgait_name):
        try:
            subgait_path = self.get_subgait_path(gait_name, subgait_name)
        except KeyError as e:
            rospy.logerr(str(e))
            return None

        subgait_yaml = yaml.load(open(subgait_path), Loader=yaml.SafeLoader)
        subgait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/Subgait', subgait_yaml)
        subgait.name = subgait_name
        subgait.version = self.gait_version_map[gait_name][subgait_name]
        return subgait

    def validate_subgait_name(self, gait_name, subgait_name):
        try:
            self.gait_version_map[gait_name]
        except KeyError:
            rospy.logerr("Gait " + gait_name + " does not exist")
            return False
        try:
            self.gait_version_map[gait_name][subgait_name]
        except KeyError:
            rospy.logerr("Subgait " + subgait_name + " does not exist")
            return False
        return True

    def validate_version_name(self, gait_name, subgait_name, version):
        if len(gait_name) == 0 or len(subgait_name) == 0 or len(version) == 0:
            return False

        subgait_path = os.path.join(self.gait_directory, gait_name, subgait_name, version + '.subgait')
        try:
            open(subgait_path)
        except IOError:
            return False
        return True

    def get_subgait_path(self, gait_name, subgait_name):
        if not self.validate_subgait_name(gait_name, subgait_name):
            raise KeyError(
                "Could not find subgait " + gait_name + "/" + subgait_name +
                " in the mapping" + str(self.gait_version_map))
        if not self.validate_version_name(gait_name, subgait_name, self.gait_version_map[gait_name][subgait_name]):
            raise KeyError(
                "Could not find subgait file " + gait_name + "/" + subgait_name + "/" +
                self.gait_version_map[gait_name][subgait_name] + ".subgait")

        return os.path.join(self.gait_directory, gait_name, subgait_name,
                            self.gait_version_map[gait_name][subgait_name] + '.subgait')

    def scan_directory(self):
        """Scan the gait_directory recursively and create a dictionary of all subgait files"""
        rootdir = self.gait_directory.rstrip(os.sep)

        directory_dict = {}
        for gait in os.listdir(rootdir):
            gait_path = os.path.join(rootdir, gait)

            if not os.path.isdir(gait_path):
                continue

            gait_dict = {"image": os.path.join(gait_path, gait + '.png'), "subgaits": {}}

            for subgait in os.listdir(gait_path):
                subgait_path = os.path.join(gait_path, subgait)
                if not os.path.isdir(subgait_path):
                    continue
                versions = []
                for version in os.listdir(os.path.join(subgait_path)):
                    versions.append(version.replace(".subgait", ""))
                versions.sort()
                gait_dict["subgaits"][subgait] = versions
            directory_dict[gait] = gait_dict

        return directory_dict

    def validate_version_map(self, map):
        """Check if all subgaits in the map exist."""
        for gait in map:
            for subgait in map[gait]:
                version = map[gait][subgait]
                if not self.validate_version_name(gait, subgait, version):
                    return False
        return True

    def validate_gait_by_name(self, gait_name):
        gait_path = os.path.join(self.gait_directory, gait_name, gait_name + ".gait")
        try:
            gait_yaml = yaml.load(open(gait_path), Loader=yaml.SafeLoader)
        except IOError:
            rospy.logerr("No such gait at %s.", gait_path)
            return False
        gait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/GaitGoal',
                                                                   gait_yaml,
                                                                   kind="message")
        return self.validate_gait(gait)

    def validate_gait(self, gait):
        if gait is None:
            rospy.logerr("Cannot validate gait None")
            return False
        if len(gait.graph.from_subgait) != len(gait.graph.to_subgait):
            rospy.logerr("Graph has the wrong size in gait %s", gait.name)
            return False
        if "start" not in gait.graph.from_subgait:
            rospy.logerr("Start does not exist in gait %s", gait.name)
            return False
        if "end" not in gait.graph.to_subgait:
            rospy.logerr("End does not exist in gait %s", gait.name)
            return False
        if "start" in gait.graph.to_subgait:
            rospy.logerr("Gait %s has a transition to start", gait.name)
            return False
        if "end" in gait.graph.from_subgait:
            rospy.logerr("Gait %s has a transition from end", gait.name)
            return False

        for i in range(0, len(gait.graph.from_subgait)):
            if not self.validate_subgait_transition(gait.name, gait.graph.from_subgait[i], gait.graph.to_subgait[i]):
                return False
        return True

    def validate_subgait_transition(self, gait_name, from_name, to_name):
        if from_name == "start":
            if self.get_subgait(gait_name, to_name) is None:
                rospy.logerr("Could not find 'to' subgait %s/%s", gait_name, from_name)
                return False
            else:
                return True

        if to_name == "end":
            if self.get_subgait(gait_name, from_name) is None:
                rospy.logerr("Could not find 'from' subgait %s/%s", gait_name, from_name)
                return False
            else:
                return True

        from_subgait = self.get_subgait(gait_name, from_name)
        to_subgait = self.get_subgait(gait_name, to_name)

        if from_subgait is None:
            rospy.logerr("Could not find 'from' subgait %s/%s", gait_name, from_name)
            return False
        if to_subgait is None:
            rospy.logerr("Could not find 'to' subgait %s/%s", gait_name, to_name)
            return False

        if not self.validate_trajectory_transition(from_subgait.trajectory, to_subgait.trajectory):
            rospy.logerr("Wrong transition from %s/%s/%s.subgait to %s/%s/%s.subgait",
                         gait_name, from_name, from_subgait.version, gait_name, to_name, to_subgait.version)
            return False
        return True

    @staticmethod
    def validate_trajectory_transition(old_trajectory, new_trajectory):
        # Clear time_from_start as it doesn't need to match up.
        old_trajectory.points[-1].time_from_start = rospy.Duration(0)
        new_trajectory.points[0].time_from_start = rospy.Duration(0)

        return old_trajectory.joint_names == new_trajectory.joint_names \
            and old_trajectory.points[-1] == new_trajectory.points[0]
