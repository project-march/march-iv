#!/usr/bin/env python
import os

import yaml
import rospkg
import rospy
import socket

from march_shared_resources.msg import Subgait
from trajectory_msgs.msg import JointTrajectory
from rospy_message_converter import message_converter
from urdf_parser_py import urdf


class GaitSelection(object):

    def __init__(self, package, directory):
        try:
            default_yaml = os.path.join(rospkg.RosPack().get_path(package), directory, 'default.yaml')
        except rospkg.common.ResourceNotFound:
            rospy.logerr("Could not find package %s, Shutting down.", package)
            raise Exception("Could not find package " + package + ", Shutting down.", )
        try:
            default_config = yaml.load(open(default_yaml), Loader=yaml.SafeLoader)
        except IOError:
            rospy.logerr("Could not find %s/%s/default.yaml, Shutting down.", package, directory)
            raise Exception("Could not find " + package + "/" + directory + "/default.yaml, Shutting down.")

        self.gait_directory = os.path.join(rospkg.RosPack().get_path(package), directory)

        self.gait_version_map = default_config["gaits"]
        self.loaded_subgaits = None
        self.joint_names = []
        self.robot = None

        try:
            self.robot = urdf.Robot.from_parameter_server()

            for joint in self.robot.joints:
                if joint.type != "fixed":
                    self.joint_names.append(joint.name)

        except KeyError:
            rospy.logwarn("No urdf found, cannot filter out unused joints. "
                          "The gait selection will publish gaits with all joints.")
        except socket.error:
            rospy.loginfo("Could not connect to parameter server.")

        rospy.loginfo("GaitSelection initialized with gait_directory %s/%s.", package, directory)
        rospy.logdebug("GaitSelection initialized with gait_version_map %s.", str(self.gait_version_map))

        self.load_subgait_files()

    def set_gait_version_map(self, gait_version_map):
        self.gait_version_map = gait_version_map
        self.load_subgait_files()

    def set_subgait_version(self, gait_name, subgait_name, version):
        if self.validate_version_name(gait_name, subgait_name, version):
            self.gait_version_map[gait_name][subgait_name] = version
            self.load_subgait_files()
            return True
        return False

    def load_subgait_files(self):
        rospy.logdebug("Loading subgait files")
        self.loaded_subgaits = {}

        for gait in self.gait_version_map:
            self.loaded_subgaits[gait] = {}
            for subgait in self.gait_version_map[gait]:
                self.loaded_subgaits[gait][subgait] = self.load_subgait(gait, subgait)

    def get_subgait(self, gait_name, subgait_name):
        try:
            return self.loaded_subgaits[gait_name][subgait_name]
        except KeyError:
            return None

    def load_subgait(self, gait_name, subgait_name):
        try:
            subgait_path = self.get_subgait_path(gait_name, subgait_name)
        except KeyError as e:
            rospy.logerr(str(e))
            return None

        subgait_yaml = yaml.load(open(subgait_path), Loader=yaml.SafeLoader)

        try:
            subgait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/Subgait',
                                                                          subgait_yaml)
        except ValueError as e:
            rospy.logerr(str(e))
            rospy.logerr("Could not load subgait " + gait_name + "/" + subgait_name + " from " + subgait_path)
            return None

        subgait.name = subgait_name
        subgait.version = self.gait_version_map[gait_name][subgait_name]

        if self.robot is not None:
            subgait = self.filter_subgait(subgait, self.joint_names)

        return subgait

    def filter_subgait(self, subgait, joint_names):
        """Remove joints from the subgait if they are not present in the urdf."""

        for i in reversed(range(0, len(subgait.trajectory.joint_names))):
            joint = subgait.trajectory.joint_names[i]
            if joint not in joint_names:
                del subgait.trajectory.joint_names[i]
                for j in range(0, len(subgait.trajectory.points)):
                    if len(subgait.trajectory.points[j].positions) > 0:
                        del subgait.trajectory.points[j].positions[i]
                    if len(subgait.trajectory.points[j].velocities) > 0:
                        del subgait.trajectory.points[j].velocities[i]
                    if len(subgait.trajectory.points[j].accelerations) > 0:
                        del subgait.trajectory.points[j].acceleration[i]
                    if len(subgait.trajectory.points[j].effort) > 0:
                        del subgait.trajectory.points[j].effort[i]

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
        if set(old_trajectory.joint_names) != set(new_trajectory.joint_names):
            rospy.logwarn("Joint names are not equal: %s, %s",
                          str(old_trajectory.joint_names), str(new_trajectory.joint_names))
            return False

        last_old_point_positions = set(zip(old_trajectory.joint_names, old_trajectory.points[-1].positions))
        last_old_point_velocities = set(zip(old_trajectory.joint_names, old_trajectory.points[-1].velocities))
        last_old_point_accelerations = set(zip(old_trajectory.joint_names, old_trajectory.points[-1].accelerations))
        last_old_point_effort = set(zip(old_trajectory.joint_names, old_trajectory.points[-1].effort))

        first_new_point_positions = set(zip(new_trajectory.joint_names, new_trajectory.points[0].positions))
        first_new_point_velocities = set(zip(new_trajectory.joint_names, new_trajectory.points[0].velocities))
        first_new_point_accelerations = set(zip(new_trajectory.joint_names, new_trajectory.points[0].accelerations))
        first_new_point_effort = set(zip(new_trajectory.joint_names, new_trajectory.points[0].effort))

        return last_old_point_positions == first_new_point_positions and \
            last_old_point_velocities == first_new_point_velocities and \
            last_old_point_accelerations == first_new_point_accelerations and \
            last_old_point_effort == first_new_point_effort
