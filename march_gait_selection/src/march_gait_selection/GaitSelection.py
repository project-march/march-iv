
import os

import yaml
import rospkg
import rospy
import socket

from urdf_parser_py import urdf

from march_shared_classes.gait.subgait import Subgait
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError
from march_shared_classes.exceptions.gait_exceptions import *


class GaitSelection(object):
    """Base class for the gait selection module"""
    def __init__(self, package, directory):
        self.gait_version_map = None
        self.loaded_subgaits = None
        self.gait_directory = None
        self.joint_names = list()
        self.robot = None

        package_path = self.get_ros_package_path(package)
        default_yaml = os.path.join(package_path, directory, 'default.yaml')

        if not os.path.isfile(default_yaml):
            raise FileNotFoundError(file_path=default_yaml)

        with open(default_yaml, 'r') as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        self.gait_directory = os.path.join(package_path, directory)
        self.gait_version_map = default_config["gaits"]

        try:
            self.robot = urdf.Robot.from_parameter_server('/robot_description')
            self.joint_names = [joint.name for joint in self.robot.joints if joint.type != "fixed"]

        except KeyError:
            rospy.logwarn("No urdf found, cannot filter unused joints, Gait selection will publish all joints.")

        except socket.error:
            rospy.logerr("Could not connect to parameter server.")

        rospy.loginfo("GaitSelection initialized with package: {pk} of directory {dr}".format(pk=package, dr=directory))
        rospy.logdebug("GaitSelection initialized with gait_version_map: {vm}".format(vm=str(self.gait_version_map)))

        self.load_subgait_files()

    @staticmethod
    def get_ros_package_path(package):
        """Get the path of where the given (ros) package is located"""
        try:
            return rospkg.RosPack().get_path(package)
        except rospkg.common.ResourceNotFound:
            raise PackageNotFoundError(package)

    def get_subgait_path(self, gait_name, subgait_name):
        """Construct the subgait file path with the given gait and subgait name"""
        self.validate_gaits_in_map(gait_name, subgait_name)

        subgait_path = os.path.join(self.gait_directory, gait_name, subgait_name,
                                    self.gait_version_map[gait_name][subgait_name] + '.subgait')

        if not os.path.isfile(subgait_path):
            raise FileNotFoundError(file_path=subgait_path)

        return subgait_path

    def get_subgait_from_loaded_subgaits(self, gait_name, subgait_name):
        """Get the sub gait data from gait and subgait name"""
        return self.loaded_subgaits[gait_name][subgait_name]

    def set_gait_version_map(self, gait_version_map):
        """Set gait version map and reload subgait files
        :param gait_version_map:
            list of gaits from yaml file
        """
        self.gait_version_map = gait_version_map
        self.load_subgait_files()

    def set_subgait_version(self, gait_name, subgait_name, version):
        """Set the sub gait version to a specific sub gait and reload subgait files
        :param gait_name:
            name of the gait in which the subgait is placed
        :param subgait_name:
            name of the subgait
        :param version:
            new version to set to parameter

        :returns
            if input was valid and version is set return True else False
        """
        if self.validate_version_file(self.gait_directory, gait_name, subgait_name, version):
            self.gait_version_map[gait_name][subgait_name] = version
            self.load_subgait_files()
            return True
        return False

    def validate_gaits_in_map(self, gait_name=None, subgait_name=None):
        """Validate the given name and subgait in the parsed map"""
        if gait_name is not None:
            if self.gait_version_map.get(gait_name) is None:
                raise GaitNameNotFound(gait_name)

            if subgait_name is not None:
                if self.gait_version_map[gait_name].get(subgait_name) is None:
                    raise SubgaitNameNotFound(subgait_name)

        return True

    @staticmethod
    def validate_version_file(gait_directory, gait_name, subgait_name, version):
        """Validate if the given gait name, subgait name and version form file path"""
        if any(len(directory_name) == 0 for directory_name in (gait_name, subgait_name, version)):
            return False

        subgait_path = os.path.join(gait_directory, gait_name, subgait_name, version + '.subgait')
        if os.path.isfile(subgait_path):
            return True

        return False

    def validate_version_in_map(self, gait_map):
        """Check if the given version in the gait mapping exists within the directory"""
        for gait in gait_map:
            for subgait in gait_map[gait]:
                version = gait_map[gait][subgait]
                if not self.validate_version_file(self.gait_directory, gait, subgait, version):
                    return False
        return True

    def validate_gait_file(self, gait_name):
        """Validate if the given gait name exists in the gait mapping
        NOTE: the gait has a gait map which has an identical name
        """
        gait_map = gait_name
        gait_path = os.path.join(self.gait_directory, gait_map, gait_name + ".gait")
        if not os.path.isfile(gait_path):
            raise FileNotFoundError(gait_path)

        return self.validate_gait_content(gait_path)

    def load_subgait(self, gait_name, subgait_name):
        """Read the .subgait file and extract the data
        :returns
            if gait and subgait names are valid return populated SubGaitSelector object
        """
        subgait_path = self.get_subgait_path(gait_name, subgait_name)
        subgait = Subgait.from_file(self.robot, subgait_path)

        return subgait

    def load_subgait_files(self):
        """extract data from the subgait file and set to the corresponding subgait parameter"""
        rospy.logdebug("Loading subgait files")
        self.loaded_subgaits = {}

        for gait in self.gait_version_map:
            self.loaded_subgaits[gait] = {}
            for subgait in self.gait_version_map[gait]:
                self.loaded_subgaits[gait][subgait] = self.load_subgait(gait, subgait)

    def scan_directory(self):
        """Scan the gait_directory recursively and create a dictionary of all subgait files"""
        root_dir = self.gait_directory.rstrip(os.sep)

        directory_dict = {}
        for gait in os.listdir(root_dir):
            gait_path = os.path.join(root_dir, gait)

            if os.path.isdir(gait_path):
                gait_dict = {"image": os.path.join(gait_path, gait + '.png'), "subgaits": {}}

                for subgait in os.listdir(gait_path):
                    subgait_path = os.path.join(gait_path, subgait)

                    if os.path.isdir(subgait_path):
                        versions = []
                        for version in os.listdir(os.path.join(subgait_path)):
                            if version.endswith('.subgait'):
                                versions.append(version.replace('.subgait', ''))

                        versions.sort()
                        gait_dict["subgaits"][subgait] = versions

                    directory_dict[gait] = gait_dict
        return directory_dict

    def validate_gait_content(self, gait_path):
        """Validate if a start and end  point is defined in the gait"""
        with open(gait_path, 'r') as gait_file:
            gait = yaml.load(gait_file, Loader=yaml.SafeLoader)

        gait_name = gait['name']
        gait_content = gait['graph']

        if len(gait_content['from_subgait']) != len(gait_content['to_subgait']):
            raise NonValidGaitContent(msg='Gait {gn} does not have equal amount of subgaits'.format(gn=gait_name))

        if 'start' not in gait_content['from_subgait'] or 'start' in gait_content['to_subgait']:
            raise NonValidGaitContent(msg='Gait {gn} does not have valid starting subgaits'.format(gn=gait_name))

        if 'end' not in gait_content['to_subgait'] or 'end' in gait_content['from_subgait']:
            raise NonValidGaitContent(msg='Gait {gn} does not have valid ending subgaits'.format(gn=gait_name))

        from_subgaits = gait_content['from_subgait']
        to_subgaits = gait_content['to_subgait']

        subgaits = from_subgaits + to_subgaits
        subgaits = filter(lambda name: name not in ('start', 'end'), subgaits)

        if not all(self.validate_gaits_in_map(gait_name, subgait_name) for subgait_name in subgaits):
            raise NonValidGaitContent(msg='Subgaits in gait {gn} do not exist in the mapped subgait'.format(gn=gait_name))

        if all(self._validate_trajectory_transition(gait_name, from_subgait, to_subgait) for from_subgait, to_subgait
               in zip(from_subgaits, to_subgaits)):
            return True

    def _validate_trajectory_transition(self, gait_name, from_subgait_name, to_subgait_name):
        """Compare and validate the trajectory end and start points
        :param gait_name:
            Name of the general gait to which these subgaits are assigned to
        :param from_subgait_name:
            a subgait of the list of subgaits defined in the from_subgait header of the gait file
        :param to_subgait_name:
            a subgait of the list of subgaits defined in the to_subgait header of the gait file
        """
        if all(name not in ('start', 'end') for name in (from_subgait_name, to_subgait_name)):

            from_subgait_obj = self.get_subgait_from_loaded_subgaits(gait_name, from_subgait_name)
            to_subgait_obj = self.get_subgait_from_loaded_subgaits(gait_name, to_subgait_name)

            from_subgait_joint_names = set(from_subgait_obj.get_joint_names())
            to_subgait_joint_names = set(to_subgait_obj.get_joint_names())

            if from_subgait_joint_names != to_subgait_joint_names:
                raise NonValidGaitContent(msg='Structure of joints does not match between subgait {} and subgait {}'
                                          .format(from_subgait_name, to_subgait_name))

            for fs_joint_name, ts_joint_name in zip(from_subgait_joint_names, to_subgait_joint_names):
                fs_joint_point = from_subgait_obj.get_joint(name=fs_joint_name).get_setpoints_unzipped()
                ts_joint_point = to_subgait_obj.get_joint(name=ts_joint_name).get_setpoints_unzipped()

                for index, (fs_point_data, ts_point_data) in enumerate(zip(fs_joint_point, ts_joint_point)):
                    if index == 0:
                        continue  # timestamps should not be compared

                    if fs_point_data[-1] != ts_point_data[0]:
                        raise NonValidGaitContent(msg="End setpoint of joint {} in subgait {} does not match"
                                                  .format(fs_joint_name, from_subgait_name))

        return True
