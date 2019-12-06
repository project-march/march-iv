
import os

import rospkg
import rospy
from urdf_parser_py import urdf
import yaml

from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError
from march_shared_classes.gait.gait import Gait


class GaitSelection(object):
    """Base class for the gait selection module."""

    def __init__(self, package, directory):
        self.loaded_gaits = None
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
        self._gait_version_map = default_config['gaits']

        self.robot = urdf.Robot.from_parameter_server('/robot_description')
        self.joint_names = [joint.name for joint in self.robot.joints if joint.type != 'fixed']

        rospy.loginfo('GaitSelection initialized with package: {pk} of directory {dr}'.format(pk=package, dr=directory))
        rospy.logdebug('GaitSelection initialized with gait_version_map: {vm}'.format(vm=str(self.gait_version_map)))

        self.load_gaits()

    @staticmethod
    def get_ros_package_path(package):
        """Get the path of where the given (ros) package is located."""
        try:
            return rospkg.RosPack().get_path(package)
        except rospkg.common.ResourceNotFound:
            raise PackageNotFoundError(package)

    @property
    def gait_version_map(self):
        """Get the version map of the gait selection module as property."""
        return self._gait_version_map

    @gait_version_map.setter
    def gait_version_map(self, new_version_map):
        """Set new version map and reload the gaits from the directory."""
        self._gait_version_map = new_version_map
        self.load_gaits()

    def load_gaits(self):
        """Load the gaits in the specified gait directory."""
        self.loaded_gaits = list()

        for gait in self._gait_version_map:
            loaded_gait = Gait.from_file(gait, self.gait_directory, self.robot, self._gait_version_map)
            self.loaded_gaits.append(loaded_gait)

    def scan_directory(self):
        """Scan the gait_directory recursively and create a dictionary of all subgait files.

        :returns:
            dictionary of the maps and files within the directory
        """
        root_dir = self.gait_directory.rstrip(os.sep)

        directory_dict = {}
        for gait in os.listdir(root_dir):
            gait_path = os.path.join(root_dir, gait)

            if os.path.isdir(gait_path):
                gait_dict = {'image': os.path.join(gait_path, gait + '.png'), 'subgaits': {}}

                for subgait in os.listdir(gait_path):
                    subgait_path = os.path.join(gait_path, subgait)

                    if os.path.isdir(subgait_path):
                        versions = []
                        for version in os.listdir(os.path.join(subgait_path)):
                            if version.endswith('.subgait'):
                                versions.append(version.replace('.subgait', ''))

                        versions.sort()
                        gait_dict['subgaits'][subgait] = versions

                    directory_dict[gait] = gait_dict
        return directory_dict

    def validate_versions_in_directory(self, new_gait_version_map):
        """Validate if the given version numbers in the version map exist in the selected directory."""
        for gait_name in new_gait_version_map:
            for subgait_name in new_gait_version_map[gait_name]:
                version = new_gait_version_map[gait_name][subgait_name]
                subgait_path = os.path.join(self.gait_directory, gait_name, subgait_name, version + '.subgait')

                if not os.path.isfile(subgait_path):
                    return False
        return True

    def validate_gait_in_directory(self, gait_name):
        """Check if the gait file exists in the given directory."""
        gait_map = gait_name
        gait_path = os.path.join(self.gait_directory, gait_map, gait_name + '.gait')

        if not os.path.isfile(gait_path):
            raise FileNotFoundError(gait_path)

    def __getitem__(self, name):
        """Get a gait from the loaded gaits."""
        return next((gait for gait in self.loaded_gaits if gait.gait_name == name), None)
