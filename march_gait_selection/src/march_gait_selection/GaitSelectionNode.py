import ast
import os

import rospkg
import rospy
from std_srvs.srv import Trigger
import yaml


from march_shared_classes.exceptions.gait_exceptions import GaitError
from march_shared_resources.srv import StringTrigger

from .GaitSelection import GaitSelection
from .PerformGaitAction import PerformGaitAction


NODE_NAME = 'gait_selection'
GAIT_FILES_MAP_NAME = 'march_gait_files'
GAIT_DIRECTORY_NAME = 'gait'


def set_gait_version_map(msg, gait_selection):
    """Set a new gait version map to the gait selection class."""
    backup_map = gait_selection.gait_version_map

    try:
        new_gait_version_map = ast.literal_eval(msg.string)

        if not gait_selection.validate_versions_in_directory(new_gait_version_map):
            return [False, 'Gait version map: {gm}, is not valid'.format(gm=new_gait_version_map)]

        gait_selection.gait_version_map = new_gait_version_map

        return [True, 'Gait version map set to: \n {gm}'.format(gm=str(gait_selection.gait_version_map))]

    except ValueError:
        return [False, 'Not a valid dictionary: {msg}'.format(msg=str(msg.string))]

    except GaitError as e:
        gait_selection.gait_version_map = backup_map
        return [False, 'Error occurred when constructing gaits: {er}'.format(er=e)]


def update_default_versions(gait_package, gait_directory, gait_version_map):
    """Update the default.yaml file in the given directory."""
    default_yaml = os.path.join(rospkg.RosPack().get_path(gait_package), gait_directory, 'default.yaml')
    new_default_dict = {'gaits': gait_version_map}

    try:
        with open(default_yaml, 'w') as default_yaml_content:
            yaml_content = yaml.dump(new_default_dict)
            default_yaml_content.write(yaml_content)

        return [True, 'New default values were written to: {pn}'.format(pn=default_yaml)]

    except IOError:
        return [False, 'Error occurred when writing to file path: {pn}'.format(pn=default_yaml)]


def main():
    rospy.init_node(NODE_NAME)
    gait_package = rospy.get_param('~gait_package', GAIT_FILES_MAP_NAME)
    gait_directory = rospy.get_param('~gait_directory', GAIT_DIRECTORY_NAME)

    gait_selection = GaitSelection(gait_package, gait_directory)

    # Use lambdas to process service calls inline
    rospy.Service('/march/gait_selection/get_version_map', Trigger,
                  lambda msg: [True, str(gait_selection.gait_version_map)])

    rospy.Service('/march/gait_selection/set_version_map', StringTrigger,
                  lambda msg: set_gait_version_map(msg, gait_selection))

    rospy.Service('/march/gait_selection/get_directory_structure', Trigger,
                  lambda msg: [True, str(gait_selection.scan_directory())])

    rospy.Service('/march/gait_selection/update_default_versions', Trigger,
                  lambda msg: update_default_versions(gait_package, gait_directory, gait_selection.gait_version_map))

    PerformGaitAction(gait_selection)
    rospy.spin()
