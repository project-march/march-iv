import ast
import os

import rospkg
import rospy
from std_srvs.srv import Trigger
import yaml

from march_shared_resources.srv import StringTrigger

from .GaitSelection import GaitSelection
from .PerformGaitAction import PerformGaitAction


def set_selected_version_callback(msg, gait_selection):
    try:
        string = msg.string.replace('.subgait', '')
        gait_name, subgait_name, version_name = string.split('/')
    except ValueError:
        return [False, 'Could not split gait ' + msg.string + '.']

    if gait_selection.set_subgait_version(gait_name, subgait_name, version_name):
        return [True, 'Subgait ' + gait_name + '/' + subgait_name + ' now uses version ' + version_name + '.']
    return [False, 'Version ' + gait_name + '/' + subgait_name + '/' + version_name + '.subgait is not valid.']


def set_gait_version_map(msg, gait_selection):
    try:
        gait_map = ast.literal_eval(msg.string)
    except ValueError:
        return [False, 'Not a valid dictionary ' + str(msg.string)]

    if not gait_selection.validate_version_in_map(map):
        return [False, 'Gait version map is not valid ' + str(map)]

    backup_map = gait_selection.gait_version_map
    gait_selection.set_gait_version_map(map)
    for gait in map:
        if not gait_selection.validate_gait_file(gait):
            gait_selection.set_gait_version_map(backup_map)
            return [False, 'Gait ' + gait + ' is invalid']
    return [True, 'Gait version map set to ' + str(gait_selection.gait_version_map)]


def update_default_versions(gait_package, gait_directory, gait_version_map):
    default_yaml = os.path.join(rospkg.RosPack().get_path(gait_package), gait_directory, 'default.yaml')
    default_dict = {'gaits': gait_version_map}
    try:
        output_file = open(default_yaml, 'w+')
        yaml_content = yaml.dump(default_dict)
        output_file.write(yaml_content)
        output_file.close()
    except IOError:
        return [False, 'Could not write to ' + default_yaml]
    return [True, 'Succesfully wrote defaults ' + str(gait_version_map) + ' to file ' + default_yaml]


def main():
    rospy.init_node('gait_selection')
    gait_package = rospy.get_param('~gait_package', 'march_gait_files')
    gait_directory = rospy.get_param('~gait_directory', 'gait')

    gait_selection = GaitSelection(gait_package, gait_directory)

    # Use lambdas to process service calls inline
    get_gait_version_map_service = rospy.Service('/march/gait_selection/get_version_map', Trigger,  # noqa F841
                                                 lambda msg: [True,
                                                              str(gait_selection.gait_version_map)])

    get_all_gait_files_service = rospy.Service('/march/gait_selection/get_directory_structure', Trigger,  # noqa F841
                                               lambda msg: [True,
                                                            str(gait_selection.scan_directory())])

    set_selected_version_service = rospy.Service('/march/gait_selection/set_version', StringTrigger,  # noqa F841
                                                 lambda msg: set_selected_version_callback(
                                                     msg, gait_selection))

    set_gait_version_map_service = rospy.Service('/march/gait_selection/set_version_map', StringTrigger,  # noqa F841
                                                 lambda msg: set_gait_version_map(
                                                     msg, gait_selection))

    update_default_versions_service = rospy.Service('/march/gait_selection/update_default_versions', Trigger,  # noqa F841
                                                    lambda msg: update_default_versions(
                                                        gait_package, gait_directory, gait_selection.gait_version_map))

    perform_gait_server = PerformGaitAction(gait_selection)  # noqa F841

    rospy.spin()
