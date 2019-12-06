
import os
import yaml

from march_shared_classes.exceptions.general_exceptions import *
from march_shared_classes.exceptions.gait_exceptions import *

from march_shared_classes.gait.subgait import Subgait


class Gait(object):
    """base class for a generated gait"""

    def __init__(self, gait_name, subgaits, from_subgaits_name, to_subgaits_names):
        self.gait_name = gait_name
        self.subgaits = subgaits
        self.from_subgaits_names = from_subgaits_name
        self.to_subgaits_names = to_subgaits_names

    @classmethod
    def from_file(cls, gait_name, gait_directory, robot, gait_version_map):
        """Extract the data from the .gait file
        :param gait_name:
            name of the gait to unpack
        :param gait_directory:
            path of the directory where the .gait file is located
        :param robot:
            the robot corresponding to the given .gait file
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits
        """
        gait_map = gait_name
        gait_path = os.path.join(gait_directory, gait_map, gait_name + '.gait')

        if not os.path.isfile(gait_path):
            raise FileNotFoundError(gait_path)

        with open(gait_path, 'r') as gait_file:
            gait = yaml.load(gait_file, Loader=yaml.SafeLoader)

        return cls.from_dict(robot, gait, gait_directory, gait_version_map)

    @classmethod
    def from_dict(cls, robot, gait_dictionary, gait_directory, gait_version_map):
        """ Create a new gait object using the .gait and .subgait files
        :param robot:
            the robot corresponding to the given .gait file
        :param gait_dictionary:
            the information of the .gait file as a dictionary
        :param gait_directory:
            path of the directory where the .gait file is located
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits

        :return:
            If the data in the files is validated a gait object is returned
        """
        gait_name = gait_dictionary['name']
        gait_content = gait_dictionary['graph']

        if len(gait_content['from_subgait']) != len(gait_content['to_subgait']):
            raise NonValidGaitContent(msg='Gait {gn} does not have equal amount of subgaits'.format(gn=gait_name))

        if 'start' not in gait_content['from_subgait'] or 'start' in gait_content['to_subgait']:
            raise NonValidGaitContent(msg='Gait {gn} does not have valid starting subgaits'.format(gn=gait_name))

        if 'end' not in gait_content['to_subgait'] or 'end' in gait_content['from_subgait']:
            raise NonValidGaitContent(msg='Gait {gn} does not have valid ending subgaits'.format(gn=gait_name))

        from_subgaits_names = gait_content['from_subgait']
        to_subgaits_names = gait_content['to_subgait']

        subgait_names = gait_content['from_subgait'] + gait_content['to_subgait']
        subgait_names = filter(lambda name: name not in ('start', 'end'), subgait_names)

        subgaits = [cls.load_subgait(robot, gait_directory, gait_name, subgait_name, gait_version_map)
                    for subgait_name in subgait_names]

        cls._validate_trajectory_transition(subgaits, from_subgaits_names, to_subgaits_names)

        return cls(gait_name, subgaits, from_subgaits_names, to_subgaits_names)

    @staticmethod
    def load_subgait(robot, gait_directory, gait_name, subgait_name, gait_version_map):
        """Read the .subgait file and extract the data
        :returns
            if gait and subgait names are valid return populated SubGaitSelector object
        """
        if not gait_version_map.get(gait_name):
            raise GaitNameNotFound(gait_name)

        if not gait_version_map[gait_name].get(subgait_name):
            raise SubgaitNameNotFound(subgait_name)

        version = gait_version_map[gait_name][subgait_name]
        subgait_path = os.path.join(gait_directory, gait_name, subgait_name, version + '.subgait')

        if not os.path.isfile(subgait_path):
            raise FileNotFoundError(file_path=subgait_path)

        return Subgait.from_file(robot, subgait_path)

    @staticmethod
    def _validate_trajectory_transition(subgaits, from_subgait_names, to_subgait_names):
        """Compare and validate the trajectory end and start points
        :param subgaits:
            The list of subgait objects used in this gait
        :param from_subgait_names:
            the list of subgait names stated as from_subgaits in the .gait file
        :param to_subgait_names:
            the list of subgait names stated as to_subgaits in the .gait file
        """
        for from_subgait_name, to_subgait_name in zip(from_subgait_names, to_subgait_names):

            if not all(name not in ('start', 'end') for name in (from_subgait_name, to_subgait_name)):
                continue  # a start or end point can not be compared to a subgait

            from_subgait = next((subgait for subgait in subgaits if subgait.subgait_name == from_subgait_name), None)
            to_subgait = next((subgait for subgait in subgaits if subgait.subgait_name == to_subgait_name), None)

            from_subgait_joint_names = set(from_subgait.get_joint_names())
            to_subgait_joint_names = set(to_subgait.get_joint_names())

            if from_subgait_joint_names != to_subgait_joint_names:
                raise NonValidGaitContent(msg='Structure of joints does not match between subgait {} and subgait {}'
                                          .format(from_subgait_name, to_subgait_name))

            for from_subgait_joint_name, to_subgait_joint_name in zip(from_subgait_joint_names, to_subgait_joint_names):
                from_joint = from_subgait.get_joint(name=from_subgait_joint_name)
                to_joint = to_subgait.get_joint(name=to_subgait_joint_name)

                from_time, from_positions, from_velocities = from_joint.get_setpoints_unzipped()
                to_time, to_positions, to_velocities = to_joint.get_setpoints_unzipped()

                from_joint_points = [from_positions, from_velocities]
                to_joint_points = [to_positions, to_velocities]

                for from_data_point, to_data_point in zip(from_joint_points, to_joint_points):
                    if from_data_point[-1] != to_data_point[0]:
                        raise NonValidGaitContent(msg='End setpoint of joint {} in subgait {} does not match'
                                                  .format(from_subgait_joint_name, from_subgait_name))

    def __getitem__(self, name):
        """Get a subgait from the loaded subgaits"""
        return next((subgait for subgait in self.subgaits if subgait.subgait_name == name), None)
