import os

import yaml

from march_shared_classes.exceptions.gait_exceptions import GaitNameNotFound, NonValidGaitContent, SubgaitNameNotFound
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError
from march_shared_classes.gait.subgait import Subgait


class Gait(object):
    """base class for a generated gait."""

    def __init__(self, gait_name, subgaits, from_subgaits_names, to_subgaits_names, *args):
        self._validate_gait_file(gait_name, from_subgaits_names, to_subgaits_names)
        self._validate_trajectory_transition(subgaits, from_subgaits_names, to_subgaits_names)

        self.gait_name = gait_name
        self.subgaits = subgaits
        self.from_subgaits_names = from_subgaits_names
        self.to_subgaits_names = to_subgaits_names

    @classmethod
    def from_file(cls, gait_name, gait_directory, robot, gait_version_map, *args):
        """Extract the data from the .gait file.

        :param gait_name:
            name of the gait to unpack
        :param gait_directory:
            path of the directory where the .gait file is located
        :param robot:
            the robot corresponding to the given .gait file
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits
        """
        gait_folder = gait_name
        gait_path = os.path.join(gait_directory, gait_folder, gait_name + '.gait')
        if not os.path.isfile(gait_path):
            raise FileNotFoundError(gait_path)

        with open(gait_path, 'r') as gait_file:
            gait_dictionary = yaml.load(gait_file, Loader=yaml.SafeLoader)

        return cls.from_dict(robot, gait_dictionary, gait_directory, gait_version_map, *args)

    @classmethod
    def from_dict(cls, robot, gait_dictionary, gait_directory, gait_version_map, *args):
        """Create a new gait object using the .gait and .subgait files.

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

        from_subgaits_names = gait_content['from_subgait']
        to_subgaits_names = gait_content['to_subgait']

        subgait_names = gait_content['from_subgait'] + gait_content['to_subgait']
        subgait_names = filter(lambda name: name not in ('start', 'end'), subgait_names)

        subgaits = [cls.load_subgait(robot, gait_directory, gait_name, subgait_name, gait_version_map)
                    for subgait_name in subgait_names]

        return cls(gait_name, subgaits, from_subgaits_names, to_subgaits_names, *args)

    @staticmethod
    def load_subgait(robot, gait_directory, gait_name, subgait_name, gait_version_map):
        """Read the .subgait file and extract the data.

        :returns
            if gait and subgait names are valid return populated Gait object
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
    def _validate_gait_file(gait_name, from_subgait_names, to_subgait_names):
        """Validate if the data in the gait file is valid.

        :param from_subgait_names:
            the list of subgait names stated as from_subgaits in the .gait file
        :param to_subgait_names:
            the list of subgait names stated as to_subgaits in the .gait file
        """
        if len(from_subgait_names) != len(to_subgait_names):
            raise NonValidGaitContent(msg='Gait {gn} does not have equal amount of subgaits'.format(gn=gait_name))

        if 'start' not in from_subgait_names or 'start' in to_subgait_names:
            raise NonValidGaitContent(msg='Gait {gn} does not have valid starting subgaits'.format(gn=gait_name))

        if 'end' not in to_subgait_names or 'end' in from_subgait_names:
            raise NonValidGaitContent(msg='Gait {gn} does not have valid ending subgaits'.format(gn=gait_name))

    @staticmethod
    def _validate_trajectory_transition(subgaits, from_subgait_names, to_subgait_names):
        """Compare and validate the trajectory end and start points.

        :param subgaits:
            The list of subgait objects used in this gait
        :param from_subgait_names:
            the list of subgait names stated as from_subgaits in the .gait file
        :param to_subgait_names:
            the list of subgait names stated as to_subgaits in the .gait file
        """
        for from_subgait_name, to_subgait_name in zip(from_subgait_names, to_subgait_names):

            if not all(name not in ('start', 'end', None) for name in (from_subgait_name, to_subgait_name)):
                continue  # a start or end point can not be compared to a subgait

            from_subgait = next((subgait for subgait in subgaits if subgait.subgait_name == from_subgait_name), None)
            to_subgait = next((subgait for subgait in subgaits if subgait.subgait_name == to_subgait_name), None)

            if not from_subgait.validate_subgait_transition(to_subgait):
                raise NonValidGaitContent(msg='End setpoint of subgait {sn} to subgait {ns} does not match'
                                          .format(sn=from_subgait.subgait_name, ns=to_subgait.subgait_name))

    def __getitem__(self, name):
        """Get a subgait from the loaded subgaits."""
        return next((subgait for subgait in self.subgaits if subgait.subgait_name == name), None)
