import os

import yaml

from march_shared_classes.exceptions.gait_exceptions import GaitNameNotFound, NonValidGaitContent, SubgaitNameNotFound
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError
from march_shared_classes.gait.subgait import Subgait


class Gait(object):
    """base class for a generated gait."""

    def __init__(self, gait_name, subgaits, graph):
        """Initializes and verifies the gait.

        :param str gait_name: Name of the gait
        :param dict subgaits: Mapping of names to subgait instances
        :param dict graph: Mapping of subgait names transitions
        """
        self._validate_gait_graph(gait_name, graph)
        self._validate_trajectory_transition(subgaits, graph)

        self.gait_name = gait_name
        self.subgaits = subgaits
        self.graph = graph

    @classmethod
    def from_file(cls, gait_name, gait_directory, robot, gait_version_map):
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

        return cls.from_dict(robot, gait_dictionary, gait_directory, gait_version_map)

    @classmethod
    def from_dict(cls, robot, gait_dictionary, gait_directory, gait_version_map):
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
        if len(from_subgaits_names) != len(to_subgaits_names):
            raise NonValidGaitContent('to_subgait and from_subgait are not of equal length')

        graph = dict(zip(from_subgaits_names, to_subgaits_names))
        cls._validate_gait_graph(gait_name, graph)

        subgait_names = set(from_subgaits_names + to_subgaits_names)
        subgaits = dict([(name, cls.load_subgait(robot, gait_directory, gait_name, name, gait_version_map))
                         for name in subgait_names if name not in ('start', 'end')])

        return cls(gait_name, subgaits, graph)

    @staticmethod
    def load_subgait(robot, gait_directory, gait_name, subgait_name, gait_version_map):
        """Read the .subgait file and extract the data.

        :returns
            if gait and subgait names are valid return populated Gait object
        """
        if gait_name not in gait_version_map:
            raise GaitNameNotFound(gait_name)
        if subgait_name not in gait_version_map[gait_name]:
            raise SubgaitNameNotFound(subgait_name)

        version = gait_version_map[gait_name][subgait_name]
        subgait_path = os.path.join(gait_directory, gait_name, subgait_name, version + '.subgait')
        if not os.path.isfile(subgait_path):
            raise FileNotFoundError(file_path=subgait_path)

        return Subgait.from_file(robot, subgait_path)

    @staticmethod
    def _validate_gait_graph(gait_name, graph):
        """Validate if the data in the gait file is valid.

        :param dict graph: Mapping of subgaits that transition to other subgaits
        """
        if 'start' not in graph.keys() or 'start' in graph.values():
            raise NonValidGaitContent(msg='Gait {gn} does not have valid starting subgaits'.format(gn=gait_name))

        if 'end' not in graph.values() or 'end' in graph.keys():
            raise NonValidGaitContent(msg='Gait {gn} does not have valid ending subgaits'.format(gn=gait_name))

    @staticmethod
    def _validate_trajectory_transition(subgaits, graph):
        """Compares and validates the trajectory end and start points.

        :param dict subgaits: Mapping of subgait names to subgait instances
        :param dict graph: Mapping of subgaits transitions
        """
        for from_subgait_name, to_subgait_name in graph.items():

            if any(name in ('start', 'end', None) for name in (from_subgait_name, to_subgait_name)):
                continue  # a start or end point cannot be compared to a subgait

            from_subgait = subgaits[from_subgait_name]
            to_subgait = subgaits[to_subgait_name]

            if not from_subgait.validate_subgait_transition(to_subgait):
                raise NonValidGaitContent(msg='End setpoint of subgait {sn} to subgait {ns} does not match'
                                          .format(sn=from_subgait.subgait_name, ns=to_subgait.subgait_name))

    def set_subgait_versions(self, robot, gait_directory, version_map):
        """Updates the given subgait versions and verifies transitions.

        :param str gait_directory: path to the gait directory
        :param dict version_map: Mapping subgait names to versions
        """
        for subgait_name in version_map.keys():
            if self.__getitem__(subgait_name) is None:
                raise SubgaitNameNotFound(subgait_name,
                                          'Subgait {0} does not exist for {1}'.format(subgait_name, self.gait_name))

        pass

    def __getitem__(self, name):
        """Returns a subgait from the loaded subgaits."""
        return self.subgaits[name]
