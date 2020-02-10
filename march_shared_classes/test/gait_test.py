import unittest

import rospkg
from urdf_parser_py import urdf
import yaml

from march_shared_classes.exceptions.gait_exceptions import GaitNameNotFound, NonValidGaitContent, SubgaitNameNotFound
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError
from march_shared_classes.gait.gait import Gait
from march_shared_classes.gait.subgait import Subgait


class GaitTest(unittest.TestCase):
    def setUp(self):
        self.gait_name = 'walk'
        self.resources_folder = rospkg.RosPack().get_path('march_shared_classes') + '/test/resources'
        self.robot = urdf.Robot.from_xml_file(self.resources_folder + '/march4.urdf')

        self.default_yaml = self.resources_folder + '/default.yaml'
        with open(self.default_yaml, 'r') as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)
        self.gait_version_map = default_config['gaits']

        self.gait = Gait.from_file(self.gait_name, self.resources_folder, self.robot, self.gait_version_map)

        self.from_subgait = ['start', 'right_open', 'left_swing', 'right_swing', 'left_close', 'start', 'right_open',
                             'left_swing', 'right_close']
        self.to_subgait = ['right_open', 'left_swing', 'right_swing', 'left_close', 'end', 'right_open', 'left_swing',
                           'right_close', 'end']

    # Gait.from_file tests
    def test_from_file_valid_path(self):
        self.assertIsInstance(self.gait, Gait)

    def test_from_file_invalid_path(self):
        with self.assertRaises(FileNotFoundError):
            Gait.from_file(self.gait_name, self.resources_folder + '/gaits', self.robot, self.gait_version_map)

    # __init__ (_validate_gait_file) tests
    def test_init_invalid_gait_file_unequal_length(self):
        from_subgait = ['start', 'right_open', 'left_swing', 'right_swing', 'left_close', 'start', 'right_open',
                        'left_swing', 'right_close', 'left_close']
        with self.assertRaises(NonValidGaitContent):
            Gait(self.gait_name, self.gait.subgaits, from_subgait, self.to_subgait)

    def test_init_invalid_gait_file_no_start(self):
        from_subgait = ['right_open', 'right_open', 'left_swing', 'right_swing', 'left_close', 'start', 'right_open',
                        'left_swing', 'right_close']
        with self.assertRaises(NonValidGaitContent):
            Gait(self.gait_name, self.gait.subgaits, from_subgait, self.to_subgait)

    def test_init_invalid_gait_file_start_in_to_subgait(self):
        to_subgait = ['start', 'left_swing', 'right_swing', 'left_close', 'end', 'right_open', 'left_swing',
                      'right_close', 'end']
        with self.assertRaises(NonValidGaitContent):
            Gait(self.gait_name, self.gait.subgaits, self.from_subgait, to_subgait)

    def test_init_invalid_gait_file_no_end(self):
        to_subgait = ['right_open', 'left_swing', 'right_swing', 'left_close', 'end', 'right_open', 'left_swing',
                      'right_close', 'left_close']
        with self.assertRaises(NonValidGaitContent):
            Gait(self.gait_name, self.gait.subgaits, self.from_subgait, to_subgait)

    def test_init_invalid_gait_file_end_in_from_subgait(self):
        from_subgait = ['start', 'right_open', 'left_swing', 'right_swing', 'left_close', 'start', 'right_open',
                        'left_swing', 'end']
        with self.assertRaises(NonValidGaitContent):
            Gait(self.gait_name, self.gait.subgaits, from_subgait, self.to_subgait)

    # __init__ (_validate_trajectory_transition) test
    def test_init_invalid_joint_trjaectory_transition(self):
        self.gait.subgaits[0].joints[0].setpoints[-1].position = 124
        with self.assertRaises(NonValidGaitContent):
            Gait(self.gait_name, self.gait.subgaits, self.from_subgait, self.to_subgait)

    # load_subgait tests
    def test_load_existing_subgait(self):
        subgait = Gait.load_subgait(self.robot, self.resources_folder,
                                    self.gait_name, 'left_swing', self.gait_version_map)
        self.assertIsInstance(subgait, Subgait)

    def test_load_subgait_unexisting_gait_error(self):
        with self.assertRaises(GaitNameNotFound):
            Gait.load_subgait(self.robot, self.resources_folder, 'walk_small', 'left_swing', self.gait_version_map)

    def test_load_subgait_unexisting_subgait_error(self):
        with self.assertRaises(SubgaitNameNotFound):
            Gait.load_subgait(self.robot, self.resources_folder, self.gait_name, 'left_open', self.gait_version_map)

    def test_load_subgait_unexisting_version_error(self):
        self.gait_version_map['walk']['right_open'] = 'MV_walk_rightopen_non_existing_banana_version'

        with self.assertRaises(FileNotFoundError):
            Gait.load_subgait(self.robot, self.resources_folder, self.gait_name, 'right_open', self.gait_version_map)
