import unittest
import rospkg

from march_shared_classes.gait.subgait import Subgait
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from urdf_parser_py import urdf
import xacro


class SubgaitTest(unittest.TestCase):
    def setUp(self):
        self.resources_folder = rospkg.RosPack().get_path("march_shared_classes") + "/test/resources"
        self.robot = urdf.Robot.from_xml_file(self.resources_folder + "/march4.urdf")
        self.subgait_path = self.resources_folder + "/walk/left_swing/MV_walk_leftswing_v2.subgait"
        self.subgait = Subgait.from_file(self.robot, self.subgait_path)

    # Subgait.from_file tests
    def test_from_file_valid_path(self):
        self.assertIsInstance(self.subgait, Subgait)

    def test_from_file_invalid_path(self):
        self.assertIsNone(Subgait.from_file(self.robot, self.resources_folder + "/MV_walk_leftswing_v2.subgait"))

    def test_from_file_none_path(self):
        self.assertIsNone(Subgait.from_file(self.robot, None))

    def test_from_file_no_robot(self):
        self.assertIsNone(Subgait.from_file(None, self.subgait_path))
