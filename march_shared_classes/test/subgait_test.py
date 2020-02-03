import unittest
import rospkg

from march_shared_classes.gait.subgait import Subgait
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from urdf_parser_py import urdf
import xacro


class SubgaitTest(unittest.TestCase):
    def setUp(self):
        resources_folder = rospkg.RosPack().get_path("march_shared_classes") + "/test/resources"
        urdf_path = resources_folder + "/march4.urdf"
        robot = urdf.Robot.from_xml_file(urdf_path)
        subgait_path = resources_folder + "/walk/left_swing/MV_walk_leftswing_v2.subgait"
        self.subgait = Subgait.from_file(robot, subgait_path)

    def test_from_file_valid_path(self):
        self.assertIsNotNone(self.subgait)

    def test_something_2(self):
        self.assertEqual(True, True)
