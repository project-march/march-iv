import unittest
import rospkg

from march_shared_classes.gait.subgait import Subgait
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from urdf_parser_py import urdf
import xacro


class SubgaitTest(unittest.TestCase):
    def setUp(self):
        self.gait_name = 'walk'
        self.subgait_name = 'left_swing'
        self.version = 'MV_walk_leftswing_v2'
        self.resources_folder = rospkg.RosPack().get_path('march_shared_classes') + '/test/resources'
        self.robot = urdf.Robot.from_xml_file(self.resources_folder + '/march4.urdf')
        self.subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait=self.subgait_name,
                                                                              version=self.version)
        self.subgait = Subgait.from_file(self.robot, self.subgait_path)
        self.subgait_msg = self.subgait.to_subgait_msg()

    # Subgait.from_file tests
    def test_from_file_valid_path(self):
        self.assertIsInstance(self.subgait, Subgait)

    def test_from_file_invalid_path(self):
        self.assertIsNone(Subgait.from_file(self.robot, self.resources_folder + '/MV_walk_leftswing_v2.subgait'))

    def test_from_file_none_path(self):
        self.assertIsNone(Subgait.from_file(self.robot, None))

    def test_from_file_no_robot(self):
        self.assertIsNone(Subgait.from_file(None, self.subgait_path))

    # to_subgait_msg tests
    def test_to_subgait_msg_name(self):
        self.assertEqual(self.subgait_msg.name, self.subgait_name)

    def test_to_subgait_msg_gait_type(self):
        self.assertEqual(self.subgait_msg.gait_type, 'walk_like')

    def test_to_subgait_msg_gait_trajectory_joint_names(self):
        self.assertEqual(len(self.subgait_msg.trajectory.joint_names), 8)

    def test_to_subgait_msg_gait_trajectory_points(self):
        self.assertEqual(len(self.subgait_msg.trajectory.points), 9)

    def test_to_subgait_msg_gait_trajectory_points_positions(self):
        self.assertEqual(len(self.subgait_msg.trajectory.points[1].positions), 8)

    def test_to_subgait_msg_setpoints(self):
        self.assertEqual(len(self.subgait_msg.setpoints), 9)

    def test_to_subgait_msg_setpoints_joint_names(self):
        self.assertEqual(len(self.subgait_msg.setpoints[1].joint_names), 8)

    def test_to_subgait_msg_description(self):
        self.assertEqual(self.subgait_msg.description, 'The MIV walking gait, but with a somewhat faster swing.')

    def test_to_subgait_msg_gait_duration(self):
        self.assertEqual(self.subgait_msg.duration.secs, 1)
        self.assertEqual(self.subgait_msg.duration.nsecs, 100000000)

    # validate_subgait_transition tests
    def test_valid_subgait_transition(self):
        other_subgait_name = 'right_close'
        other_version = 'MV_walk_rightclose_v2'
        other_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=self.gait_name,
                                                                               subgait=other_subgait_name,
                                                                               version=other_version)
        other_subgait = Subgait.from_file(self.robot, other_subgait_path)
        self.assertTrue(self.subgait.validate_subgait_transition(other_subgait))








