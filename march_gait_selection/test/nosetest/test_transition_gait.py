
import unittest

import rospy
import rosunit

from march_gait_selection.PerformGaitAction import PerformGaitAction
from march_gait_selection.GaitSelection import GaitSelection

NODE_NAME = 'gait_selection'
GAIT_FILES_MAP_NAME = 'march_gait_files'
GAIT_DIRECTORY_NAME = 'gait'

PKG = 'march_gait_selection'
NAME = 'transition_gait'


class TransitionGait(unittest.TestCase):
    def setUp(self):
        rospy.init_node(NODE_NAME)
        gait_package = rospy.get_param('~gait_package', GAIT_FILES_MAP_NAME)
        gait_directory = rospy.get_param('~gait_directory', GAIT_DIRECTORY_NAME)

        gait_selection = GaitSelection(gait_package, gait_directory)
        self.perform_gait_action = PerformGaitAction(gait_selection)
        print('done')

    def test_if_works(self):
        self.assertTrue(False, msg='did not work')


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TransitionGait)
