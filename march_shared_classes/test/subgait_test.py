import unittest
import rosunit

from march_shared_classes.gait.subgait import Subgait
from march_shared_classes.gait.joint_trajectory import JointTrajectory


class SubgaitTest(unittest.TestCase):
    def test_something(self):
        self.assertEqual(True, True)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_subgait', SubgaitTest)
