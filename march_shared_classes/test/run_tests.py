#!/usr/bin/env python
import rosunit

from .limits_test import LimitsTest
from .setpoint_test import SetpointTest
from .joint_trajectory_test import JointTrajectoryTest

PKG = 'march_shared_classes'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'limits_test', LimitsTest)
    rosunit.unitrun(PKG, 'setpoint_test', SetpointTest)
    rosunit.unitrun(PKG, 'joint_trajectory_test', JointTrajectoryTest)
