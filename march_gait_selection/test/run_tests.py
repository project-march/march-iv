import rosunit

from .test_gait_transition import TestTransitionTrajectory


PKG = 'march_gait_selection'


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_transition', TestTransitionTrajectory)
