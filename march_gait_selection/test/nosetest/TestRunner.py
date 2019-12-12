import rosunit

from . import (TestBasicGaitSelection, TestGetSubgait, TestScanDirectory,
               TestValidateGait, TestValidateSubgaitName, TestValidateSubgaitTransition,
               TestValidateTrajectoryTransition, TestValidateVersionMap, TestValidateVersionName)

PKG = 'march_gait_selection'


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_basic_gait_selection', TestBasicGaitSelection)
    rosunit.unitrun(PKG, 'test_get_subgait', TestGetSubgait)
    rosunit.unitrun(PKG, 'test_scan_directory', TestScanDirectory)
    rosunit.unitrun(PKG, 'test_validate_gait', TestValidateGait)
    rosunit.unitrun(PKG, 'test_validate_subgait_name', TestValidateSubgaitName)
    rosunit.unitrun(PKG, 'test_validate_subgait_transition', TestValidateSubgaitTransition)
    rosunit.unitrun(PKG, 'test_validate_trajectory_transition', TestValidateTrajectoryTransition)
    rosunit.unitrun(PKG, 'test_validate_version_map', TestValidateVersionMap)
    rosunit.unitrun(PKG, 'test_validate_version_name', TestValidateVersionName)
