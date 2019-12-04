import rosunit

import TestGetSubgait
import TestValidateSubgaitName
import TestScanDirectory
import TestBasicGaitSelection
import TestValidateSubgaitTransition

PKG = "march_gait_selection"


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_basic_gait_selection', TestBasicGaitSelection)
    rosunit.unitrun(PKG, 'test_get_subgait', TestGetSubgait)
    rosunit.unitrun(PKG, 'test_scan_directory', TestScanDirectory)
    rosunit.unitrun(PKG, 'test_validate_subgait_name', TestValidateSubgaitName)
    rosunit.unitrun(PKG, 'test_validate_subgait_transition', TestValidateSubgaitTransition)

