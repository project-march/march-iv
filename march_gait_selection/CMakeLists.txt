cmake_minimum_required(VERSION 2.8.3)
project(march_gait_selection)

find_package(catkin REQUIRED COMPONENTS
    march_shared_classes
    march_shared_resources
    std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_classes
    march_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/run_tests.py)
endif()
