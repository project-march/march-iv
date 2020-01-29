cmake_minimum_required(VERSION 2.8.3)
project(march_gait_selection)

find_package(catkin REQUIRED COMPONENTS
    march_description
    march_shared_classes
    march_shared_resources
    std_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    march_description
    march_shared_classes
    march_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node test/run_tests
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch test/launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/launch/testing_gait_selection.test)
endif()
