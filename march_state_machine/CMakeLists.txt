cmake_minimum_required(VERSION 2.8.3)
project(march_state_machine)

find_package(catkin REQUIRED COMPONENTS
    controller_manager_msgs
    march_shared_resources
    std_msgs
    std_srvs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    controller_manager_msgs
    march_shared_resources
    std_msgs
)

install(DIRECTORY launch sounds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/launch/smach.test)
endif()
