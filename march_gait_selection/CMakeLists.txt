cmake_minimum_required(VERSION 2.8.3)
project(march_gait_selection)


find_package(
        catkin REQUIRED COMPONENTS
        march_shared_classes
        march_shared_resources
        rospy
        std_msgs
        )

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        march_shared_classes
        march_shared_resources
        rospy
        std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

catkin_install_python(PROGRAMS test/run_tests
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY test/launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
        enable_testing()
        find_package(catkin COMPONENTS rostest)
        add_rostest(test/launch/testing_gait_selection.test)
endif()
