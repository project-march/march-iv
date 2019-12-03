cmake_minimum_required(VERSION 2.8.3)
project(march_gain_scheduling)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    march_shared_resources
    rospy
    std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_resources
    rospy
    std_msgs
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/one_step_linear_interpolation_test.py)
endif()

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
