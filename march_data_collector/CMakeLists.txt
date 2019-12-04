cmake_minimum_required(VERSION 2.8.3)
project(march_data_collector)

find_package(catkin REQUIRED COMPONENTS
    control_msgs
    march_shared_resources
    rospy
    sensor_msgs
    std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    control_msgs
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
