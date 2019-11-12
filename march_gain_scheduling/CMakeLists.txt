cmake_minimum_required(VERSION 2.8.3)
project(march_gain_scheduling)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    march_shared_resources
    roslint
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

file(GLOB_RECURSE lintfiles "src/*.py")
roslint_python(${lintfiles})

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
