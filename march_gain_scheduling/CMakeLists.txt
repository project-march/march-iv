cmake_minimum_required(VERSION 2.8.3)
project(march_gain_scheduling)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    roslint
    std_msgs
    rospy
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_resources
    std_msgs
    rospy
)

file(GLOB_RECURSE lintfiles "src/*.py")
roslint_python(${lintfiles})

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
