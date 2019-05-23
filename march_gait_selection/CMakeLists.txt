cmake_minimum_required(VERSION 2.8.3)
project(march_gait_selection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        roslint
        rospy
        std_msgs
        rospy_message_converter
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        rospy
        std_msgs
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE lintfiles
        "src/*.py"
        )

roslint_python(${lintfiles})

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 add_rostest(test/launch/gait_selection.test)
 catkin_add_nosetests(test)
endif()