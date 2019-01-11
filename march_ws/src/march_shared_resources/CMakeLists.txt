cmake_minimum_required(VERSION 2.8.3)
project(march_shared_resources)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std = c++ 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        actionlib_msgs
        message_generation
        message_runtime
        std_msgs
        )

add_message_files(
        FILES
        Gait.msg
        GaitStatus.msg
        PerformGait.msg
        PlayInputMaster.msg
        StepSize.msg
)

add_service_files(
        FILES
        GaitInput.srv
        GaitInstruction.srv
        GaitRequest.srv
        PlayInput.srv
        Trigger.srv
)

add_action_files(
        DIRECTORY action
        FILES MoveToPose.action
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        #LIBRARIES march_shared_resources
        CATKIN_DEPENDS
        actionlib_msgs
        message_generation
        message_runtime
        std_msgs
        #DEPENDS system_lib
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)


include_directories(
        #include
        ${catkin_INCLUDE_DIRS}
)

