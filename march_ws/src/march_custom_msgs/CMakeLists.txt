cmake_minimum_required(VERSION 2.8.3)
project(march_custom_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std = c++ 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        message_generation
        message_runtime
        std_msgs
        )

add_message_files(
        FILES
        Gait.msg
        GaitInput.msg
        GaitStatus.msg
        PerformGait.msg
        PlayInput.msg
        PlayInputMaster.msg
)

add_service_files(
        FILES
        GaitInstruction.srv
        GaitRequest.srv
        Trigger.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        #INCLUDE_DIRS include
        #LIBRARIES march_custom_msgs
        CATKIN_DEPENDS
        message_generation
        message_runtime
        std_msgs
        #DEPENDS system_lib
)

include_directories(
        #include
        ${catkin_INCLUDE_DIRS}
)
