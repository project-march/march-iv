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
        roslint
        trajectory_msgs
        )

add_message_files(
        FILES
        Error.msg
        Gait.msg
        GaitInstruction.msg
        GaitStatus.msg
        Graph.msg
        PerformGait.msg
        PlayInputMaster.msg
        PowerDistributionBoardState.msg
        HighVoltageNet.msg
        LowVoltageNet.msg
        StepSize.msg
        Setpoint.msg
        Sound.msg
        Subgait.msg
        ImcErrorState.msg
        AfterLimitCommand.msg
)

add_service_files(
        FILES
        GaitInput.srv
        GaitRequest.srv
        PlayInput.srv
        Trigger.srv
        StringTrigger.srv
)

add_action_files(
        DIRECTORY action
        FILES Gait.action
        GaitName.action
        MoveToGait.action
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs
        trajectory_msgs
)

catkin_package(
        INCLUDE_DIRS include
        #LIBRARIES march_shared_resources
        CATKIN_DEPENDS
        actionlib_msgs
        message_generation
        message_runtime
        std_msgs
        trajectory_msgs
        #DEPENDS system_lib
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)


include_directories(
        #include
        ${catkin_INCLUDE_DIRS}
)

