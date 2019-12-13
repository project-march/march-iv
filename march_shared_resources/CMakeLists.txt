cmake_minimum_required(VERSION 2.8.3)
project(march_shared_resources)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    message_runtime
    std_msgs
    trajectory_msgs
)

add_message_files(
    FILES
    AfterLimitJointCommand.msg
    Error.msg
    GaitInstruction.msg
    GaitInstructionResponse.msg
    Graph.msg
    HighVoltageNet.msg
    ImcErrorState.msg
    LowVoltageNet.msg
    PowerDistributionBoardState.msg
    Setpoint.msg
    Sound.msg
    Subgait.msg
)

add_service_files(
    FILES
    StringTrigger.srv
    Trigger.srv
)

add_action_files(
    DIRECTORY action
    FILES
    Gait.action
    GaitName.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    message_generation
    message_runtime
    std_msgs
    trajectory_msgs
)

