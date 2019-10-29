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
    FeetDistances.msg
    Gait.msg
    GaitInstruction.msg
    GaitStatus.msg
    Graph.msg
    HighVoltageNet.msg
    ImcErrorState.msg
    LowVoltageNet.msg
    PerformGait.msg
    PlayInputMaster.msg
    PowerDistributionBoardState.msg
    Setpoint.msg
    Sound.msg
    StepSize.msg
    Subgait.msg
)

add_service_files(
    FILES
    GaitInput.srv
    GaitRequest.srv
    PlayInput.srv
    StringTrigger.srv
    Trigger.srv
)

add_action_files(
    DIRECTORY action
    FILES
    Gait.action
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
    CATKIN_DEPENDS
    actionlib_msgs
    message_generation
    message_runtime
    std_msgs
    trajectory_msgs
)

