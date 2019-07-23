// Copyright 2019 Project March.

#include <control_msgs/JointTrajectoryControllerState.h>
#include "march_safety/TrajectorySafety.h"

TrajectorySafety::TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler,
                                   std::vector<std::string> joint_names)
  : safety_handler(safety_handler), joint_names(std::move(joint_names))
{
  this->trajectory_subscriber = n->subscribe<control_msgs::JointTrajectoryControllerState>(
      std::string(TopicNames::trajectory_controller_states), 1000, &TrajectorySafety::trajectoryCallback, this);
}

void TrajectorySafety::trajectoryCallback(const control_msgs::JointTrajectoryControllerState)
{
}
