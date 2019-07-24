// Copyright 2019 Project March.

#include <control_msgs/JointTrajectoryControllerState.h>
#include "march_safety/TrajectorySafety.h"

TrajectorySafety::TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler,
                                   std::vector<std::string> joint_names)
  : safety_handler(safety_handler), joint_names(std::move(joint_names))
{
  this->trajectory_subscriber = n->subscribe<control_msgs::JointTrajectoryControllerState>(
      std::string(TopicNames::trajectory_controller) + "/state", 1000, &TrajectorySafety::trajectoryCallback, this);

  for (auto it = this->joint_names.begin(); it != this->joint_names.end(); it++)
  {
    double joint_tolerance;
    if (n->getParam(std::string(TopicNames::trajectory_controller) + "/constraints/" + *it + "/trajectory",
                    joint_tolerance))
    {
      this->trajectory_tolerances[*it] = joint_tolerance;
    }
    else
      ROS_WARN("tolerance of joint %c cannot be found", *it->c_str());
  }
}

void TrajectorySafety::trajectoryCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
  int counter = 0;
  for (auto it = msg->joint_names.begin(); it != msg->joint_names.end(); ++it)
  {
    this->position_errors[*it] = msg->error.positions[counter];
    ROS_INFO("error: %f", position_errors[*it]);
    counter++;
  }
  toleranceCheck();
}

void TrajectorySafety::toleranceCheck()
{
  for (auto & trajectory_tolerance : this->trajectory_tolerances)
  {
    std::string joint_name = trajectory_tolerance.first;

    if (trajectory_tolerance.second < position_errors.find(joint_name)->second)
    {
        ROS_WARN("tolerances have been passed!");
    }
  }
}

std::string TrajectorySafety::getJointName(std::vector<std::string>::iterator it)
{
  std::ostringstream message_stream;
  message_stream << *it;
  return message_stream.str();
}