// Copyright 2019 Project March.

#include "march_safety/TrajectorySafety.h"

TrajectorySafety::TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler,
                                   std::vector<std::string> joint_names)
  : safety_handler(safety_handler), joint_names(std::move(joint_names))
{
  n->getParam(ros::this_node::getName() + std::string("/controller_name"), this->controller_name);
  controller_path = "/" + controller_name;

  bool stop_trajectory_duration_status =
      n->getParam(controller_path + "/stop_trajectory_duration", this->stop_trajectory_duration);

  // check whether the controller is available, through the use of an action client

  // wait for action server to come up, if so the controller is available
  if ((this->safety_handler->getControllerStatus(controller_name)) == "unknown")
  {
    ROS_FATAL("either the controller is unavailable, or the controller path is incorrect");
    std::runtime_error("controller safety is unavailable, due to controller unavailable or wrong controller path");
  }

  this->trajectory_subscriber = n->subscribe<control_msgs::JointTrajectoryControllerState>(
      controller_path + "/state", 1000, &TrajectorySafety::trajectoryCallback, this);

  if (this->stop_trajectory_duration < 0)
  {
    ROS_WARN("stop_trajectory_duration has been set below zero, stopping will be immediate ");
  }
  if (!stop_trajectory_duration_status)
  {
    ROS_WARN("stop_trajectory_duration has not been explicitly stated in the controller yaml. Using default of 0.0");
    this->stop_trajectory_duration = 0;
  }
  for (auto it = this->joint_names.begin(); it != this->joint_names.end(); it++)
  {
    double joint_tolerance;
    if (n->getParam(controller_path + "/constraints/" + *it + "/trajectory", joint_tolerance))
    {
      this->trajectory_tolerances[*it] = joint_tolerance;
    }
    else
    {
      ROS_FATAL("tolerance of joint %s cannot be found. Make sure it is indicated in the controller yaml",
                getJointName(it).c_str());
    }
  }
}

void TrajectorySafety::trajectoryCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
  if (joint_names.size() != msg->joint_names.size())
  {
    ROS_FATAL("the size of joint_names and msg size is not the same");
    std::runtime_error("joint_names size is not equal to message size");
  }

  for (int i = 0; i < joint_names.size(); i++)
  {
    std::string joint_name = joint_names[i];
    this->position_errors[joint_name] = msg->error.positions[i];
  }
  toleranceCheck();
}

void TrajectorySafety::toleranceCheck()
{
  for (auto& trajectory_tolerance : this->trajectory_tolerances)
  {
    std::string joint_name = trajectory_tolerance.first;

    if (trajectory_tolerance.second < std::abs(position_errors.find(joint_name)->second))
    {
      ROS_WARN("tolerances of joint %s have been passed. Stopping movement", trajectory_tolerance.first.c_str());
      safety_handler->stopController(controller_name, stop_trajectory_duration);
    }
  }
}

std::string TrajectorySafety::getJointName(std::vector<std::string>::iterator it)
{
  std::ostringstream message_stream;
  message_stream << *it;
  return message_stream.str();
}
