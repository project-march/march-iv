// Copyright 2019 Project March.
#ifndef MARCH_WS_SAFETYHANDLER_H
#define MARCH_WS_SAFETYHANDLER_H

#include <string>
#include <ros/ros.h>
#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>
#include "trajectory_msgs/JointTrajectory.h"

class SafetyHandler
{
  ros::NodeHandle* n;
  ros::Publisher* error_publisher;
  ros::Publisher* sound_publisher;
  ros::Publisher* enter_hold_position_publisher;

  trajectory_msgs::JointTrajectory empty_trajectory;

public:
  SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher, ros::Publisher* sound_publisher);

  void publishFatal(std::string message);

  void publishNonFatal(std::string message);

  void publishErrorMessage(const std::string& message, int8_t error_type) const;

  void publishErrorSound(int8_t error_type) const;

  void publishEmptyTrajectory();
};

#endif  // MARCH_WS_SAFETYHANDLER_H
