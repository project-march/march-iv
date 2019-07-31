// Copyright 2019 Project March.
#ifndef MARCH_WS_SAFETYHANDLER_H
#define MARCH_WS_SAFETYHANDLER_H

#include <string>
#include <ros/ros.h>
#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <march_shared_resources/GaitInstruction.h>

class SafetyHandler
{
  ros::NodeHandle* n;
  ros::Publisher* error_publisher;
  ros::Publisher* sound_publisher;
  ros::Publisher* gait_instruction_publisher;
  ros::Publisher* stop_trajectory_publisher;

  trajectory_msgs::JointTrajectory empty_trajectory;

public:
  SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher, ros::Publisher* sound_publisher,
                ros::Publisher* gait_instruction_publisher, ros::Publisher* stop_trajectory_publisher);

  void publishFatal(std::string message);

  void publishNonFatal(std::string message);

  void publishErrorMessage(const std::string& message, int8_t error_type) const;

  void publishStopMessage() const;

  void publishErrorSound(int8_t error_type) const;

  void publishStopTrajectory();

  void publishStopController(const std::string& stop_controller);

  std::string getControllerStatus(const std::string& controller_name);

  void stopController(const std::string& stop_controller, float stop_trajectory_duration);
};

#endif  // MARCH_WS_SAFETYHANDLER_H
