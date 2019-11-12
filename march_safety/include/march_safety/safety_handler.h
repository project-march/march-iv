// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_SAFETY_HANDLER_H
#define MARCH_SAFETY_SAFETY_HANDLER_H

#include <string>
#include <ros/ros.h>
#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>
#include <march_shared_resources/GaitInstruction.h>

class SafetyHandler
{
public:
  SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher, ros::Publisher* sound_publisher,
                ros::Publisher* gait_instruction_publisher);

  void publishFatal(std::string message);

  void publishNonFatal(std::string message);

  void publishErrorMessage(const std::string& message, int8_t error_type) const;

  void publishStopMessage() const;

  void publishErrorSound(int8_t error_type) const;

private:
  ros::NodeHandle* n_;
  ros::Publisher* error_publisher_;
  ros::Publisher* sound_publisher_;
  ros::Publisher* gait_instruction_publisher_;
};

#endif  // MARCH_SAFETY_SAFETY_HANDLER_H
