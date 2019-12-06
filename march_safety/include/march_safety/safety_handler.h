// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_SAFETY_HANDLER_H
#define MARCH_SAFETY_SAFETY_HANDLER_H

#include <string>
#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <march_shared_resources/Error.h>
#include <march_shared_resources/GaitInstruction.h>

class SafetyHandler
{
public:
  SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher,
                ros::Publisher* gait_instruction_publisher,
                sound_play::SoundClient& sound_client);

  void publishFatal(const std::string& message);

  void publishNonFatal(const std::string& message);

  void publishErrorMessage(const std::string& message, int8_t error_type) const;

  void publishStopMessage() const;

private:
  ros::NodeHandle* n_;
  ros::Publisher* error_publisher_;
  ros::Publisher* gait_instruction_publisher_;

  sound_play::Sound fatal_sound_;
  sound_play::Sound non_fatal_sound_;
};

#endif  // MARCH_SAFETY_SAFETY_HANDLER_H
