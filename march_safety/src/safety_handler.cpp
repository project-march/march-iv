// Copyright 2019 Project March.
#include "march_safety/safety_handler.h"

#include <string>

SafetyHandler::SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher,
                             ros::Publisher* gait_instruction_publisher,
                             sound_play::SoundClient& sound_client)
  : n_(n)
  , error_publisher_(error_publisher)
  , gait_instruction_publisher_(gait_instruction_publisher)
  , fatal_sound_(sound_client.waveSoundFromPkg("march_safety", "sound/fatal.wav"))
  , non_fatal_sound_(sound_client.waveSoundFromPkg("march_safety", "sound/non_fatal.wav"))
{
}

void SafetyHandler::publishErrorMessage(const std::string& message, int8_t error_type) const
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  error_msg.header.stamp = ros::Time::now();
  error_msg.error_message = message;
  error_msg.type = error_type;
  this->error_publisher_->publish(error_msg);
}

void SafetyHandler::publishStopMessage() const
{
  march_shared_resources::GaitInstruction gait_instruction_msg;
  gait_instruction_msg.header.stamp = ros::Time::now();
  gait_instruction_msg.type = march_shared_resources::GaitInstruction::STOP;
  this->gait_instruction_publisher_->publish(gait_instruction_msg);
}

void SafetyHandler::publishFatal(const std::string& message)
{
  ROS_ERROR("%s", message.c_str());

  this->publishErrorMessage(message, march_shared_resources::Error::FATAL);
  this->non_fatal_sound_.play();
}

void SafetyHandler::publishNonFatal(const std::string& message)
{
  ROS_ERROR("%s", message.c_str());

  this->publishStopMessage();
  this->publishErrorMessage(message, march_shared_resources::Error::NON_FATAL);
  this->non_fatal_sound_.play();
}
