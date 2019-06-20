// Copyright 2019 Project March.
#include "march_safety/ErrorHandler.h"

void ErrorHandler::publishError(int8_t errorSeverity, std::string message)
{
  march_shared_resources::Error error_msg;
  error_msg.error_code = 1;  // For now a randomly chosen error code
  error_msg.error_message = message;
  error_msg.type = errorSeverity;
  if(errorSeverity == march_shared_resources::Error::FATAL){
    ROS_FATAL("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
  }else{
    ROS_ERROR("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
  }
  error_publisher->publish(error_msg);
}

ErrorHandler::ErrorHandler(ros::Publisher* errorPublisher, ros::NodeHandle* n) : n(n), error_publisher(errorPublisher)
{
}
