// Copyright 2019 Project March.
#ifndef MARCH_WS_ERRORHANDLER_H
#define MARCH_WS_ERRORHANDLER_H

#include <string>
#include <ros/ros.h>
#include <march_shared_resources/Error.h>

class ErrorHandler
{
  ros::NodeHandle* n;
  ros::Publisher* error_publisher;

public:
  ErrorHandler(ros::Publisher* errorPublisher, ros::NodeHandle* n);

  void publishError(int8_t errorSeverity, std::string message);
};

#endif  // MARCH_WS_ERRORHANDLER_H
