// Copyright 2019 Project March.time
#ifndef PROJECT_INPUTDEVICESAFETY_H
#define PROJECT_INPUTDEVICESAFETY_H

#include "ros/ros.h"
#include "std_msgs/Time.h"
#include "ErrorHandler.h"
#include <sstream>

#include <march_shared_resources/Error.h>

class InputDeviceSafety
{
  ErrorHandler* errorHandler;
  ros::NodeHandle* n;
  ros::Duration connection_timeout;
  ros::Time time_last_alive;
  ros::Time time_last_send_error;
  ros::Subscriber subscriber_input_device_alive;

  void inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg);

  march_shared_resources::Error createErrorMessage();

  void createSubscribers();

public:
  InputDeviceSafety(ErrorHandler* errorHandler, ros::NodeHandle* n);
  void checkConnection();
};

#endif  // PROJECT_INPUTDEVICESAFETY_H
