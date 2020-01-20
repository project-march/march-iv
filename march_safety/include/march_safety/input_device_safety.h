// Copyright 2019 Project March
#ifndef MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
#define MARCH_SAFETY_INPUT_DEVICE_SAFETY_H

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sstream>

#include <march_shared_resources/Error.h>

#include "march_safety/safety_type.h"
#include "march_safety/safety_handler.h"

class InputDeviceSafety : public SafetyType
{
public:
  InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler);

  void update() override;

private:
  void inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg);

  SafetyHandler* safety_handler_;
  ros::Duration connection_timeout_;
  ros::Time time_last_alive_;
  ros::Time time_last_send_error_;
  ros::Subscriber subscriber_input_device_alive_;
  int send_errors_interval_;
};

#endif  // MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
