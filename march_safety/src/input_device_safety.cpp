// Copyright 2019 Project March.
#include "march_safety/input_device_safety.h"

#include <string>

InputDeviceSafety::InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler)
{
  int milliseconds;
  n->getParam(ros::this_node::getName() + std::string("/input_device_connection_timeout"), milliseconds);
  double send_errors_interval_param;
  n->getParam(ros::this_node::getName() + std::string("/send_errors_interval"), send_errors_interval_param);
  this->send_errors_interval_ = send_errors_interval_param;
  this->connection_timeout_ = ros::Duration(milliseconds / 1000.0);
  this->safety_handler_ = safety_handler;
  this->time_last_alive_ = ros::Time(0);
  this->time_last_send_error_ = ros::Time(0);
  this->subscriber_input_device_alive_ = n->subscribe<std_msgs::Time>(
      "/march/input_device/alive", 1000, &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg)
{
  this->time_last_alive_ = msg->data;
  if (this->time_last_alive_.toSec() == 0)
  {
    ROS_INFO("Safety node started listening to the input device alive topic");
  }
}

void InputDeviceSafety::update()
{
  if (this->time_last_alive_.toSec() == 0)
  {
    ROS_INFO_THROTTLE(5, "No input device connected yet");
    return;
  }
  // send at most an error every second
  if (ros::Time::now() > this->time_last_send_error_ + ros::Duration(this->send_errors_interval_ / 1000.0))
  {
    // Check if there is no alive msg receive for the timeout duration.
    if (ros::Time::now() > this->time_last_alive_ + this->connection_timeout_)
    {
      std::ostringstream message_stream;
      message_stream << "Input Device Connection Lost. Current time: " << ros::Time::now().toSec()
                     << " and last alive message was: " << this->time_last_alive_.toSec()
                     << "The difference in time is: " << ros::Time::now().toSec() - this->time_last_alive_.toSec();
      std::string error_message = message_stream.str();
      this->safety_handler_->publishNonFatal(error_message);
      this->time_last_send_error_ = ros::Time::now();
    }
    // Check if the alive msg is not timestamped with a future time.
    // This can happen when one node is using sim_tim and others aren't.
    // Add small margin to take the stamp offset between board and PC into account
    if (ros::Time::now() + ros::Duration(0.5) < this->time_last_alive_)
    {
      std::ostringstream message_stream;
      message_stream << "Input Device Connection message is from the future. Current time: " << ros::Time::now().toSec()
                     << " and last alive message was: " << this->time_last_alive_.toSec()
                     << "The difference in time is: " << this->time_last_alive_.toSec() - ros::Time::now().toSec();
      std::string error_message = message_stream.str();
      this->safety_handler_->publishNonFatal(error_message);
      this->time_last_send_error_ = ros::Time::now();
    }
  }
}
