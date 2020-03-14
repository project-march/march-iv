// Copyright 2019 Project March.
#include "march_safety/input_device_safety.h"

#include <string>

InputDeviceSafety::InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler)
  : time_last_alive_(ros::Time(0)), is_connected_(false)
{
  int milliseconds;
  ros::param::get("~input_device_connection_timeout", milliseconds);
  this->connection_timeout_ = ros::Duration(milliseconds / 1000.0);
  this->safety_handler_ = safety_handler;
  this->subscriber_input_device_alive_ =
      n->subscribe<std_msgs::Time>("/march/input_device/alive", 10, &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg)
{
  this->time_last_alive_ = msg->data;
}

void InputDeviceSafety::update(const ros::Time& now)
{
  if (this->time_last_alive_.isZero())
  {
    ROS_INFO_THROTTLE(5, "No input device connected yet");
    return;
  }

  const bool timed_out = now > this->time_last_alive_ + this->connection_timeout_;
  // Check if no alive msg has been received for the timeout duration.
  if (this->is_connected_ && timed_out)
  {
    this->is_connected_ = false;
    std::ostringstream message_stream;
    message_stream << "Input Device Connection Lost. Current time is " << now.toSec() << " and last alive message was "
                   << this->time_last_alive_.toSec()
                   << ". The difference in time is: " << now.toSec() - this->time_last_alive_.toSec();
    this->safety_handler_->publishNonFatal(message_stream.str());
  }
  else if (!this->is_connected_ && !timed_out)
  {
    this->is_connected_ = true;
    ROS_INFO("Input device reconnected");
  }

  if (!this->is_connected_)
  {
    ROS_INFO_DELAYED_THROTTLE(5.0, "No input device connected");
  }

  // Check if the alive msg is not timestamped with a future time.
  // This can happen when one node is using sim_time and others aren't.
  // Add small margin to take the stamp offset between board and PC into account
  if (now + ros::Duration(0.5) < this->time_last_alive_)
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "Input Device Connection message is from the future. Current time: "
                                      << now.toSec()
                                      << " and last alive message was: " << this->time_last_alive_.toSec()
                                      << "The difference in time is: " << this->time_last_alive_.toSec() - now.toSec());
  }
}
