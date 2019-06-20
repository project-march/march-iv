// Copyright 2019 Project March.
#include <march_safety/InputDeviceSafety.h>

InputDeviceSafety::InputDeviceSafety(ErrorHandler* errorHandler, ros::NodeHandle* n) : errorHandler(errorHandler), n(n)
{
  int milliseconds;
  n->getParam(ros::this_node::getName() + std::string("/input_device_connection_timeout"), milliseconds);
  ROS_INFO("timeout time: %f s", ((double)milliseconds) / 1000);
  this->connection_timeout = ros::Duration(0, milliseconds * 1000000);
  this->createSubscribers();
  this->time_last_alive = ros::Time(0);
  this->time_last_send_error = ros::Time(0);
}

void InputDeviceSafety::inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg)
{
  this->time_last_alive = msg->data;
}

void InputDeviceSafety::createSubscribers()
{
  subscriber_input_device_alive = n->subscribe<std_msgs::Time>("/march/input_device/alive", 1000,
                                                               &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::checkConnection()
{
  if (time_last_alive.toSec() == 0)
  {
    ROS_DEBUG_THROTTLE(5, "No input device connected yet");
    return;
  }
  if (ros::Time::now() > time_last_alive + this->connection_timeout)
  {
    if (ros::Time::now() > time_last_send_error + ros::Duration(1))
    {
      std::ostringstream message_stream;
      message_stream << "Input Device Connection Lost. Current time: " << ros::Time::now().toSec()
                     << " and last alive message was: " << this->time_last_alive.toSec()
                     << "The difference in time is: " << ros::Time::now().toSec() - this->time_last_alive.toSec();
      std::string error_message = message_stream.str();
      errorHandler->publishError(march_shared_resources::Error::NON_FATAL, error_message);
      this->time_last_send_error = ros::Time::now();
    }
  }
}