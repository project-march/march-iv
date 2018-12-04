// Copyright 2018 Project March.

#include <chrono>
#include <thread>

#include "MovementAPI.h"
#include "march_custom_msgs/GaitRequest.h"
#include "ros/ros.h"

bool MovementAPI::request_gait_file(march_custom_msgs::GaitRequest::Request& request,
                                    march_custom_msgs::GaitRequest::Response& response)
{
  ROS_INFO("asdasd");
  return true;
}