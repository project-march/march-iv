// Copyright 2018 Project March.

#ifndef PROJECT_MOVEMENT_API_H
#define PROJECT_MOVEMENT_API_H

#include "march_custom_msgs/GaitRequest.h"

class MovementAPI
{
public:
  static bool request_gait_file(march_custom_msgs::GaitRequest::Request& request,
                                march_custom_msgs::GaitRequest::Response& response);
};

#endif  // PROJECT_MOVEMENT_API_H
