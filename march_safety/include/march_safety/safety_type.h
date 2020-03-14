// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_SAFETY_TYPE_H
#define MARCH_SAFETY_SAFETY_TYPE_H
#include <ros/ros.h>

class SafetyType
{
public:
  virtual void update(const ros::Time& now) = 0;
};

#endif  // MARCH_SAFETY_SAFETY_TYPE_H
