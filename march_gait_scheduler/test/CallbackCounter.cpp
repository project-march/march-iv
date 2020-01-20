// Copyright 2019 Project March.

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/Temperature.h>
#include <ros/ros.h>

struct CallbackCounter
{
  int count;

  CallbackCounter() : count(0)
  {
  }

  void cb_action(const control_msgs::FollowJointTrajectoryGoalConstPtr& /* goal */)
  {
    count++;
  }
};
