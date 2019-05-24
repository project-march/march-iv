// Copyright 2019 Project March.
#ifndef MARCH_GAIT_SCHEDULER_SCHEDULER_H
#define MARCH_GAIT_SCHEDULER_SCHEDULER_H

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <march_shared_resources/GaitGoal.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

class Scheduler
{
  const march_shared_resources::GaitGoal* lastGaitGoal = nullptr;
  ros::Time startLastGait;

  ros::Time getStartTime(ros::Duration offset);
  bool lastScheduledGaitNotStarted();
  static trajectory_msgs::JointTrajectory setStartTimeGait(trajectory_msgs::JointTrajectory trajectory, ros::Time time);

public:
  /**
   * A gait is succeeded this duration before its actual completion.
   */
  ros::Duration GAIT_SUCCEEDED_OFFSET = ros::Duration(0.2);
  ros::Time getEndTimeCurrentGait();
  control_msgs::FollowJointTrajectoryGoal scheduleGait(const march_shared_resources::GaitGoal *gaitGoal,
                                                       ros::Duration offset);
};

#endif  // MARCH_GAIT_SCHEDULER_SCHEDULER_H