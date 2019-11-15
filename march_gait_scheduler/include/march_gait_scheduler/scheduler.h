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
public:
  /**
   * A gait is succeeded this duration before its actual completion.
   */
  ros::Duration GAIT_SUCCEEDED_OFFSET = ros::Duration(0.2);
  bool gaitDone = false;
  ros::Time getEndTimeCurrentGait();
  control_msgs::FollowJointTrajectoryGoal scheduleGait(const march_shared_resources::GaitGoal* gait_goal,
                                                       ros::Duration offset = ros::Duration(0, 0));

private:
  ros::Time getStartTime(ros::Duration offset);
  bool lastScheduledGaitNotStarted();
  static trajectory_msgs::JointTrajectory setStartTimeGait(trajectory_msgs::JointTrajectory trajectory, ros::Time time);

  const march_shared_resources::GaitGoal* last_gait_goal_ = nullptr;
  ros::Time start_last_gait_;
};

#endif  // MARCH_GAIT_SCHEDULER_SCHEDULER_H
