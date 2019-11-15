// Copyright 2019 Project March.
#include "march_gait_scheduler/scheduler.h"

trajectory_msgs::JointTrajectory Scheduler::setStartTimeGait(trajectory_msgs::JointTrajectory trajectory,
                                                             ros::Time time)
{
  if (!time.isZero())
  {
    trajectory.header.stamp = time;
  }
  return trajectory;
}

ros::Time Scheduler::getEndTimeCurrentGait()
{
  if (this->last_gait_goal_ != nullptr && this->last_gait_goal_ != NULL)
  {
    ros::Time end_time = this->start_last_gait_;
    end_time += this->last_gait_goal_->current_subgait.duration;
    return end_time;
  }
  return ros::Time::now();
}

ros::Time Scheduler::getStartTime(ros::Duration offset)
{
  ros::Time end_current_gait = getEndTimeCurrentGait();
  ros::Time current_time = ros::Time::now();

  if (current_time > end_current_gait)
  {
    if (offset > ros::Duration(0, 0))
    {
      return current_time + offset;
    }
    else if (offset.isZero())
    {
      // Return zero time when gait can be scheduled right now,
      // so that it doesn't drop the first trajectory point.
      return ros::Time(0, 0);
    }
    else
    {
      ROS_WARN("Negative offset is ignored, because no gait is currently being "
               "executed");
      return current_time;
    }
  }
  else
  {
    if (end_current_gait + offset < current_time)
    {
      ROS_WARN("Negative offset is partly ignored, otherwise the gait would be "
               "scheduled in the past");
      return current_time;
    }
    else
    {
      return end_current_gait + offset;
    }
  }
}
bool Scheduler::lastScheduledGaitNotStarted()
{
  return ros::Time::now() < this->start_last_gait_;
}

control_msgs::FollowJointTrajectoryGoal Scheduler::scheduleGait(const march_shared_resources::GaitGoal* gait_goal,
                                                                ros::Duration offset)
{
  ROS_DEBUG("Start time last gait: %f", this->start_last_gait_.toSec());
  ROS_DEBUG("Current time: %f", ros::Time::now().toSec());

  if (this->lastScheduledGaitNotStarted())
  {
    throw std::runtime_error("There is already a gait scheduled in the future.");
  }
  ros::Time start_time = this->getStartTime(offset);
  trajectory_msgs::JointTrajectory trajectory =
      this->setStartTimeGait(gait_goal->current_subgait.trajectory, start_time);
  control_msgs::FollowJointTrajectoryGoal trajectory_msg;
  trajectory_msg.trajectory = trajectory;

  this->start_last_gait_ = start_time.isZero() ? ros::Time::now() : start_time;
  this->last_gait_goal_ = gait_goal;

  return trajectory_msg;
}
