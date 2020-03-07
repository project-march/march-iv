// Copyright 2019 Project March.
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <march_gait_scheduler/scheduler.h>

class EndTimeTest : public ::testing::Test
{
protected:
  trajectory_msgs::JointTrajectory fake_sit_trajectory()
  {
    march_shared_resources::GaitGoal goal;
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.joint_names = { "left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle" };
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = { 1.3, 1.3, 0.349065850399, 1.3, 1.3, 0.349065850399 };
    point.velocities = { 0, 0, 0, 0, 0, 0 };
    point.time_from_start = ros::Duration().fromSec(0);
    jointTrajectory.points = { point };
    return jointTrajectory;
  }
};

TEST_F(EndTimeTest, NothingScheduled)
{
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  Scheduler scheduler;
  ASSERT_NEAR(current_time.toSec(), scheduler.getEndTimeCurrentGait().toSec(), 0.1);
}

TEST_F(EndTimeTest, OnGaitScheduled)
{
  const double duration = 3;
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.current_subgait.duration = ros::Duration().fromSec(duration);
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  ASSERT_NEAR(current_time.toSec(), scheduler.getEndTimeCurrentGait().toSec(), 0.1);

  scheduler.scheduleGait(&gaitGoalConst, ros::Duration().fromSec(0));
  ASSERT_NEAR(current_time.toSec() + duration, scheduler.getEndTimeCurrentGait().toSec(), 0.1);
}

TEST_F(EndTimeTest, TwoGaitsScheduled)
{
  const double duration = 3;
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.current_subgait.duration = ros::Duration().fromSec(duration);
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  ASSERT_NEAR(current_time.toSec(), scheduler.getEndTimeCurrentGait().toSec(), 0.1);

  scheduler.scheduleGait(&gaitGoalConst, ros::Duration().fromSec(0));
  ASSERT_NEAR(current_time.toSec() + duration, scheduler.getEndTimeCurrentGait().toSec(), 0.1);

  scheduler.scheduleGait(&gaitGoalConst, ros::Duration().fromSec(0));
  ASSERT_NEAR(current_time.toSec() + 2 * duration, scheduler.getEndTimeCurrentGait().toSec(), 0.1);
}
