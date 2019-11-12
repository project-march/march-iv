// Copyright 2019 Project March.
#include "ros/ros.h"
#include "gtest/gtest.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <march_gait_scheduler/scheduler.h>

class ScheduleMultipleGaitsTest : public ::testing::Test
{
protected:
  static march_shared_resources::GaitGoal fake_sit_goal()
  {
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.joint_names = { "left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle" };
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = { 1.3, 1.3, 0.349065850399, 1.3, 1.3, 0.349065850399 };
    point.velocities = { 0, 0, 0, 0, 0, 0 };
    point.time_from_start = ros::Duration().fromSec(3);
    jointTrajectory.points = { point };

    march_shared_resources::GaitGoal sitGaitGoal;
    sitGaitGoal.current_subgait.trajectory = jointTrajectory;
    sitGaitGoal.current_subgait.name = "sit";
    sitGaitGoal.current_subgait.duration = ros::Duration().fromSec(3);
    sitGaitGoal.name = "sit";
    return sitGaitGoal;
  }

  static march_shared_resources::GaitGoal fake_stand_goal()
  {
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.joint_names = { "left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle" };
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = { 0, 0, 0, 0, 0, 0 };
    point.velocities = { 0, 0, 0, 0, 0, 0 };
    point.time_from_start = ros::Duration().fromSec(3);
    jointTrajectory.points = { point };

    march_shared_resources::GaitGoal standGaitGoal;
    standGaitGoal.current_subgait.trajectory = jointTrajectory;
    standGaitGoal.current_subgait.name = "stand";
    standGaitGoal.current_subgait.duration = ros::Duration().fromSec(3);
    standGaitGoal.name = "stand";
    return standGaitGoal;
  }
};

TEST_F(ScheduleMultipleGaitsTest, ScheduleTwoNoOffset)
{
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitSitGoal = fake_sit_goal();
  ros::Duration gaitDuration = gaitSitGoal.current_subgait.duration;

  const auto& gaitSitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitSitGoal);

  march_shared_resources::GaitGoal gaitStandGoal = fake_stand_goal();
  const auto& gaitStandGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitStandGoal);

  Scheduler scheduler;

  control_msgs::FollowJointTrajectoryGoal trajectoryMsgSit =
      scheduler.scheduleGait(&gaitSitGoalConst, ros::Duration().fromSec(0));

  control_msgs::FollowJointTrajectoryGoal trajectoryMsgStand =
      scheduler.scheduleGait(&gaitStandGoalConst, ros::Duration().fromSec(0));

  ros::Time secondStartTime = (current_time + gaitDuration);

  ASSERT_NEAR(current_time.toSec(), trajectoryMsgSit.trajectory.header.stamp.toSec(), 0.1);
  ASSERT_NEAR(secondStartTime.toSec(), trajectoryMsgStand.trajectory.header.stamp.toSec(), 0.1);
}

TEST_F(ScheduleMultipleGaitsTest, ScheduleTwoWithOffset)
{
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitSitGoal = fake_sit_goal();
  ros::Duration gaitDuration = gaitSitGoal.current_subgait.duration;

  const auto& gaitSitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitSitGoal);

  march_shared_resources::GaitGoal gaitStandGoal = fake_stand_goal();
  const auto& gaitStandGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitStandGoal);

  Scheduler scheduler;

  control_msgs::FollowJointTrajectoryGoal trajectoryMsgSit =
      scheduler.scheduleGait(&gaitSitGoalConst, ros::Duration().fromSec(0));

  ros::Duration offset = ros::Duration().fromSec(3);
  control_msgs::FollowJointTrajectoryGoal trajectoryMsgStand = scheduler.scheduleGait(&gaitStandGoalConst, offset);

  ros::Time secondStartTime = current_time + gaitDuration + offset;

  ASSERT_NEAR(current_time.toSec(), trajectoryMsgSit.trajectory.header.stamp.toSec(), 0.1);
  ASSERT_NEAR(secondStartTime.toSec(), trajectoryMsgStand.trajectory.header.stamp.toSec(), 0.1);
}

TEST_F(ScheduleMultipleGaitsTest, ScheduleThreeNoOffset)
{
  ros::Time::init();
  march_shared_resources::GaitGoal gaitSitGoal = fake_sit_goal();
  ros::Duration gaitDuration = gaitSitGoal.current_subgait.duration;

  const auto& gaitSitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitSitGoal);

  march_shared_resources::GaitGoal gaitStandGoal = fake_stand_goal();
  const auto& gaitStandGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitStandGoal);

  Scheduler scheduler;
  scheduler.scheduleGait(&gaitSitGoalConst, ros::Duration().fromSec(0));
  scheduler.scheduleGait(&gaitStandGoalConst, ros::Duration().fromSec(0));

  ASSERT_THROW(scheduler.scheduleGait(&gaitSitGoalConst, ros::Duration().fromSec(0)), std::runtime_error);
}

TEST_F(ScheduleMultipleGaitsTest, ScheduleSecondGaitInThePast)
{
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitSitGoal = fake_sit_goal();
  ros::Duration gaitDuration = gaitSitGoal.current_subgait.duration;

  const auto& gaitSitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitSitGoal);

  march_shared_resources::GaitGoal gaitStandGoal = fake_stand_goal();
  const auto& gaitStandGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitStandGoal);

  Scheduler scheduler;

  control_msgs::FollowJointTrajectoryGoal trajectoryMsgSit =
      scheduler.scheduleGait(&gaitSitGoalConst, ros::Duration().fromSec(0));

  ros::Duration offset = ros::Duration().fromSec(-6);
  control_msgs::FollowJointTrajectoryGoal trajectoryMsgStand = scheduler.scheduleGait(&gaitStandGoalConst, offset);

  ASSERT_NEAR(current_time.toSec(), trajectoryMsgSit.trajectory.header.stamp.toSec(), 0.1);
  ASSERT_NEAR(current_time.toSec(), trajectoryMsgStand.trajectory.header.stamp.toSec(), 0.1);
}
