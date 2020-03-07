// Copyright 2019 Project March.
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <march_gait_scheduler/scheduler.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class ScheduleOneGaitTest : public ::testing::Test
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
    point.time_from_start = ros::Duration().fromSec(3);
    jointTrajectory.points = { point };
    return jointTrajectory;
  }
};

TEST_F(ScheduleOneGaitTest, ScheduleNow)
{
  ros::Time::init();
  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
      scheduler.scheduleGait(&gaitGoalConst, ros::Duration().fromSec(0));

  ASSERT_TRUE(trajectoryMsg.trajectory.header.stamp.isZero());
}

TEST_F(ScheduleOneGaitTest, ScheduledTrajectoryTheSameAsRequested)
{
  ros::Time::init();
  march_shared_resources::GaitGoal gaitGoal;
  const trajectory_msgs::JointTrajectory& trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.trajectory = trajectory;
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
      scheduler.scheduleGait(&gaitGoalConst, ros::Duration().fromSec(0));
  for (int i = 0; (unsigned)i < trajectory.points.size(); i++)
  {
    trajectory_msgs::JointTrajectoryPoint pointExpected = trajectory.points[i];
    trajectory_msgs::JointTrajectoryPoint point = trajectoryMsg.trajectory.points[i];
    ASSERT_NEAR(pointExpected.time_from_start.toNSec(), point.time_from_start.toNSec(), 0.1);
    ASSERT_NEAR(pointExpected.accelerations.size(), point.accelerations.size(), 0.1);
    ASSERT_NEAR(pointExpected.velocities[1], point.velocities[1], 0.1);
    ASSERT_NEAR(pointExpected.positions[2], point.positions[2], 0.1);
    ASSERT_NEAR(pointExpected.effort.size(), point.effort.size(), 0.1);
  }
}

TEST_F(ScheduleOneGaitTest, ScheduleInTheFuture)
{
  ros::Time::init();
  ros::Time currentTime = ros::Time::now();
  ros::Duration offset = ros::Duration().fromSec(50);
  ros::Time futureTime = currentTime + offset;

  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryGoal trajectoryMsg = scheduler.scheduleGait(&gaitGoalConst, offset);

  ASSERT_NEAR(futureTime.toSec(), trajectoryMsg.trajectory.header.stamp.toSec(), 0.1);
  ASSERT_NE(currentTime.toSec(), trajectoryMsg.trajectory.header.stamp.toSec());
}

TEST_F(ScheduleOneGaitTest, ScheduleInThePast)
{
  ros::Time::init();
  ros::Time currentTime = ros::Time::now();
  ros::Duration offset = ros::Duration().fromSec(-50);

  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryGoal trajectoryMsg = scheduler.scheduleGait(&gaitGoalConst, offset);

  ASSERT_NEAR(currentTime.toSec(), trajectoryMsg.trajectory.header.stamp.toSec(), 0.1);
  ASSERT_NE(currentTime.toSec(), trajectoryMsg.trajectory.header.stamp.toSec());
}
