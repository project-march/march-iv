// Copyright 2019 Project March.
#include "ros/ros.h"
#include "gtest/gtest.h"
#include "CallbackCounter.cpp"
#include <march_gait_scheduler/Scheduler.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <march_shared_resources/GaitAction.h>

class AcceptanceTest : public ::testing::Test
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
};

 TEST_F(AcceptanceTest, ScheduleNoGaits)
{
  ros::NodeHandle nh;
  ros::Time::init();
  CallbackCounter callbackCounter;

  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ServerFollowJoint;
  ServerFollowJoint scheduleGaitActionServer(nh, "/march/controller/trajectory/follow_joint_trajectory",
                                                boost::bind(&CallbackCounter::cb_action, &callbackCounter, _1), false);
  scheduleGaitActionServer.start();

  actionlib::SimpleActionClient<march_shared_resources::GaitAction> scheduleGaitAction("march/gait/schedule", true);
  scheduleGaitAction.waitForServer(ros::Duration(2));
  EXPECT_TRUE(scheduleGaitAction.isServerConnected());

  ros::Duration timeout_duration = ros::Duration(1);
  ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionGoal>(
      "/march/controller/trajectory/follow_joint_trajectory/goal", timeout_duration);
  ros::spinOnce();

   EXPECT_EQ(0, callbackCounter.count);
}

TEST_F(AcceptanceTest, ScheduleOneGait)
{
  ros::NodeHandle nh;
  ros::Time::init();
  CallbackCounter callbackCounter;

  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ServerFollowJoint;
  ServerFollowJoint scheduleGaitActionServer(nh, "/march/controller/trajectory/follow_joint_trajectory",
                                                boost::bind(&CallbackCounter::cb_action, &callbackCounter, _1), false);

  scheduleGaitActionServer.start();

  actionlib::SimpleActionClient<march_shared_resources::GaitAction> scheduleGaitAction("march/gait/schedule", true);
  scheduleGaitAction.waitForServer(ros::Duration(2));
  EXPECT_TRUE(scheduleGaitAction.isServerConnected());

  march_shared_resources::GaitGoal goal = fake_sit_goal();
  scheduleGaitAction.sendGoal(goal);

  ros::Rate rate(100);
  double time_out_time = ros::Time::now().toSec() + 1;
  while (callbackCounter.count == 0)
  {
    if (ros::Time::now().toSec() > time_out_time)
    {
      FAIL() << "Time out, callback not received in 1 second";
    }
    rate.sleep();
    ros::spinOnce();
  }

  EXPECT_EQ(1, callbackCounter.count);
}
