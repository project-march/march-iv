// Copyright 2018 Project March.

#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <march_shared_resources/GaitAction.h>
#include <march_shared_resources/GaitGoal.h>
#include <march_gait_scheduler/Scheduler.h>
#include <march_gait_scheduler/SchedulerConfig.h>

typedef actionlib::SimpleActionServer<march_shared_resources::GaitAction> ScheduleGaitActionServer;
ScheduleGaitActionServer* scheduleGaitActionServer;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* followJointTrajectoryAction;
Scheduler* scheduler;

void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  if (!scheduleGaitActionServer->isActive() || scheduler->gaitDone)
  {
    ROS_INFO("Gait already done or action already ended");
    return;
  }
  if (result->error_code == result->SUCCESSFUL)
  {
    scheduleGaitActionServer->setSucceeded();
    ROS_WARN("Gait trajectory execution DONE, this should only happen when GAIT_SUCCEEDED_OFFSET is 0");
    ROS_INFO("Schedule gait action SUCCEEDED");
  }
  else
  {
    scheduleGaitActionServer->setAborted();
    ROS_WARN("Schedule gait action FAILED, FollowJointTrajectory error_code is %d ", result->error_code);
  }
}

void activeCallback()
{
  ROS_DEBUG("Gait trajectory goal just went active");
}

void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  if (scheduler->getEndTimeCurrentGait().toSec() - feedback->header.stamp.toSec() <
      scheduler->GAIT_SUCCEEDED_OFFSET.toSec())
  {
    if (scheduler->getEndTimeCurrentGait().toSec() - feedback->header.stamp.toSec() < 0)
    {
      ROS_ERROR("Negative difference");
      return;
    }
    if (!scheduleGaitActionServer->isActive() || scheduler->gaitDone)
    {
      ROS_INFO("Gait already done or action already ended");
      return;
    }
    scheduler->gaitDone = true;
    scheduleGaitActionServer->setSucceeded();
    ROS_DEBUG("Schedule gait action SUCCEEDED");
  }
}

void scheduleGaitCallback(const march_shared_resources::GaitGoalConstPtr& goal)
{
  ROS_DEBUG("Gait Scheduler received msg to schedule: %s of %s", goal->current_subgait.name.c_str(),
            goal->name.c_str());
  try
  {
    control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
        scheduler->scheduleGait(goal.get(), ros::Duration().fromSec(0));
    followJointTrajectoryAction->sendGoal(trajectoryMsg, &doneCallback, &activeCallback, &feedbackCallback);
    scheduler->gaitDone = false;
    ROS_DEBUG("follow joint trajectory action called");
  }
  catch (std::runtime_error error)
  {
    ROS_ERROR("When trying to schedule a trajectory the following error occurred: %s", error.what());
    ROS_ERROR("Gait scheduling ABORTED");
    scheduleGaitActionServer->setAborted();
    return;
  }

  ros::Rate r(100);
  while (scheduleGaitActionServer->isActive())
  {
    r.sleep();
    ROS_DEBUG_THROTTLE(1, "Waiting on gait execution");
  }
}

/**
 * This callback is called when parameters from the config file are changed during run-time.
 * This method updates the local values which depend on these parameters to make ensure the values are not out-of-date.
 * @param config the config file with all the parameters
 * @param level A bitmask
 */
void schedulerConfigCallback(march_gait_scheduler::SchedulerConfig& config, uint32_t level)
{
  scheduler->GAIT_SUCCEEDED_OFFSET = ros::Duration(config.gait_succeeded_offset);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_scheduler_node");
  ros::NodeHandle n;

  scheduler = new Scheduler();

  followJointTrajectoryAction = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
      "/march/controller/trajectory/follow_joint_trajectory", true);

  ROS_DEBUG("Wait on joint trajectory action server");
  bool isConnected = followJointTrajectoryAction->waitForServer(ros::Duration(10));
  if (isConnected)
  {
    ROS_INFO("Connected to joint trajectory action server");
  }
  else
  {
    ROS_ERROR("Not connected to joint trajectory action server");
  }

  dynamic_reconfigure::Server<march_gait_scheduler::SchedulerConfig> server;
  server.setCallback(boost::bind(&schedulerConfigCallback, _1, _2));

  scheduleGaitActionServer = new ScheduleGaitActionServer(n, "march/gait/schedule", &scheduleGaitCallback, false);
  scheduleGaitActionServer->start();

  ros::spin();

  return 0;
}
