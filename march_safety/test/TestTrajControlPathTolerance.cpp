///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <sensor_msgs/JointState.h>

// Floating-point value comparison threshold
const double EPS = 0.01;

using actionlib::SimpleClientGoalState;

class JointTrajectoryControllerTest : public ::testing::Test
{
public:
  JointTrajectoryControllerTest()
    : nh("march_controller"), short_timeout(1.0), long_timeout(10.0), stop_trajectory_duration(0.0)
  {
    n_joints = (2);
    joint_names.resize(n_joints);
    joint_names[0] = "joint1";
    joint_names[1] = "joint2";

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints, 0.0);
    point.velocities.resize(n_joints, 0.0);
    point.accelerations.resize(n_joints, 0.0);

    // Go home trajectory
    traj_home.joint_names = joint_names;
    traj_home.points.resize(1, point);
    traj_home.points[0].time_from_start = ros::Duration(1.0);

    // Three-point trajectory
    points.resize(3, point);
    points[0].positions[0] = M_PI / 4.0;
    points[0].positions[1] = 0.0;
    points[0].time_from_start = ros::Duration(1.0);

    points[1].positions[0] = 0.0;
    points[1].positions[1] = -M_PI / 4.0;
    points[1].time_from_start = ros::Duration(2.0);

    points[2].positions[0] = -M_PI / 4.0;
    points[2].positions[1] = M_PI / 4.0;
    points[2].time_from_start = ros::Duration(4.0);

    traj.joint_names = joint_names;
    traj.points = points;

    // Action goals
    traj_home_goal.trajectory = traj_home;
    traj_goal.trajectory = traj;

    // Smoothing publisher (determines how well the robot follows a trajectory)
    smoothing_pub = ros::NodeHandle().advertise<std_msgs::Float64>("smoothing", 1);

    // Trajectory publisher
    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/march/controller/trajectory/command", 1);

    // State subscriber
    state_sub =
        nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &JointTrajectoryControllerTest::stateCB, this);

    // Query state service client
    query_state_service = nh.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

    // Controller management services
    load_controller_service =
        nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    unload_controller_service =
        nh.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");
    switch_controller_service =
        nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    // Action client
    const std::string action_server_name = "/march/controller/trajectory/follow_joint_trajectory";
    action_client.reset(new ActionClient(action_server_name));

    nh.getParam("stop_trajectory_duration", stop_trajectory_duration);
  }

  ~JointTrajectoryControllerTest() override
  {
    state_sub.shutdown();  // This is important, to make sure that the callback is not woken up later in the destructor
  }

protected:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef std::shared_ptr<ActionClient> ActionClientPtr;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;

  std::mutex mutex;
  ros::NodeHandle nh;

  unsigned int n_joints;
  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectory traj_home;
  trajectory_msgs::JointTrajectory traj;
  ActionGoal traj_home_goal;
  ActionGoal traj_goal;

  ros::Duration short_timeout;
  ros::Duration long_timeout;

  ros::Publisher smoothing_pub;
  ros::Publisher delay_pub;
  ros::Publisher upper_bound_pub;
  ros::Publisher traj_pub;
  ros::Subscriber state_sub;
  ActionClientPtr action_client;

  sensor_msgs::JointState controller_state;
  std::vector<double> controller_min_actual_velocity;
  std::vector<double> controller_max_actual_velocity;

  double stop_trajectory_duration;

  void stateCB(const sensor_msgs::JointStateConstPtr& state)
  {
    std::lock_guard<std::mutex> lock(mutex);
    controller_state = *state;

    std::transform(controller_min_actual_velocity.begin(), controller_min_actual_velocity.end(),
                   state->velocity.begin(), controller_min_actual_velocity.begin(),
                   [](double a, double b) { return std::min(a, b); });
    std::transform(controller_max_actual_velocity.begin(), controller_max_actual_velocity.end(),
                   state->velocity.begin(), controller_max_actual_velocity.begin(),
                   [](double a, double b) { return std::max(a, b); });
  }

  sensor_msgs::JointState getState()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return controller_state;
  }

  bool waitForNextState(const ros::Duration& timeout)
  {
    ros::Time start_time = ros::Time::now();
    ros::Time state_time = getState().header.stamp;
    while (getState().header.stamp <= state_time && ros::ok())
    {
      if (timeout >= ros::Duration(0.0) && (ros::Time::now() - start_time) > timeout)
      {
        return false;
      }  // Timed-out
      ros::Duration(0.001).sleep();
    }
  }

  static bool waitForState(const ActionClientPtr& action_client, const actionlib::SimpleClientGoalState& state,
                           const ros::Duration& timeout)
  {
    using ros::Time;
    using ros::Duration;

    Time start_time = Time::now();
    while (action_client->getState() != state && ros::ok())
    {
      if (timeout >= Duration(0.0) && (Time::now() - start_time) > timeout)
      {
        return false;
      }  // Timed-out
      ros::Duration(0.01).sleep();
    }
    return true;
  }

  std::string getControllerStatus(const std::string& controller_name)
  {
    ros::ServiceClient client = this->nh.serviceClient<controller_manager_msgs::ListControllers>(
        ros::this_node::getNamespace() + "/controller_manager/list_controllers");
    auto ctr = controller_manager_msgs::ListControllers();

    client.call(ctr);

    for (auto& it : ctr.response.controller)
    {
      if (it.name == controller_name)
      {
        return it.state;
      }
    }
  }
};

TEST_F(JointTrajectoryControllerTest, pathToleranceViolation)
{
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0);  // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Make robot respond with a delay
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.9;
    smoothing_pub.publish(smoothing);
    ros::Duration(0.5).sleep();
  }

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0);  // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  // Wait until done
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ABORTED, long_timeout));
  EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED);

  // check that the controller manager has stopped the controller
  EXPECT_EQ("stopped", getControllerStatus("march/controller/trajectory"));

  sensor_msgs::JointState state2 = getState();
  //
  EXPECT_EQ(state1.effort[1], state2.effort[1]);

  ros::Duration(0.5).sleep();

  sensor_msgs::JointState state3 = getState();
  EXPECT_EQ(state3.effort[1], state2.effort[1]);

  // Restore perfect control
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.0;
    smoothing_pub.publish(smoothing);
    ros::Duration(0.5).sleep();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joint_trajectory_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}