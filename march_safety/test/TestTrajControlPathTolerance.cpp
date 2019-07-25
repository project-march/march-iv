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

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager_msgs/ListControllers.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

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
    traj_sub = nh.subscribe<trajectory_msgs::JointTrajectory>("/march/controller/trajectory/command", 1,
                                                              &JointTrajectoryControllerTest::trajCommandCb, this);

    // State subscriber
    state_sub =
        nh.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                &JointTrajectoryControllerTest::jointStateCb, this);

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
  ros::Subscriber traj_sub;
  ros::Subscriber state_sub;
  ActionClientPtr action_client;

  sensor_msgs::JointState joint_states;
  trajectory_msgs::JointTrajectory controller_command;
  trajectory_msgs::JointTrajectory empty_trajectory;

  double stop_trajectory_duration;

  void jointStateCb(const sensor_msgs::JointStateConstPtr& state)
  {
    std::lock_guard<std::mutex> lock(mutex);
    joint_states = *state;
  }

  void trajCommandCb(const trajectory_msgs::JointTrajectoryConstPtr& command)
  {
    std::lock_guard<std::mutex> lock(mutex);
    controller_command = *command;
  }

  sensor_msgs::JointState getJointState()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return joint_states;
  }

  trajectory_msgs::JointTrajectory getTrajCommand()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return controller_command;
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
  sensor_msgs::JointState start_state = getJointState();

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

  // Wait until done and that abort has been registered by client
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ABORTED, long_timeout));
  EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED);

  // check that the controller manager has stopped the controller, there is a little bit of time
  ros::Duration(0.1).sleep();
  EXPECT_EQ("stopped", getControllerStatus("march/controller/trajectory"));

  sensor_msgs::JointState state1 = getJointState();

  empty_trajectory.joint_names.clear();
  empty_trajectory.points.clear();

  EXPECT_EQ(empty_trajectory.points.size(), getTrajCommand().points.size());
  EXPECT_EQ(empty_trajectory.joint_names.size(), getTrajCommand().joint_names.size());

  // make sure that a movement has occurred
  EXPECT_NE(traj.points.front().positions[0], std::abs(start_state.position[0] - state1.position[0]));

  // Check that we're not moving
  ros::Duration(0.5).sleep();  // Wait
  sensor_msgs::JointState state2 = getJointState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(state1.position[i], state2.position[i], EPS);
    EXPECT_NEAR(state1.velocity[i], state2.velocity[i], EPS);
    EXPECT_NEAR(state1.effort[i], state2.effort[i], EPS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "march_safety_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}