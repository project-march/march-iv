// Copyright 2018 Project March.

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Temperature.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <sstream>
#include <vector>

#include <march_shared_resources/TopicNames.h>
#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>

#include <march_safety/InputDeviceSafety.h>
#include <march_safety/TemperatureSafety.h>
#include <march_safety/TrajectorySafety.h>
#include <march_safety/SafetyHandler.h>
#include <urdf/model.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_safety_node");
  ros::NodeHandle n;
  ros::Rate rate(200);
  std::string controller_name;

  // Create an error publisher to notify the system (state machine) if something is wrong
  ros::Publisher error_publisher = n.advertise<march_shared_resources::Error>("/march/error", 1000);
  ros::Publisher sound_publisher = n.advertise<march_shared_resources::Sound>("/march/sound/schedule", 1000);

    n.getParam(ros::this_node::getName() + std::string("/controller_name"), controller_name);
  ros::Publisher stop_trajectory_publisher =
      n.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 1000);

  SafetyHandler safetyHandler = SafetyHandler(&n, &error_publisher, &sound_publisher, &stop_trajectory_publisher);

  std::vector<SafetyType> safety_list;

  int count = 0;
  while (!n.hasParam("/march/joint_names"))
  {
    ros::Duration(0.5).sleep();
    count++;
    if (count > 10)
    {
      ROS_ERROR("Failed to read the joint_names from the parameter server.");
      throw std::runtime_error("Failed to read the joint_names from the parameter server.");
    }
  }

  std::vector<std::string> joint_names;
  n.getParam("/march/joint_names", joint_names);

  TemperatureSafety temperatureSafety = TemperatureSafety(&n, &safetyHandler, joint_names);
  safety_list.push_back(temperatureSafety);

  InputDeviceSafety inputDeviceSafety = InputDeviceSafety(&n, &safetyHandler);
  safety_list.push_back(inputDeviceSafety);

  TrajectorySafety trajectorySafety = TrajectorySafety(&n, &safetyHandler, joint_names);
  safety_list.push_back(trajectorySafety);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    for (auto& i : safety_list)
    {
      i.update();
    }
  }

  return 0;
}
