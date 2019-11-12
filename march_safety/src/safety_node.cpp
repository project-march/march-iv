// Copyright 2018 Project March.
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Temperature.h>
#include <urdf/model.h>

#include "march_shared_resources/Error.h"
#include "march_shared_resources/Sound.h"
#include "march_shared_resources/GaitInstruction.h"

#include "march_safety/input_device_safety.h"
#include "march_safety/temperature_safety.h"
#include "march_safety/safety_handler.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_safety_node");
  ros::NodeHandle n;
  ros::Rate rate(200);

  ROS_DEBUG("Trying to get parameter /march/joint_names");
  while (ros::ok() && !n.hasParam("/march/joint_names"))
  {
    ros::Duration(0.5).sleep();
    ROS_DEBUG("Waiting on /march/joint_names to be available");
  }

  std::vector<std::string> joint_names;
  n.getParam("/march/joint_names", joint_names);
  ROS_DEBUG("Got joint names");

  // Create an error publisher to notify the system (state machine) if something is wrong
  ros::Publisher error_publisher = n.advertise<march_shared_resources::Error>("/march/error", 1000);
  ros::Publisher sound_publisher = n.advertise<march_shared_resources::Sound>("/march/sound/schedule", 1000);
  ros::Publisher gait_instruction_publisher =
      n.advertise<march_shared_resources::GaitInstruction>("/march/input_device/instruction", 1000);

  SafetyHandler safety_handler = SafetyHandler(&n, &error_publisher, &sound_publisher, &gait_instruction_publisher);

  std::vector<std::unique_ptr<SafetyType>> safety_list;
  safety_list.push_back(std::unique_ptr<SafetyType>(new TemperatureSafety(&n, &safety_handler, joint_names)));
  safety_list.push_back(std::unique_ptr<SafetyType>(new InputDeviceSafety(&n, &safety_handler)));

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    for (auto& i : safety_list)
    {
      i->update();
    }
  }

  return 0;
}
