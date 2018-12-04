// Copyright 2018 Project March.

#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "../src/common/communication/TopicNames.h"
#include "LaunchAPI.h"
#include "MovementAPI.h"

int main(int argc, char** argv)
{
  ROS_INFO("Starting the api_node...");
  ros::init(argc, argv, "api_node");
  ros::NodeHandle n;

  ros::ServiceServer config_validator =
      n.advertiseService(ServiceNames::config_validation, LaunchAPI::config_validator);
  ros::ServiceServer urdf_validator = n.advertiseService(ServiceNames::urdf_validation, LaunchAPI::urdf_validator);

  ros::ServiceServer xml_validator = n.advertiseService(ServiceNames::xml_validation, LaunchAPI::xml_validator);
  ros::ServiceServer move_to_gait_file =
      n.advertiseService(ServiceNames::request_gait_file, MovementAPI::request_gait_file);

  ROS_INFO("api_node started successfully!");
  ros::Rate rate(100);
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
