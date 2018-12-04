// Copyright 2018 Project March.

#include "ros/ros.h"
#include "input_device_node.h"
#include "std_msgs/Float64.h"
#include "../../march_main/src/common/communication/TopicNames.h"
#include <march_custom_msgs/GaitInput.h>
#include <march_custom_msgs/PlayInput.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "input_device_node");
  ros::NodeHandle n;
  ros::Rate rate(200);
  ros::Publisher input_device_gait = n.advertise<march_custom_msgs::GaitInput>(TopicNames::input_device_gait, 1000);
  ros::Publisher input_device_play = n.advertise<march_custom_msgs::PlayInput>(TopicNames::input_device_play, 1000);

  while (ros::ok()) {
    rate.sleep();
//    march_custom_msgs::GaitInput srv;
//    srv.request.gait_name = "Walking"
//    srv.request.time = ros::Time::now();
//    gait_input_client.call(srv);
    ros::spinOnce();
  }

  return 0;
}
