// Copyright 2018 Project March.

#include "ros/ros.h"
#include "input_device_node.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Time.h"
#include "../../march_main/src/common/communication/TopicNames.h"
#include <march_custom_msgs/GaitInput.h>
#include <march_custom_msgs/PlayInput.h>

void gaitDoneCallback(const march_custom_msgs::GaitInput msg)
{
}

void gaitPerformingCallback(const march_custom_msgs::GaitInput msg)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input_device_node");
  ros::NodeHandle n;
  ros::Rate rate(200);

  ros::Publisher input_device_gait = n.advertise<march_custom_msgs::GaitInput>(TopicNames::input_device_gait, 1000);
  ros::Publisher input_device_stop = n.advertise<std_msgs::Time>(TopicNames::input_device_stop, 1000);
  ros::Publisher input_device_trigger = n.advertise<std_msgs::Time>(TopicNames::input_device_trigger, 1000);

  ros::Subscriber input_device_gait_done = n.subscribe(TopicNames::input_device_gait_done, 1000, gaitDoneCallback);
  ros::Subscriber input_device_gait_performing =
      n.subscribe(TopicNames::input_device_gait_performing, 1000, gaitPerformingCallback);

  while (ros::ok())
  {
    rate.sleep();

    ros::spinOnce();
  }

  return 0;
}
