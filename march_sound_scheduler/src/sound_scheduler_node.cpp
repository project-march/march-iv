// Copyright 2018 Project March.
#include <ros/ros.h>

#include "march_sound_scheduler/scheduler.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_sound_scheduler");
  ros::NodeHandle n;

  ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("robotsound", 0);

  while (ros::ok() && 0 == sound_pub.getNumSubscribers())
  {
    ROS_DEBUG("Waiting on sound play topic");
    ros::Duration(0.1).sleep();
  }

  Scheduler scheduler;

  ros::Subscriber sound_sub = n.subscribe("/march/sound/schedule", 10, &Scheduler::scheduleMsg, &scheduler);

  ros::Rate rate(10);
  while (ros::ok())
  {
    scheduler.spin();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
