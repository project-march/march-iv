// Copyright 2019 Project March.

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include "error_counter.h"

TEST(TestNoTemperatureError, belowSpecificThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint1", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint1.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1u, pub_joint1.getNumSubscribers());
  EXPECT_EQ(1u, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  int temperature;
  nh.getParam("/march_safety_node/temperature_thresholds_non_fatal/test_joint1", temperature);
  msg.temperature = temperature - 1;
  pub_joint1.publish(msg);

  // Wait to receive message
  int timeout_duration;
  nh.getParam("/march_safety_node/ros_timeout", timeout_duration);

  ros::Duration duration = ros::Duration(timeout_duration);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint1", duration);
  ros::spinOnce();

  EXPECT_EQ(0u, errorCounter.count);
}

/**
 * Below specific threshold, but above the default threshold.
 * This tests if the specific threshold overrides the default threshold.
 */
TEST(TestNoTemperatureError, belowSpecificThreshold2)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint2 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint2", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint2.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1u, pub_joint2.getNumSubscribers());
  EXPECT_EQ(1u, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  int temperature;
  nh.getParam("/march_safety_node/temperature_thresholds_non_fatal/test_joint2", temperature);
  msg.temperature = temperature - 1;
  pub_joint2.publish(msg);

  // Wait to receive message
  int timeout_duration;
  nh.getParam("/march_safety_node/ros_timeout", timeout_duration);

  ros::Duration duration = ros::Duration(timeout_duration);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint2", duration);
  ros::spinOnce();

  EXPECT_EQ(0u, errorCounter.count);
}

TEST(TestNoTemperatureError, belowDefaultThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint3 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint3.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1u, pub_joint3.getNumSubscribers());
  EXPECT_EQ(1u, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  int temperature;
  nh.getParam("/march_safety_node/default_temperature_threshold", temperature);
  msg.temperature = temperature - 1;
  pub_joint3.publish(msg);

  // Wait to receive message
  int timeout_duration;
  nh.getParam("/march_safety_node/ros_timeout", timeout_duration);

  ros::Duration duration = ros::Duration(timeout_duration);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(0u, errorCounter.count);
}
