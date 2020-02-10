// Copyright 2019 Project March.

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include "error_counter.h"

TEST(TestTemperatureError, exceedDefaultThreshold)
{
  ros::NodeHandle nh;
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);
  ros::Publisher pub_joint3 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);

  while (0 == pub_joint3.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_joint3.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  int temperature;
  nh.getParam("/march_safety_node/default_temperature_threshold", temperature);
  msg.temperature = temperature + 1;
  pub_joint3.publish(msg);

  // Wait to receive message
  int timeout_duration;
  nh.getParam("/march_safety_node/ros_timeout", timeout_duration);

  ros::Duration duration = ros::Duration(timeout_duration);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(1, errorCounter.count);
}

TEST(TestTemperatureError, exceedDefaultThresholdMultipleTimes)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint3 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  printf("\n number of pub: %d \n", pub_joint3.getNumSubscribers());
  while (0 == pub_joint3.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_joint3.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  int temperature;
  nh.getParam("/march_safety_node/default_temperature_threshold", temperature);
  msg.temperature = temperature + 1;

  int times = 5;
  for (int i = 0; i < times; i++)
  {
    pub_joint3.publish(msg);
  }

  // Wait to receive message
  int timeout_duration;
  nh.getParam("/march_safety_node/ros_timeout", timeout_duration);

  ros::Duration duration = ros::Duration(timeout_duration);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(times, errorCounter.count);
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_safety_test");
  testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
