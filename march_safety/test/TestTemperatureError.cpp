// Copyright 2019 Project March.

#include <march_shared_resources/TopicNames.h>
#include "gtest/gtest.h"
#include "ros/ros.h"
#include <march_safety/TemperatureSafety.h>
#include "ErrorCounter.cpp"

class TestTemperatureError : public ::testing::Test
{
protected:
};

TEST_F(TestTemperatureError, exceedSpecificThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint1", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint1.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.01).sleep();
  }
  EXPECT_EQ(1, pub_joint1.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  int temperature;
  nh.getParam("/march_safety_node/temperature_thresholds_non_fatal/test_joint1", temperature);
  msg.temperature = temperature + 1;
  pub_joint1.publish(msg);

  // Wait to receive message
  ros::Duration duration = ros::Duration(0.1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint1", duration);
  ros::spinOnce();

  EXPECT_EQ(1, errorCounter.count);
}

TEST_F(TestTemperatureError, exceedDefaultThreshold)
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
  ros::Duration duration = ros::Duration(0.1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(1, errorCounter.count);
}

TEST_F(TestTemperatureError, exceedDefaultThresholdMultipleTimes)
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
  ros::Duration duration = ros::Duration(0.1);
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
  testing::FLAGS_gtest_filter = "TestTemperatureError.*";
  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
