// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Time.h>
#include <march_shared_resources/TopicNames.h>

#include <march_safety/TemperatureSafety.h>
#include "ErrorCounter.cpp"

class TestConnectionNotLost : public ::testing::Test
{
protected:
};

TEST_F(TestConnectionNotLost, connectionNotLost)
{
  ros::Time::init();
  ros::NodeHandle nh;
  int send_errors_interval;
  nh.getParam("/march_safety_node/send_errors_interval", send_errors_interval);
  ros::Publisher pub_alive = nh.advertise<std_msgs::Time>("march/input_device/alive", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_alive.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_alive.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  std_msgs::Time timeMessage;
  timeMessage.data = ros::Time::now();
  pub_alive.publish(timeMessage);
  ros::spinOnce();
  ros::Duration(send_errors_interval / 2 / 1000).sleep();
  ros::spinOnce();
  EXPECT_EQ(0, errorCounter.count);
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
