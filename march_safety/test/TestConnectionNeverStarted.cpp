// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Time.h>
#include <march_shared_resources/TopicNames.h>

#include <march_safety/TemperatureSafety.h>
#include "ErrorCounter.cpp"

class TestConnectionNeverStarted : public ::testing::Test
{
protected:
};

TEST_F(TestConnectionNeverStarted, connectionNeverStarted)
{
  ros::Time::init();
  ros::NodeHandle nh;
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, sub.getNumPublishers());

  ros::spinOnce();
  ros::Duration(0.5).sleep();
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
