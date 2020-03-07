// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Time.h>

#include <march_safety/temperature_safety.h>
#include "error_counter.h"

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

  while (0u == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1u, sub.getNumPublishers());

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  EXPECT_EQ(0u, errorCounter.count);
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
