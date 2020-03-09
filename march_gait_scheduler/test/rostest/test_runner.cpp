// Copyright 2018 Project March.

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testnode");
  testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
