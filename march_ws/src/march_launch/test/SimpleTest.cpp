// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"

class cube_test : public ::testing::Test
{
 protected:
};

TEST_F(cube_test, TestNothing)
{
  ASSERT_EQ(3, 3);
}

TEST_F(cube_test, TestVolume)
{
  ASSERT_EQ(10, 10);
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "testNode");
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}