// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"

class ExampleTest : public ::testing::Test
{
protected:
};

TEST_F(ExampleTest, AlwaysTrue)
{
  ASSERT_EQ(3, 3);
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "testnode");
  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
