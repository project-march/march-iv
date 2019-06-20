// Copyright 2018 Project March.

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Temperature.h"
#include <sstream>

#include <march_shared_resources/TopicNames.h>
#include <march_shared_resources/Error.h>

#include <march_safety/InputDeviceSafety.h>
#include <march_safety/TemperatureSafety.h>
#include <march_safety/ErrorHandler.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_safety_node");
  ros::NodeHandle n;
  ros::Rate rate(200);

  // Create an error publisher to notify the system (state machine) if something is wrong
  ros::Publisher error_publisher = n.advertise<march_shared_resources::Error>(TopicNames::error, 1000);


  ErrorHandler errorHandler = ErrorHandler(&error_publisher, &n);

  TemperatureSafety temperatureSafety = TemperatureSafety(&errorHandler, &n);

  InputDeviceSafety inputDeviceSafety = InputDeviceSafety(&errorHandler, &n);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    inputDeviceSafety.checkConnection();
  }

  return 0;
}
