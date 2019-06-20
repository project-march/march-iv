// Copyright 2019 Project March.
#ifndef PROJECT_TEMPERATURESAFETY_H
#define PROJECT_TEMPERATURESAFETY_H

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "ErrorHandler.h"
#include <sstream>

#include <march_shared_resources/TopicNames.h>
#include <march_shared_resources/Error.h>

class TemperatureSafety
{
  ErrorHandler* errorHandler;
  ros::NodeHandle* n;
  double default_temperature_threshold;
  std::map<std::string, double> temperature_thresholds_map;
  std::vector<ros::Subscriber> temperature_subscribers = {};

  /**
   * This callback checks if the temperature values do not exceed the defined threshold
   * @param msg the temperature message
   * @param sensor_name the name of the sender
   */
  void temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name);

  /**
   * Create the temperature error message
   * @param temperature the measured temperature
   * @param sensor_name the sender of the temperature
   */
  march_shared_resources::Error createErrorMessage(double temperature, const std::string& sensor_name);

  /**
   * Create a subscriber for every sensor that publishes temperatures
   */
  void createSubscribers();

  /**
   * Find the specific defined threshold for this sensor
   * If there is none, fall back to the default
   * @param sensor_name
   * @return the threshold
   */
  double getThreshold(const std::string& sensor_name);

public:
  TemperatureSafety(ErrorHandler* errorHandler, ros::NodeHandle* n);
};

#endif  // PROJECT_TEMPERATURESAFETY_H
