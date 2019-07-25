// Copyright 2019 Project March.
#ifndef PROJECT_TEMPERATURESAFETY_H
#define PROJECT_TEMPERATURESAFETY_H

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "SafetyType.h"
#include "SafetyHandler.h"
#include <sstream>

#include <march_shared_resources/TopicNames.h>
#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>

class TemperatureSafety : public SafetyType
{
  ros::NodeHandle n;
  SafetyHandler* safety_handler;
  double default_temperature_threshold;
  double send_errors_interval;
  ros::Time time_last_send_error;
  std::map<std::string, double> fatal_temperature_thresholds_map;
  std::map<std::string, double> non_fatal_temperature_thresholds_map;
  std::map<std::string, double> warning_temperature_thresholds_map;
  std::vector<ros::Subscriber> temperature_subscribers = {};
  std::vector<std::string> joint_names;

  /**
   * This callback checks if the temperature values do not exceed the defined threshold
   * @param msg the temperature message
   * @param sensor_name the name of the sender
   */
  void temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name);

  /**
   * Create a subscriber for every sensor that publishes temperatures
   */
  void createSubscribers();

  std::string getErrorMessage(double temperature, const std::string& sensor_name);

  /**
   * Find the specific defined threshold for this sensor
   * If there is none, fall back to the default
   * @param sensor_name
   * @return the threshold
   */
  double getThreshold(const std::string& sensor_name, std::map<std::string, double> temperature_thresholds_map);

public:
  TemperatureSafety(ros::NodeHandle* n, SafetyHandler* safety_handler, std::vector<std::string> joint_names);

  void update() override
  {
  }
};

#endif  // PROJECT_TEMPERATURESAFETY_H
