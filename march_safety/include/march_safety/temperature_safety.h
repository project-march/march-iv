// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_TEMPERATURE_SAFETY_H
#define MARCH_SAFETY_TEMPERATURE_SAFETY_H
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>

#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>

#include "march_safety/safety_type.h"
#include "march_safety/safety_handler.h"

class TemperatureSafety : public SafetyType
{
public:
  TemperatureSafety(ros::NodeHandle* n, SafetyHandler* safety_handler, std::vector<std::string> joint_names);

  void update() override
  {
  }

private:
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

  ros::NodeHandle* n_;
  SafetyHandler* safety_handler_;
  double default_temperature_threshold_;
  double send_errors_interval_;
  ros::Time time_last_send_error_;
  std::map<std::string, double> fatal_temperature_thresholds_map_;
  std::map<std::string, double> non_fatal_temperature_thresholds_map_;
  std::map<std::string, double> warning_temperature_thresholds_map_;
  std::vector<ros::Subscriber> temperature_subscribers_ = {};
  std::vector<std::string> joint_names_;
};

#endif  // MARCH_SAFETY_TEMPERATURE_SAFETY_H
