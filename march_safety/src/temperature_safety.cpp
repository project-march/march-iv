// Copyright 2019 Project March.
#include "march_safety/temperature_safety.h"

#include <map>
#include <string>
#include <vector>

// TODO(@Tim) Throw an exception when no temperatures are published.

TemperatureSafety::TemperatureSafety(ros::NodeHandle* n, SafetyHandler* safety_handler,
                                     std::vector<std::string> joint_names)
  : n_(n), safety_handler_(safety_handler), joint_names_(std::move(joint_names))
{
  ros::param::get("~default_temperature_threshold", this->default_temperature_threshold_);
  ros::param::get("~temperature_thresholds_warning", this->warning_temperature_thresholds_map_);
  ros::param::get("~temperature_thresholds_non_fatal", this->non_fatal_temperature_thresholds_map_);
  ros::param::get("~temperature_thresholds_fatal", this->fatal_temperature_thresholds_map_);
  double send_errors_interval_param;
  ros::param::get("~send_errors_interval", send_errors_interval_param);
  this->send_errors_interval_ = send_errors_interval_param;
  this->time_last_send_error_ = ros::Time(0);

  this->createSubscribers();
}

void TemperatureSafety::temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name)
{
  // send at most an error every second
  if (ros::Time::now() <= (this->time_last_send_error_ + ros::Duration(this->send_errors_interval_ / 1000)))
  {
    return;
  }

  double temperature = msg->temperature;
  if (temperature <= getThreshold(sensor_name, this->warning_temperature_thresholds_map_))
  {
    return;
  }

  std::string error_message = this->getErrorMessage(temperature, sensor_name);

  // TODO(Olav) this is a temporary fix, this should be fixed locally on the slaves ask Electro.
  if (temperature > 200)
  {
    ROS_WARN("%s", error_message.c_str());
    return;
  }

  // If the threshold is exceeded raise an error
  if (temperature > this->getThreshold(sensor_name, this->fatal_temperature_thresholds_map_))
  {
    this->safety_handler_->publishFatal(error_message);
  }
  else if (temperature > this->getThreshold(sensor_name, this->non_fatal_temperature_thresholds_map_))
  {
    this->safety_handler_->publishNonFatal(error_message);
  }
  else if (temperature > this->getThreshold(sensor_name, this->warning_temperature_thresholds_map_))
  {
    ROS_WARN("%s", error_message.c_str());
  }
}

std::string TemperatureSafety::getErrorMessage(double temperature, const std::string& sensor_name)
{
  std::ostringstream message_stream;
  message_stream << sensor_name << " temperature too high: " << temperature;
  std::string error_message = message_stream.str();
  return error_message;
}

double TemperatureSafety::getThreshold(const std::string& sensor_name,
                                       std::map<std::string, double> temperature_thresholds_map)
{
  if (temperature_thresholds_map.find(sensor_name) != temperature_thresholds_map.end())
  {
    // Return specific defined threshold for this sensor
    return temperature_thresholds_map[sensor_name];
  }
  else
  {
    // Fall back to default if there is no defined threshold
    ROS_WARN_ONCE("There is a specific temperature threshold missing for %s sensor", sensor_name.c_str());
    return this->default_temperature_threshold_;
  }
}

void TemperatureSafety::createSubscribers()
{
  for (const std::string& joint_name : this->joint_names_)
  {
    // Use boost::bind to pass on the sensor_name as extra parameter to the callback method
    ros::Subscriber subscriber_temperature = this->n_->subscribe<sensor_msgs::Temperature>(
        "/march/temperature/" + joint_name, 1000,
        boost::bind(&TemperatureSafety::temperatureCallback, this, _1, joint_name));

    this->temperature_subscribers_.push_back(subscriber_temperature);
  }
}
