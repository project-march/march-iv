// Copyright 2019 Project March.
#include <march_safety/TemperatureSafety.h>

// TODO(@Tim) Throw an exception when no temperatures are published.

TemperatureSafety::TemperatureSafety(ros::NodeHandle* n, SafetyHandler* safety_handler,
                                     std::vector<std::string> joint_names)
  : joint_names(joint_names)
{
  n->getParam(ros::this_node::getName() + std::string("/default_temperature_threshold"), default_temperature_threshold);
  n->getParam(ros::this_node::getName() + "/temperature_thresholds_warning", warning_temperature_thresholds_map);
  n->getParam(ros::this_node::getName() + "/temperature_thresholds_non_fatal", non_fatal_temperature_thresholds_map);
  n->getParam(ros::this_node::getName() + "/temperature_thresholds_fatal", fatal_temperature_thresholds_map);
  double send_errors_interval_param;
  n->getParam(ros::this_node::getName() + std::string("/send_errors_interval"), send_errors_interval_param);
  this->send_errors_interval = send_errors_interval_param;
  this->time_last_send_error = ros::Time(0);
  this->safety_handler = safety_handler;

  this->createSubscribers();
}

void TemperatureSafety::temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name)
{
  // send at most an error every second
  if (ros::Time::now() <= (time_last_send_error + ros::Duration(this->send_errors_interval / 1000)))
  {
    return;
  }

  double temperature = msg->temperature;
  if (temperature <= getThreshold(sensor_name, warning_temperature_thresholds_map))
  {
    return;
  }

  std::string error_message = getErrorMessage(temperature, sensor_name);

  // TODO(Olav) this is a temporary fix, this should be fixed locally on the slaves ask Electro.
  if (temperature > 2000)
  {
    ROS_WARN("%s", error_message.c_str());
    return;
  }

  // If the threshold is exceeded raise an error
  if (temperature > getThreshold(sensor_name, fatal_temperature_thresholds_map))
  {
    safety_handler->publishFatal(error_message);
  }
  else if (temperature > getThreshold(sensor_name, non_fatal_temperature_thresholds_map))
  {
    safety_handler->publishNonFatal(error_message);
  }
  else if (temperature > getThreshold(sensor_name, warning_temperature_thresholds_map))
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
    return default_temperature_threshold;
  }
}

void TemperatureSafety::createSubscribers()
{
  for (const std::string& joint_name : joint_names)
  {
    // Use boost::bind to pass on the sensor_name as extra parameter to the callback method
    ros::Subscriber subscriber_temperature = n.subscribe<sensor_msgs::Temperature>(
        "march/temperature/" + joint_name, 1000,
        boost::bind(&TemperatureSafety::temperatureCallback, this, _1, joint_name));

    temperature_subscribers.push_back(subscriber_temperature);
  }
}
