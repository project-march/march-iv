// Copyright 2018 Project March.

#ifndef PROJECT_LAUNCH_API_H
#define PROJECT_LAUNCH_API_H

#include <march_custom_msgs/Trigger.h>

class LaunchAPI
{
public:
  /**
   * Returns true if the config is valid.
   * @srv march_custom_msgs/Trigger.srv
   * srv.success is true iff the config is valid.
  */
  static bool urdf_validator(march_custom_msgs::Trigger::Request& request,
                             march_custom_msgs::Trigger::Response& response);

  static bool config_validator(march_custom_msgs::Trigger::Request& request,
                               march_custom_msgs::Trigger::Response& response);

  static bool xml_validator(march_custom_msgs::Trigger::Request& request,
                            march_custom_msgs::Trigger::Response& response);
};

#endif  // PROJECT_LAUNCH_API_H
