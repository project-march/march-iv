// Copyright 2018 Project March.

#ifndef SRC_TRAJECTORYSAFETY_H
#define SRC_TRAJECTORYSAFETY_H

#include "ros/ros.h"
#include "SafetyType.h"
#include "SafetyHandler.h"
#include "control_msgs/JointTrajectoryControllerState.h"

#include <march_shared_resources/TopicNames.h>

class TrajectorySafety : public SafetyType
{
    ros::NodeHandle n;
    SafetyHandler* safety_handler;
    std::vector<std::string> joint_names;
    std::string controller_name;
    std::string controller_path;
    std::map< std::string, double> trajectory_tolerances;
    std::map< std::string, double>  position_errors ;
    ros::Subscriber trajectory_subscriber;

    void trajectoryCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
    void toleranceCheck();
    static std::string getJointName(std::vector<std::string>::iterator it);

public:
    TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler, std::vector<std::string> joint_names);
};

#endif //SRC_TRAJECTORYSAFETY_H
