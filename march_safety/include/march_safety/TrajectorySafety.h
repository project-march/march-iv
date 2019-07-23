// Copyright 2018 Project March.

#ifndef SRC_TRAJECTORYSAFETY_H
#define SRC_TRAJECTORYSAFETY_H

#include "ros/ros.h"
#include "SafetyType.h"
#include "SafetyHandler.h"

class TrajectorySafety : public SafetyType
{
    ros::NodeHandle n;
    SafetyHandler* safety_handler;
    std::vector<std::string> joint_names;
    std::vector<double> position_errors ;


public:
    TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler, std::vector<std::string> joint_names);
};

#endif //SRC_TRAJECTORYSAFETY_H
