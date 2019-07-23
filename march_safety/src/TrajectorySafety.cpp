// Copyright 2019 Project March.

#include "march_safety/TrajectorySafety.h"

TrajectorySafety::TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler,
                                   std::vector<std::string> joint_names)
        : safety_handler(safety_handler), joint_names(std::move(joint_names))
{

}