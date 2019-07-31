// Copyright 2018 Project March.

#ifndef SRC_TRAJECTORYSAFETY_H
#define SRC_TRAJECTORYSAFETY_H

#include "ros/ros.h"
#include "SafetyType.h"
#include "SafetyHandler.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <march_shared_resources/TopicNames.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControllerClient;

class TrajectorySafety : public SafetyType
{
    ros::NodeHandle n;
    SafetyHandler* safety_handler;
    std::vector<std::string> joint_names;
    std::string controller_name;
    std::string controller_path;
    float stop_trajectory_duration;
    std::map< std::string, double> trajectory_tolerances;
    std::map< std::string, double>  position_errors;
    ros::Subscriber trajectory_subscriber;
    ControllerClient *controller_client;

    void trajectoryCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
    void toleranceCheck();
    static std::string getJointName(std::vector<std::string>::iterator it);

public:
    TrajectorySafety(ros::NodeHandle* n, SafetyHandler* safety_handler, std::vector<std::string> joint_names);

    void update() override
    {
    }
};

#endif //SRC_TRAJECTORYSAFETY_H
