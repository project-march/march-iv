// Copyright 2019 Project March.
#include "march_safety/SafetyHandler.h"

SafetyHandler::SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher, ros::Publisher* sound_publisher,
                             ros::Publisher* stop_trajectory_publisher)
  : n(n)
  , error_publisher(error_publisher)
  , sound_publisher(sound_publisher)
  , stop_trajectory_publisher(stop_trajectory_publisher)
{
}

void SafetyHandler::publishErrorMessage(const std::string& message, int8_t error_type) const
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  error_msg.error_message = message;
  error_msg.type = error_type;
  error_publisher->publish(error_msg);
}

void SafetyHandler::publishErrorSound(int8_t error_type) const
{
  march_shared_resources::Sound sound;
  sound.time = ros::Time::now();
  if (error_type == march_shared_resources::Error::FATAL)
  {
    sound.file_name = "fatal.wav";
  }
  else if (error_type == march_shared_resources::Error::NON_FATAL)
  {
    sound.file_name = "non-fatal.wav";
  }
  sound_publisher->publish(sound);
}

void SafetyHandler::publishFatal(std::string message)
{
  ROS_ERROR("%s", message.c_str());

  publishErrorMessage(message, march_shared_resources::Error::FATAL);

  publishErrorSound(march_shared_resources::Error::FATAL);
}

void SafetyHandler::publishNonFatal(std::string message)
{
  ROS_ERROR("%s", message.c_str());

  publishErrorMessage(message, march_shared_resources::Error::NON_FATAL);

  publishErrorSound(march_shared_resources::Error::NON_FATAL);
}

void SafetyHandler::publishStopTrajectory()
{
  empty_trajectory.points.clear();
  empty_trajectory.joint_names.clear();
  stop_trajectory_publisher->publish(empty_trajectory);
}

bool SafetyHandler::stopController(const std::string& stop_controller, float stop_trajectory_duration)
{
  // stopping the controller will execute one last update() call once. Depending on the interface either the position
  // command or the effort command will afterwards stay constant.
  ros::ServiceClient client = this->n->serviceClient<controller_manager_msgs::SwitchController>(
      ros::this_node::getNamespace() + "/controller_manager/switch_controller");
  controller_manager_msgs::SwitchController ctr = controller_manager_msgs::SwitchController();

  ROS_INFO("service name: %s", client.getService().c_str());

  ctr.request.stop_controllers.clear();
  ctr.request.start_controllers.clear();
  ctr.request.stop_controllers.push_back(stop_controller);
  ctr.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

  // sleep to give trajectory controller time to register ABORT message to any client server connected
  ros::Duration(0.05).sleep();

    //publish empty trajectory to induce position hold mode and wait for the time specified needed to enter the mode
  publishStopTrajectory();
  ros::Duration(stop_trajectory_duration).sleep();

  while (!client.exists())
  {
    ROS_INFO("waiting for controller manager service");
      ros::Duration(0.1).sleep();;
  }

  while (!client.call(ctr))
  {
    publishStopTrajectory();
  }
  ROS_INFO("the %s controller has been stopped", stop_controller.c_str());
  return true;
}