// Copyright 2018 Project March.
#ifndef SRC_INPUT_DEVICE_MAIN_H_
#define SRC_INPUT_DEVICE_MAIN_H_

#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/StepSize.h>

class main {

  void gaitDoneCallback(const march_custom_msgs::Gait msg);

  void gaitPerformingCallback(const march_custom_msgs::Gait msg);
};

#endif  // SRC_INPUT_DEVICE_MAIN_H_
