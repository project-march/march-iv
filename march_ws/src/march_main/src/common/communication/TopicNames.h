// Copyright 2018 Project March.
#ifndef MARCH_MAIN_TOPICNAMES_H
#define MARCH_MAIN_TOPICNAMES_H

#include <string>
namespace TopicNames
{
const char* const right_ankle_position = "march/right_ankle_position_controller/command";
const char* const gait_status = "gait/status";
const char* const left_hip_position = "march/left_hip_position_controller/command";
const char* const left_knee_position = "march/left_knee_position_controller/command";
const char* const left_ankle_position = "march/left_ankle_position_controller/command";
const char* const right_hip_position = "march/right_hip_position_controller/command";
const char* const right_knee_position = "march/right_knee_position_controller/command";

const char* const perform_gait = "march/perform_gait";

const char* const input_device_gait = "input_device/instruction/gait";
const char* const input_device_stop = "input_device/instruction/stop";
const char* const input_device_trigger = "input_device/instruction/trigger";

const char* const input_device_gait_done = "input_device/done/gait";
const char* const input_device_gait_performing= "input_device/performing/gait";

};  // namespace TopicNames

namespace ServiceNames
{
const char* const urdf_validation = "march/urdf_validation";
const char* const config_validation = "march/config_validation";
const char* const xml_validation = "march/xml_validation";

const char* const request_gait_file = "march/request_gait_file";
};

#endif  // MARCH_MAIN_TOPICNAMES_H
