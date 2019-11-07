// Copyright 2018 Project March.
#include <string>

#include <ros/ros.h>

#include "march_sound_scheduler/scheduled_sound.h"

ScheduledSound::ScheduledSound(const ros::Time& time, const std::string& sound) : time(time), sound(sound)
{
}