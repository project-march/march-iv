// Copyright 2019 Project March.
#ifndef MARCH_SOUND_SCHEDULER_SCHEDULEDSOUND_H
#define MARCH_SOUND_SCHEDULER_SCHEDULEDSOUND_H

#include <ros/duration.h>
#include <ros/ros.h>

class ScheduledSound
{
public:
  ros::Time time;
  std::string sound;

  ScheduledSound(const ros::Time& time, const std::string& sound);

  friend bool operator<(const ScheduledSound& lhs, const ScheduledSound& rhs)
  {
    return lhs.time > rhs.time;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const ScheduledSound& scheduledSound)
  {
    return os << scheduledSound.sound << " in " << (scheduledSound.time - ros::Time::now()) << "s";
  }
};

#endif  // MARCH_SOUND_SCHEDULER_SCHEDULEDSOUND_H
