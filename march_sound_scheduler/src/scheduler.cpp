// Copyright 2019 Project March.
#include "march_sound_scheduler/scheduler.h"
#include "march_sound_scheduler/scheduled_sound.h"

Scheduler::Scheduler()
{
}

void Scheduler::schedule(ScheduledSound sound)
{
  this->sound_queue_.push(sound);
}

void Scheduler::scheduleMsg(march_shared_resources::Sound msg)
{
  ROS_DEBUG("Received request to schedule sound %s at time %f", msg.file_name.c_str(), msg.time.toSec());
  this->schedule(ScheduledSound(msg.time, msg.file_name));
}

void Scheduler::spin()
{
  if (this->sound_queue_.empty())
  {
    ROS_DEBUG("Sounds in queue %ld", this->sound_queue_.size());
    return;
  }

  ROS_DEBUG_STREAM("Next sound: " << this->sound_queue_.top());

  if (this->sound_queue_.top().time <= ros::Time::now())
  {
    this->play(this->sound_queue_.top());
    this->sound_queue_.pop();
  }
}

void Scheduler::play(ScheduledSound sound)
{
  this->sc_.playWaveFromPkg("march_sound_scheduler", "sound/" + sound.sound);
}
