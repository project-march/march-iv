// Copyright 2019 Project March.
#include <march_sound_scheduler/Scheduler.h>
#include <march_sound_scheduler/ScheduledSound.h>
#include <ros/ros.h>

Scheduler::Scheduler()
{
}

void Scheduler::schedule(ScheduledSound sound)
{
  this->soundQueue.push(sound);
}

void Scheduler::scheduleMsg(march_shared_resources::Sound msg)
{
  ROS_DEBUG("Received request to schedule sound %s at time %f", msg.file_name.c_str(), msg.time.toSec());
  this->schedule(ScheduledSound(msg.time, msg.file_name));
}

void Scheduler::spin()
{
  if (this->soundQueue.empty())
  {
    ROS_DEBUG("Sounds in queue %ld", this->soundQueue.size());
    return;
  }

  ROS_DEBUG_STREAM("Next sound: " << this->soundQueue.top());

  if (this->soundQueue.top().time <= ros::Time::now())
  {
    this->play(this->soundQueue.top());
    this->soundQueue.pop();
  }
}

void Scheduler::play(ScheduledSound sound)
{
  sc.playWaveFromPkg("march_sound_scheduler", "sound/" + sound.sound);
}
