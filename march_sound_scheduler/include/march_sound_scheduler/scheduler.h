// Copyright 2019 Project March.
#ifndef MARCH_SOUND_SCHEDULER_SCHEDULER_H
#define MARCH_SOUND_SCHEDULER_SCHEDULER_H

#include <vector>
#include <queue>

#include <ros/duration.h>
#include <ros/ros.h>

#include <sound_play/sound_play.h>

#include <march_shared_resources/Sound.h>

#include "march_sound_scheduler/scheduled_sound.h"

class Scheduler
{
public:
  Scheduler();

  void schedule(ScheduledSound sound);
  void scheduleMsg(march_shared_resources::Sound msg);

  void spin();

  void play(ScheduledSound sound);

private:
  std::priority_queue<ScheduledSound, std::vector<ScheduledSound>> sound_queue_;
  sound_play::SoundClient sc_;
};

#endif  // MARCH_SOUND_SCHEDULER_SCHEDULER_H
