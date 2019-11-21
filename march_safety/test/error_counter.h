// Copyright 2019 Project March.
#include <march_shared_resources/Error.h>

struct ErrorCounter
{
  ErrorCounter() : count(0)
  {
  }

  void cb(const march_shared_resources::Error&)
  {
    ++count;
  }

  size_t count;
};