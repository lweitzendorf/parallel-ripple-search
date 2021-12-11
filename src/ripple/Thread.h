#pragma once

#include <cstdint>

#define NUM_THREADS 4

enum ThreadId : int8_t {
  THREAD_NONE = -1,
  THREAD_SOURCE = 0,
  THREAD_GOAL = NUM_THREADS - 1
};


