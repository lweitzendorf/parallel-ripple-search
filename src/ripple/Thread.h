#pragma once

#include <cstdint>

#define NUM_THREADS 5
#define NUM_SEARCH_THREADS (NUM_THREADS - 1)

enum ThreadId : int8_t {
  THREAD_NONE = -1,
  THREAD_SOURCE = 0,
  THREAD_GOAL = NUM_SEARCH_THREADS - 1,
  THREAD_COORDINATOR = NUM_THREADS - 1
};
