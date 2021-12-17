#pragma once

#include <cstdint>

#define LOG_ENABLED false

#define AssertUnreachable(...)                                                 \
  do {                                                                         \
    LogNOID(__VA_ARGS__);                                                      \
    assert(false);                                                             \
  } while (false)

#if LOG_ENABLED
  #define Log(str) printf("%d|" str "\n", id)
  #define LogNOID(str) printf(str "\n")
  #define Logf(fmt, ...) printf("%d|" fmt "\n", id, __VA_ARGS__)
  #define LogfNOID(fmt, ...) printf(fmt "\n", __VA_ARGS__)
#else
  #define Log(...)
  #define LogNOID(...)
  #define Log(...)
  #define LogfNOID(...)
#endif

#define NUM_THREADS 5
#define NUM_SEARCH_THREADS (NUM_THREADS - 1)
#define NUM_ESSENTIAL_THREADS 2
static_assert(NUM_SEARCH_THREADS >= NUM_ESSENTIAL_THREADS);

enum ThreadId : int8_t {
  THREAD_NONE = -1,
  THREAD_SOURCE = 0,
  THREAD_GOAL = NUM_SEARCH_THREADS - 1,
  THREAD_COORDINATOR = NUM_THREADS - 1
};

enum Phase : bool {
  PHASE_1 = false,
  PHASE_2 = true
};
