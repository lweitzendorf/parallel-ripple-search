#include "Timer.h"

void Timer::start() {
  t1 = std::chrono::high_resolution_clock::now();
}

void Timer::stop() {
  t2 = std::chrono::high_resolution_clock::now();
}

long Timer::get_milliseconds() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
}

long Timer::get_microseconds() {
  return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
}


