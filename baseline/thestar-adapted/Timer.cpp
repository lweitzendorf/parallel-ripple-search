#include "Timer.h"

void Timer::start() {
  t1 = std::chrono::high_resolution_clock::now();
}

void Timer::stop() {
  t2 = std::chrono::high_resolution_clock::now();
}

long Timer::get_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
}

