#ifndef DPHPC_TIMER_H
#define DPHPC_TIMER_H

#include <chrono>

class Timer {
  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> t1;
    std::chrono::time_point<std::chrono::high_resolution_clock> t2;

  public:
    void start();
    void stop();
    long get_milliseconds();
    long get_microseconds();
};

#endif //DPHPC_TIMER_H
