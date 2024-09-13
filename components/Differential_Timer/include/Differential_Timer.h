#pragma once
#include <stdio.h>

#define TIMER_MICRO_DEFAULT 0xFFFFFFFFFFFFFFFF // max for U64
#define TIMER_MILLI_DEFAULT 0xFFFFFFFF         // max for U32

class Timer_micro
{
public:
  void start();
  uint64_t elapsed();
  uint64_t stop();
  void clear();

private:
  uint64_t timeStamp = 0;
  uint64_t timeElapsed = 0;
};

class Timer_milli
{
public:
  void start() { micro_timer.start(); }
  uint32_t elapsed()
  {
    uint64_t micro_sec = micro_timer.elapsed();
    if (micro_sec == TIMER_MICRO_DEFAULT)
      return TIMER_MILLI_DEFAULT;
    return micro_sec / 1000;
  }
  uint32_t stop()
  {
    uint64_t micro_sec = micro_timer.stop();
    if (micro_sec == TIMER_MICRO_DEFAULT)
      return TIMER_MILLI_DEFAULT;
    return micro_sec / 1000;
  }
  void clear() { micro_timer.clear(); }

private:
  Timer_micro micro_timer;
};