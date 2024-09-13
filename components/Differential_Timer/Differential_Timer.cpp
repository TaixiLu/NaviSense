#include "Differential_Timer.h"
#include "esp_timer.h"

void Timer_micro::start()
{
  timeStamp = (uint64_t)esp_timer_get_time();
  if (timeStamp == 0)
    timeStamp = 1;
}
uint64_t Timer_micro::elapsed()
{
  if (timeStamp == 0)
  {
    if (timeElapsed == 0)
      return TIMER_MICRO_DEFAULT;
    else
      return timeElapsed;
  }
  if ((uint64_t)esp_timer_get_time() < timeStamp)
    return TIMER_MICRO_DEFAULT - timeStamp + (uint64_t)esp_timer_get_time();
  else
    return (uint64_t)esp_timer_get_time() - timeStamp;
}
uint64_t Timer_micro::stop()
{
  timeElapsed = elapsed();
  timeStamp = 0;
  return timeElapsed;
}

void Timer_micro::clear()
{
  timeStamp = 0;
  timeElapsed = 0;
}
