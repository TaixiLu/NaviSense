#pragma once
#include <queue>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

typedef struct
{
  uint32_t on_delay = 0;
  uint32_t off_delay = 0;
} stuct_io_seq;

class Fitful_IO_base
{
public:
  Fitful_IO_base();
  void begin(uint32_t on_delay, uint32_t off_delay, uint8_t count_down);
  void add(uint32_t on_delay, uint32_t off_delay, uint8_t count_down);
  bool is_busy();

  virtual void set_state(bool set_on) {};
  virtual bool get_state() { return false; };

private:
  std::queue<stuct_io_seq> state_queue;
  SemaphoreHandle_t heartbeat_Semaphore = NULL;
  TaskHandle_t heartbeat_task_handle = nullptr;
  static void heartbeat(void *param);
};

class Fitful_IO : public Fitful_IO_base
{
public:
  Fitful_IO(gpio_num_t GPIOx, bool level_src)
      : Fitful_IO_base()
  {
    gpio_num = GPIOx;
    level_on = level_src;
  }

  void set_state(bool set_on) override
  {
    if (set_on)
      gpio_set_level(gpio_num, level_on);
    else
      gpio_set_level(gpio_num, !level_on);
  };
  bool get_state() override
  {
    return gpio_get_level(gpio_num);
  };

private:
  gpio_num_t gpio_num;
  bool level_on;
};

class Fitful_IO_PWM : public Fitful_IO_base
{
public:
  Fitful_IO_PWM(ledc_channel_t pwm_ch_src, uint16_t src_Pulse)
      : Fitful_IO_base()
  {
    pwm_ch = pwm_ch_src;
    pulse_width = src_Pulse;
  }
  void update_pulseWidth(uint16_t src_Pulse, bool force_on = false)
  {
    if (src_Pulse > 255)
      src_Pulse = 255;
    pulse_width = src_Pulse;
    // ESP_LOGI("update_pulseWidth", "force_on: %d, ledc_get_duty: %d",
    //          force_on, (int)ledc_get_duty(LEDC_LOW_SPEED_MODE, pwm_ch));
    if (force_on || get_state())
      set_state(true);
  }

  void set_state(bool set_on) override
  {
    if (set_on)
      ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_ch, pulse_width);
    else
      ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_ch, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_ch);
  };
  bool get_state() override
  {
    return ledc_get_duty(LEDC_LOW_SPEED_MODE, pwm_ch) != 0;
  };

private:
  ledc_channel_t pwm_ch;
  uint16_t pulse_width = 255;
};
