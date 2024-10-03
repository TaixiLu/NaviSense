#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "DYP_Frame_MODBUS.h"
#include "Differential_Timer.h"

class DYP_Ultrusonic_MODBUS
{
private:
  uart_port_t uart_num;
  QueueHandle_t uart_queue;
  uint16_t distance[4];
  TaskHandle_t uart_event_task_handle = nullptr;
  Timer_milli recv_Timer;
  bool new_data = false;

  static void uart_event_task(void *pvParameters);

public:
  DYP_Ultrusonic_MODBUS(int tx_pin, int rx_pin, int uart_num_src);
  ~DYP_Ultrusonic_MODBUS();
  uint16_t get_distance(uint8_t sensor_index);
  bool have_new_data() { return new_data; };
  uint32_t time_from_last_data() { return recv_Timer.elapsed(); };
};
