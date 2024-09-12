#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "DYP_Frame.h"

class DYP_Ultrusonic
{
private:
  uart_port_t uart_num;
  QueueHandle_t uart_queue;
  uint16_t distance[4];

  static void uart_event_task(void *pvParameters);
  void process_received_data(uint8_t *data, int len);

public:
  DYP_Ultrusonic(int tx_pin, int rx_pin, int uart_num_src);
  ~DYP_Ultrusonic();
  uint16_t get_distance(uint8_t sensor_index);
};
