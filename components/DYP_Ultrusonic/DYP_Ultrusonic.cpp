#include "DYP_Ultrusonic.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "DYP_Ultrusonic";
#define UART_BUFFER_SIZE (1024)
#define UART_BAUD_RATE (9600)
#define UART_EVENT_QUEUE_SIZE (10)

DYP_Ultrusonic::DYP_Ultrusonic(int tx_pin, int rx_pin, int uart_num_src)
{
  uart_num = (uart_port_t)uart_num_src;
  uart_config_t uart_config = {
      .baud_rate = UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  uart_param_config(uart_num, &uart_config);
  uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, UART_EVENT_QUEUE_SIZE, &uart_queue, 0);
  xTaskCreate(uart_event_task, "uart_event_task", 2048, this, 10, NULL);
}
DYP_Ultrusonic::~DYP_Ultrusonic()
{
  uart_driver_delete(uart_num);
}

void DYP_Ultrusonic::uart_event_task(void *pvParameters)
{
  DYP_Ultrusonic *ultrasonic = static_cast<DYP_Ultrusonic *>(pvParameters);
  uart_event_t event;
  uint8_t *data = (uint8_t *)malloc(UART_BUFFER_SIZE);

  for (;;)
  {
    if (xQueueReceive(ultrasonic->uart_queue, (void *)&event, portMAX_DELAY))
    {
      if (event.type == UART_DATA)
      {
        int len = uart_read_bytes(ultrasonic->uart_num, data, sizeof(DYP_UART_frame), pdMS_TO_TICKS(500));
        if (len == sizeof(DYP_UART_frame)) // 确保数据长度正确
        {
          DYP_UART_frame frame;
          memcpy(&frame, data, len); // 将数据复制到frame中

          if (frame.check_checksum())
          {
            for (int i = 0; i < 4; i++)
            {
              ultrasonic->distance[i] = frame.get_sensor_distance(i);
            }
            ESP_LOGI(TAG, "Sensor 1: %d mm, Sensor 2: %d mm, Sensor 3: %d mm, Sensor 4: %d mm",
                     ultrasonic->distance[0], ultrasonic->distance[1],
                     ultrasonic->distance[2], ultrasonic->distance[3]);
          }
          else
          {
            ESP_LOGW(TAG, "Invalid checksum");
          }
        }
        else
        {
          ESP_LOGW(TAG, "Invalid frame length");
        }
      }
    }
  }
  free(data);
  vTaskDelete(NULL);
}

uint16_t DYP_Ultrusonic::get_distance(uint8_t sensor_index)
{
  if (sensor_index < 4)
    return distance[sensor_index];
  else
    return 0xFFFF;
}
