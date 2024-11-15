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
  xTaskCreatePinnedToCore(DYP_Ultrusonic::uart_event_task, "DYP_uart_event_task", 2000 + configMINIMAL_STACK_SIZE,
                          this, configMAX_PRIORITIES - 1, &uart_event_task_handle, 1);
}
DYP_Ultrusonic::~DYP_Ultrusonic()
{
  uart_driver_delete(uart_num);
}

void DYP_Ultrusonic::uart_event_task(void *pvParameters)
{
  DYP_Ultrusonic *DYP = static_cast<DYP_Ultrusonic *>(pvParameters);
  uart_event_t event;
  uint8_t *data = (uint8_t *)malloc(UART_BUFFER_SIZE);

  for (; uart_is_driver_installed(DYP->uart_num);)
  {
    if (xQueueReceive(DYP->uart_queue, (void *)&event, portMAX_DELAY))
    {
      if (event.type == UART_DATA)
      {
        do
        {
          if (0 == uart_read_bytes(DYP->uart_num, data, 1, pdMS_TO_TICKS(50)))
            continue;
        } while (data[0] != DYP_UART_SOF);

        int len = uart_read_bytes(DYP->uart_num, data + 1, sizeof(DYP_UART_frame) - 1, pdMS_TO_TICKS(50));
        if (len + 1 == sizeof(DYP_UART_frame)) // 确保数据长度正确
        {
          DYP_UART_frame frame;
          memcpy(&frame, data, sizeof(DYP_UART_frame)); // 将数据复制到frame中

          if (frame.SOF != DYP_UART_SOF)
          {
            ESP_LOGW(TAG, "Invalid SOF");
            continue;
          }
          if (frame.check_checksum())
          {
            for (int i = 0; i < 4; i++)
            {
              DYP->distance[i] = frame.get_sensor_distance(i);
            }
            ESP_LOGI(TAG, "refresh time %d", (int)DYP->recv_Timer.elapsed());
            DYP->recv_Timer.start();
            DYP->new_data = true;
            // ESP_LOGI(TAG, "Sensor 1: %d mm, Sensor 2: %d mm, Sensor 3: %d mm, Sensor 4: %d mm",
            //          DYP->distance[0], DYP->distance[1],
            //          DYP->distance[2], DYP->distance[3]);
          }
          else
          {
            ESP_LOGW(TAG, "Invalid checksum 0x%02X vs 0x%02X", frame.checksum,
                     (uint8_t)(frame.SOF + frame.sensor1_high + frame.sensor1_low + frame.sensor2_high +
                               frame.sensor2_low + frame.sensor3_high + frame.sensor3_low + frame.sensor4_high + frame.sensor4_low));
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
  new_data = false;
  if (sensor_index < 4 && recv_Timer.elapsed() < 500)
    return distance[sensor_index];
  else
    return 0xFFFF;
}
