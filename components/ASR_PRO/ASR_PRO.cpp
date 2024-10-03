#include "ASR_PRO.h"
#include "esp_log.h"
#include <cstring>

#define UART_BUF_SIZE 256

ASR_PRO::ASR_PRO(int tx_pin, int rx_pin, uart_port_t uart_num_src, QueueHandle_t queue)
{
  uart_num = (uart_port_t)uart_num_src;
  cmd_in_queue = queue; // Store the queue for signaling other tasks

  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  // Install UART driver
  ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 10, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Create a task to handle UART events
  xTaskCreatePinnedToCore(ASR_PRO::uart_event_task, "ASR_uart_event_task", 2000 + configMINIMAL_STACK_SIZE,
                          this, configMAX_PRIORITIES - 10, &uart_event_task_handle, 0);
}

ASR_PRO::~ASR_PRO()
{
  uart_driver_delete(uart_num);
}

void ASR_PRO::uart_event_task(void *pvParameters)
{
  ASR_PRO *asr = (ASR_PRO *)pvParameters;
  uart_event_t event;
  uint8_t dtmp[UART_BUF_SIZE];

  for (; uart_is_driver_installed(asr->uart_num);)
  {
    // Wait for UART event
    if (xQueueReceive(asr->uart_queue, (void *)&event, portMAX_DELAY))
    {
      bzero(dtmp, UART_BUF_SIZE);
      if (event.type == UART_DATA)
      {
        int len;
        do
        {
          len = uart_read_bytes((asr->uart_num), dtmp, 1, pdMS_TO_TICKS(200));
          // ESP_LOGI("ASR_PRO", "Check SOF: 0x%02X", dtmp[0]);
        } while (len == 1 && dtmp[0] != ASR_PRO_SOF);
        if (len != 1)
          continue;

        len = 1 + uart_read_bytes((asr->uart_num), dtmp + 1, sizeof(ASR_PRO_cmd_frame), pdMS_TO_TICKS(200));
        // ESP_LOGI("ASR_PRO", "recieve len: %d data: ", len);
        // for (int i = 0; i < len; i++)
        //   printf("0x%02X", dtmp[i]);
        // printf("\n");
        if (len > 0)
        {
          asr->process_received_data(dtmp, len);
        }
      }
    }
  }
}

void ASR_PRO::process_received_data(uint8_t *data, int len)
{
  ASR_PRO_cmd_frame frame;
  if (len >= sizeof(ASR_PRO_cmd_frame))
  {
    memcpy(&frame, data, sizeof(ASR_PRO_cmd_frame));
    if (frame.check_check_sum())
    {
      // Signal that a command has been received by placing it in the queue
      ASR_PRO_cmd_message msg;
      msg.cmd = frame.cmd;
      msg.data = frame.data;
      // ESP_LOGI("ASR_PRO", "Recieve cmd: %d data: %d", msg.cmd, msg.data);
      xQueueSend(cmd_in_queue, &msg, portMAX_DELAY);
    }
  }
}

void ASR_PRO::send_cmd(ASR_PRO_cmd_Out cmd, uint8_t data)
{
  ASR_PRO_cmd_frame frame(cmd, data);
  uart_write_bytes(uart_num, (const char *)&frame, sizeof(frame));
}
