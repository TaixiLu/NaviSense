#include "ASR_PRO.h"
#include "esp_log.h"
#include <cstring>

#define UART_BUF_SIZE (4 * sizeof(ASR_PRO_cmd_frame))

ASR_PRO::ASR_PRO(int tx_pin, int rx_pin, int uart_num_src, QueueHandle_t queue)
{
  uart_num = uart_num_src;
  cmd_in_queue = queue; // Store the queue for signaling other tasks

  const uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  // Install UART driver
  ESP_ERROR_CHECK(uart_driver_install((uart_port_t)uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 10, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config((uart_port_t)uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Create a task to handle UART events
  xTaskCreate(uart_event_task, "uart_event_task", 2048, this, 12, NULL);
}

ASR_PRO::~ASR_PRO()
{
  uart_driver_delete((uart_port_t)uart_num);
}

void ASR_PRO::uart_event_task(void *pvParameters)
{
  ASR_PRO *asr = (ASR_PRO *)pvParameters;
  uart_event_t event;
  uint8_t dtmp[UART_BUF_SIZE];

  while (true)
  {
    // Wait for UART event
    if (xQueueReceive(asr->uart_queue, (void *)&event, portMAX_DELAY))
    {
      bzero(dtmp, UART_BUF_SIZE);
      if (event.type == UART_DATA)
      {
        int len = uart_read_bytes((uart_port_t)(asr->uart_num), dtmp, sizeof(ASR_PRO_cmd_frame), pdMS_TO_TICKS(200));
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
      xQueueSend(cmd_in_queue, &msg, portMAX_DELAY);
    }
  }
}

void ASR_PRO::send_cmd(ASR_PRO_cmd_Out cmd, uint8_t data)
{
  ASR_PRO_cmd_frame frame(cmd, data);
  uart_write_bytes((uart_port_t)uart_num, (const char *)&frame, sizeof(frame));
}
