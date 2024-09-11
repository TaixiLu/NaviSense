#include <stdio.h>
#include "ASR_PRO.h"
#include "ICM42688.h"
#include "Timer.h"
#include "esp_log.h"

extern "C" void app_main(void)
{
    QueueHandle_t ASR_PRO_msg_queue = xQueueCreate(10, sizeof(ASR_PRO_cmd_message));

    ASR_PRO asr_pro(17, 16, UART_NUM_1, ASR_PRO_msg_queue);
    ASR_PRO_cmd_message received_msg;
    while (true)
    {
        // Wait for a command to be received
        if (xQueueReceive(ASR_PRO_msg_queue, &received_msg, portMAX_DELAY))
        {
            // Handle the received command and data
            ESP_LOGI("app_main", "Received command: %d, Data: %d", received_msg.cmd, received_msg.data);
        }
    }
}
