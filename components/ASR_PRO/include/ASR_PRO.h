#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"

typedef struct
{
  uint8_t cmd;
  uint8_t data;
} ASR_PRO_cmd_message;

typedef enum : uint8_t
{
  ASR_PRO_SYS_Ready = 0,
  ASR_PRO_Speed_Answer,        // Current speed is xx meters per second
  ASR_PRO_Low_Battery,         // Low Battery please charge the device
  ASR_PRO_Battery_Answer,      // Battery xx percent left
  ASR_PRO_Sensitivity_Answer,  // Sensitivity set to xx percent
  ASR_PRO_Alarm_Switch_Answer, // Alarm turned on/off
  ASR_PRO_Alarm_Pause_Answer,  // Alarm paused on/off
  ASR_PRO_Distance_Answer,     // Distance to closist obstacal is xx cm

} ASR_PRO_cmd_Out;

typedef enum : uint8_t
{
  ASR_PRO_Ready = 125,      // Send when ASR is finish initialization
  ASR_PRO_Speed_Inquiry,    // Inquiry the current speed, answered by ASR_PRO_Speed_Answer
  ASR_PRO_Battery_Inquiry,  // Inquiry the battery level, answered by ASR_PRO_Battery_Answer
  ASR_PRO_Sensitivity_Set,  // Set Sensitivity, use "data" to represent sensitivity percentage, answered by ASR_PRO_Sensitivity_Answer
  ASR_PRO_Alarm_Switch_Set, // Turn on alarm, use "data" to represent on/off, answered by ASR_PRO_Alarm_On_Answer/ASR_PRO_Alarm_Off_Answer
  ASR_PRO_Distance_Inquiry  // Inquiry the distance answered by ASR_PRO_Distance_Answer
} ASR_PRO_cmd_In;

#define ASR_PRO_SOF 0xFF
typedef struct __attribute__((packed, aligned(1))) ASR_PRO_cmd_frame
{
  uint8_t SOF = ASR_PRO_SOF;
  uint8_t cmd;
  uint8_t data;
  uint8_t check_sum;
  ASR_PRO_cmd_frame(uint8_t cmd_src = ASR_PRO_Ready, uint8_t data_src = 0)
  {
    cmd = cmd_src;
    data = data_src;
    update_check_sum();
  }
  void update_check_sum()
  {
    check_sum = SOF + cmd + data;
  }
  bool check_check_sum()
  {
    return check_sum == (uint8_t)(SOF + cmd + data);
  }
} ASR_PRO_cmd_frame;

class ASR_PRO
{
private:
  uart_port_t uart_num;
  QueueHandle_t uart_queue;   // For UART events
  QueueHandle_t cmd_in_queue; // For signaling received commands
  TaskHandle_t uart_event_task_handle = nullptr;

  static void uart_event_task(void *pvParameters);
  void process_received_data(uint8_t *data, int len);

public:
  ASR_PRO(int tx_pin, int rx_pin, uart_port_t uart_num_src, QueueHandle_t queue);
  ~ASR_PRO();
  void send_cmd(ASR_PRO_cmd_Out cmd, uint8_t data = 0);
};
