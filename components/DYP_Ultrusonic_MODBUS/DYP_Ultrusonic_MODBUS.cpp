#include "DYP_Ultrusonic_MODBUS.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "DYP_Ultrusonic_MODBUS";
#define UART_BUFFER_SIZE (1024)
#define UART_BAUD_RATE (9600)
#define UART_EVENT_QUEUE_SIZE (10)

uint16_t calculate_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF; // 初始值
  for (uint16_t i = 0; i < length; i++)
  {
    // printf("0x%02X ", data[i]);
    crc ^= data[i]; // 按字节异或
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001; // 多项式 0xA001
      else
        crc = crc >> 1;
    }
  }

  // 交换高位和低位
  // uint16_t swapped_crc = (crc >> 8) | (crc << 8);
  // printf("CRC16: 0x%04X\n", swapped_crc);
  return crc;
}

DYP_Ultrusonic_MODBUS::DYP_Ultrusonic_MODBUS(int tx_pin, int rx_pin, int uart_num_src)
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
  xTaskCreatePinnedToCore(DYP_Ultrusonic_MODBUS::uart_event_task, "DYP_uart_event_task", 2000 + configMINIMAL_STACK_SIZE,
                          this, configMAX_PRIORITIES - 1, &uart_event_task_handle, 1);
}
DYP_Ultrusonic_MODBUS::~DYP_Ultrusonic_MODBUS()
{
  uart_driver_delete(uart_num);
}

void DYP_Ultrusonic_MODBUS::uart_event_task(void *pvParameters)
{
  DYP_Ultrusonic_MODBUS *DYP = static_cast<DYP_Ultrusonic_MODBUS *>(pvParameters);
  uart_event_t event;
  uint8_t *data = (uint8_t *)malloc(UART_BUFFER_SIZE);

  DYP_MODBUS_Frame_Master_Out curr_out_frame(DYP_REG_ADDR_SENS1);
  for (; uart_is_driver_installed(DYP->uart_num);)
  {
    uart_write_bytes(DYP->uart_num, (const char *)&curr_out_frame, sizeof(DYP_MODBUS_Frame_Master_Out));
    // ESP_LOGI(TAG, "curr_out_frame 0x%02X 0x%02X 0x%04X 0x%02X%02X CRC 0x%04X",
    //          curr_out_frame.addr, curr_out_frame.cmd,
    //          curr_out_frame.get_reg_addr(),
    //          curr_out_frame.num_reg_to_read_H, curr_out_frame.num_reg_to_read_L,
    //          curr_out_frame.crc16);
    // printf("sending frame: ");
    // for (int i = 0; i < sizeof(DYP_MODBUS_Frame_Master_Out); i++)
    //   printf("0x%02X ", ((const char *)&curr_out_frame)[i]);
    // printf("\n");

    if (xQueueReceive(DYP->uart_queue, (void *)&event, pdMS_TO_TICKS(200)))
    {
      if (event.type == UART_DATA)
      {
        do
        {
          if (uart_read_bytes(DYP->uart_num, data, 1, pdMS_TO_TICKS(50)) <= 0)
            break;
        } while (data[0] != curr_out_frame.addr);
        if (data[0] != curr_out_frame.addr)
          continue;

        int len = 1 + uart_read_bytes(DYP->uart_num, data + 1,
                                      sizeof(DYP_MODBUS_Frame_Master_In_Single_Sens) - 1, pdMS_TO_TICKS(50));
        if (len == sizeof(DYP_MODBUS_Frame_Master_In_Single_Sens)) // 确保数据长度正确
        {
          DYP_MODBUS_Frame_Master_In_Single_Sens in_frame;
          memcpy(&in_frame, data, sizeof(DYP_MODBUS_Frame_Master_In_Single_Sens)); // 将数据复制到frame中

          if (in_frame.check_CRC())
          {
            DYP->distance[curr_out_frame.get_sens_index()] = in_frame.get_data();
            // ESP_LOGI(TAG, "refresh time %d", (int)DYP->recv_Timer.elapsed());
            DYP->recv_Timer.start();
            DYP->new_data = true;
            // printf("Sensor %d: %02.3f mm",
            //        curr_out_frame.get_sens_index(), DYP->distance[curr_out_frame.get_sens_index()] / 1000.0);
            // if (curr_out_frame.get_sens_index() == 3)
            //   printf("\n");
            curr_out_frame.switch_sens();
          }
          else
          {
            ESP_LOGW(TAG, "Invalid crc 0x%04X vs 0x%04X", in_frame.crc16,
                     calculate_CRC16((uint8_t *)&in_frame, sizeof(DYP_MODBUS_Frame_Master_Out) - sizeof(uint16_t)));
          }
        }
        else
          ESP_LOGW(TAG, "Invalid frame length %d vs %d", len, sizeof(DYP_MODBUS_Frame_Master_In_Single_Sens));
      }
    }
    // else
    //   ESP_LOGW(TAG, "Recv overtime");
  }
  free(data);
  vTaskDelete(NULL);
}

uint16_t DYP_Ultrusonic_MODBUS::get_distance(uint8_t sensor_index)
{
  new_data = false;
  if (sensor_index < 4 && recv_Timer.elapsed() < 500)
    return distance[sensor_index];
  else
    return 0xFFFF;
}
