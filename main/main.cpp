#include <stdio.h>
#include "ASR_PRO.h"
#include "Differential_Timer.h"
#include "esp_log.h"
#include "DYP_Ultrusonic.h"
#include "AnalogSensors.h"
#include "Fitful_IO.h"
#include "General_Battery.h"
#include "Speed_estimator.h"
#include "Parameters.h"

#define PIN_ASR_RX GPIO_NUM_16
#define PIN_ASR_TX GPIO_NUM_15
#define PIN_DYP_TX GPIO_NUM_17
#define PIN_DYP_RX GPIO_NUM_18
#define PIN_BEEP GPIO_NUM_36

void io_init();
void logic_Task(void *param);
void voice_interface_Task(void *param);
void battery_task(void *param);

General_Battery battery;
const float sensor_mount_pitch[4] = {5, 0, 0, 5};
const float sensor_mount_angle[4] = {-10, 0, 0, 10};
Parameters parameters;
float cur_distance;

extern "C" void app_main(void)
{
    io_init();
    xTaskCreatePinnedToCore(logic_Task, "logic", 1000 + configMINIMAL_STACK_SIZE,
                            nullptr, configMAX_PRIORITIES - 4, NULL, 0);
    xTaskCreatePinnedToCore(voice_interface_Task, "voice_interface", 2000 + configMINIMAL_STACK_SIZE,
                            nullptr, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(battery_task, "battery_task", 2000 + configMINIMAL_STACK_SIZE,
                            nullptr, configMAX_PRIORITIES - 6, NULL, 1);
}
void logic_Task(void *param)
{
    DYP_Ultrusonic distanceSensors(PIN_DYP_TX, PIN_DYP_RX, UART_NUM_2);
    Fitful_IO_PWM beeper(LEDC_CHANNEL_0, 255);
    Speed_estimator speed_estimator;
    // beeper.begin(100, 100, 2);

    for (;;)
    {
        float corrected_distance[4];
        float min_distance = infinityf();
        complimentary_angle_t curr_angle = speed_estimator.get_angles();
        for (int i = 0; i < 4; i++)
        {
            uint16_t dist_tmp = distanceSensors.get_distance(i);
            if (dist_tmp < 5000)
            {
                corrected_distance[i] = dist_tmp * cos(sensor_mount_angle[i]) *
                                        cos(sensor_mount_pitch[i] + curr_angle.pitch);
                if (min_distance > corrected_distance[i])
                    min_distance = corrected_distance[i];
            }
        }
        cur_distance = min_distance;
        float predict_speed = speed_estimator.updateSpeed(min_distance);
        float sensitivity = parameters.sensitivity * predict_speed * SPD_SENSITIVITY_RATIO;
        if (sensitivity > 1)
            sensitivity = 1;

        float alarm_level = 0; // 0~1, 1 being max
        if (parameters.Alarm_sw)
        {
            if (min_distance < ALARM_MIN_DIST)
                alarm_level = 1;
            else if (min_distance < ALARM_MAX_DIST)
                alarm_level = 1 - (min_distance - ALARM_MIN_DIST) / (ALARM_MAX_DIST - ALARM_MIN_DIST);
        }

        ESP_LOGI("Logic", "cur_distance: %0.2f, predict_speed: %0.2f, sensitivity: %0.2f, alarm_level: %0.2f",
                 cur_distance, predict_speed, sensitivity, alarm_level);

        if (alarm_level != 0)
        {
            beeper.update_pulseWidth((BEEPER_MAX_PWM - BEEPER_MIN_PWM) * alarm_level - BEEPER_MIN_PWM);
            if (!beeper.is_busy())
                beeper.begin(BEEPER_ON_TIME, BEEPER_OFF_MIN_TIME + alarm_level * (BEEPER_OFF_MAX_TIME - BEEPER_OFF_MIN_TIME), 1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void voice_interface_Task(void *param)
{
    QueueHandle_t ASR_PRO_msg_queue = xQueueCreate(10, sizeof(ASR_PRO_cmd_message));
    ASR_PRO asr_pro(PIN_ASR_TX, PIN_ASR_RX, UART_NUM_1, ASR_PRO_msg_queue);
    ASR_PRO_cmd_message received_msg;

    bool ASR_is_busy = true;
    for (;;)
    {
        // Wait for a command to be received
        if (xQueueReceive(ASR_PRO_msg_queue, &received_msg, portMAX_DELAY))
        {
            // Handle the received command and data
            ESP_LOGI("app_main", "Received command: %d, Data: %d", received_msg.cmd, received_msg.data);
            switch (received_msg.cmd)
            {
            case ASR_PRO_Ready:
                ASR_is_busy = false;
                break;
            case ASR_PRO_Busy:
                ASR_is_busy = received_msg.data;
                break;
            case ASR_PRO_Battery_Inquiry:
                asr_pro.send_cmd(ASR_PRO_Battery_Answer, battery.get_percentage());
                break;
            case ASR_PRO_Sensitivity_Set:
                parameters.sensitivity = received_msg.data;
                asr_pro.send_cmd(ASR_PRO_Sensitivity_Answer, parameters.sensitivity);
                break;
            case ASR_PRO_Alarm_Switch_Set:
                parameters.Alarm_sw = (received_msg.data != 0);
                asr_pro.send_cmd(ASR_PRO_Alarm_Switch_Answer, parameters.Alarm_sw);
                break;
            case ASR_PRO_Distance_Inquiry:
                asr_pro.send_cmd(ASR_PRO_Distance_Answer, (uint8_t)cur_distance);
                break;
            }
        }
    }
}
void io_init()
{
    gpio_config_t en_gpio_config = {};
    en_gpio_config.mode = GPIO_MODE_OUTPUT;
    en_gpio_config.pin_bit_mask =
        // (1ULL << PIN_POW_LED) |
        // (1ULL << PIN_12V_EN) |
        // (1ULL << PIN_PMOS_SW) |
        // (1ULL << PIN_CAN_EN) |
        (1ULL << PIN_IMU_EN) |
        (1ULL << PIN_BEEP);
    ESP_ERROR_CHECK(gpio_config(&en_gpio_config));
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,   // timer mode
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .timer_num = LEDC_TIMER_0,           // timer index
        .freq_hz = 500,                      //(int)(TIMER_SRC_FREQ / 256 / 1000) * 1000, // frequency of PWM signal
        .clk_cfg = LEDC_USE_APB_CLK,         // select the APB source clock
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .gpio_num = PIN_BEEP,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {false}};
    ledc_channel_config(&ledc_channel);
    gpio_set_level(PIN_IMU_EN, 1);
}

void battery_task(void *param)
{
    AnalogSensors analogSensors;
    Battery_info bat_info;
    bat_info = General_lipo;
    bat_info.capacity = 1500;
    bat_info.serial = 3;
    battery.set_bat_info(bat_info);

    while (analogSensors.get_bat_V() <= 8.1)
        vTaskDelay(pdMS_TO_TICKS(50));

    for (; true;)
    {
        float tmp_battery_volt = analogSensors.get_bat_V();

        battery.heartbeat(analogSensors.get_bat_V());
        // ESP_LOGI("Battery", "bat: %f", battery.get_percentage());

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}