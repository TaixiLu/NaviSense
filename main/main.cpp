#include <stdio.h>
#include "ASR_PRO.h"
#include "Differential_Timer.h"
#include "esp_log.h"
#include "DYP_Ultrusonic_MODBUS.h"
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
float corrected_distance[4];
float cur_distance;
bool ASR_is_ready = false;
float predict_speed = 0;
QueueHandle_t ASR_PRO_msg_queue = xQueueCreate(10, sizeof(ASR_PRO_cmd_message));
ASR_PRO asr_pro(PIN_ASR_TX, PIN_ASR_RX, UART_NUM_1, ASR_PRO_msg_queue);

extern "C" void app_main(void)
{
    io_init();
    xTaskCreatePinnedToCore(logic_Task, "logic", 3000 + configMINIMAL_STACK_SIZE,
                            nullptr, configMAX_PRIORITIES - 4, NULL, 0);
    xTaskCreatePinnedToCore(voice_interface_Task, "voice_interface", 2000 + configMINIMAL_STACK_SIZE,
                            nullptr, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(battery_task, "battery_task", 2000 + configMINIMAL_STACK_SIZE,
                            nullptr, configMAX_PRIORITIES - 6, NULL, 1);
}
void logic_Task(void *param)
{
    // DYP_Ultrusonic distanceSensors(PIN_DYP_TX, PIN_DYP_RX, UART_NUM_2);
    DYP_Ultrusonic_MODBUS distanceSensors(PIN_DYP_TX, PIN_DYP_RX, UART_NUM_2);
    Fitful_IO_PWM beeper(LEDC_CHANNEL_0, 255);
    Speed_estimator speed_estimator;
    beeper.begin(100, 100, 2);
    // for (; !ASR_is_ready;)
    //     vTaskDelay(pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (; !distanceSensors.have_new_data();)
        vTaskDelay(pdMS_TO_TICKS(50));
    asr_pro.send_cmd(ASR_PRO_SYS_Ready);

    ESP_LOGI("Logic", "System Ready!!!!!!!!!!");
    Timer_milli dist_unavailable_alarm_timer;
    Timer_milli alarm_sw_voice_timer;
    bool curr_alarm_sw = true;
    bool param_alarm_sw_buf = true;
    for (;;)
    {
        complimentary_angle_t curr_angle = speed_estimator.get_angles();
        bool angle_ctl_alarm_sw = false;
        if (fabs(curr_angle.pitch + 90) < 45 && (fabs(curr_angle.roll) - 180) < 30)
            angle_ctl_alarm_sw = true;

        if (distanceSensors.have_new_data())
        {
            float min_distance = infinityf();
            for (int i = 0; i < 4; i++)
            {
                uint16_t dist_tmp = distanceSensors.get_distance(i);
                if (dist_tmp != 0) // 0 is undetectable
                {
                    if (angle_ctl_alarm_sw)
                        corrected_distance[i] = fabs(dist_tmp * cos(sensor_mount_angle[i] / RAD_TO_DEG) *
                                                     cos((sensor_mount_pitch[i] + curr_angle.pitch - 90) / RAD_TO_DEG));
                    else
                        corrected_distance[i] = dist_tmp;
                    if (min_distance > corrected_distance[i])
                        min_distance = corrected_distance[i];
                }
                else
                    corrected_distance[i] = infinityf();
            }
            // ESP_LOGI("Logic", "min_distance:%0.2f mm  Corrected dist 1: %0.0f mm, dist 2: %0.0f mm, dist 3: %0.0f mm, dist 4: %0.0f mm",
            //          min_distance,
            //          corrected_distance[0], corrected_distance[1],
            //          corrected_distance[2], corrected_distance[3]);
            cur_distance = min_distance;
            speed_estimator.set_distance(cur_distance);
        }
        if (distanceSensors.time_from_last_data() > 300)
        {
            cur_distance = 0;
            speed_estimator.set_distance(cur_distance);
            speed_estimator.set_INS_speed(0);
        }

        if (cur_distance <= 100 && dist_unavailable_alarm_timer.elapsed() > 10000)
        {
            asr_pro.send_cmd(ASR_PRO_Distance_Answer, 0);
            dist_unavailable_alarm_timer.start();
        }

        float curr_sensitivity = 0;
        predict_speed = fabs(speed_estimator.updateSpeed());
        if (predict_speed > FULLSTOP_SPD_THRESHHOLD)
        {
            if (predict_speed > MAXSPD_THRESHHOLD)
                curr_sensitivity = parameters.sensitivity;
            else
            {
                curr_sensitivity = parameters.sensitivity *
                                   (predict_speed - FULLSTOP_SPD_THRESHHOLD) * (MAXSPD_THRESHHOLD - FULLSTOP_SPD_THRESHHOLD);
                if (curr_sensitivity > 1)
                    curr_sensitivity = 1;
                if (curr_sensitivity < 0)
                    curr_sensitivity = 0;
            }
        }

        bool new_alarm_sw = angle_ctl_alarm_sw && parameters.Alarm_sw && curr_sensitivity > 0;
        if (curr_alarm_sw != new_alarm_sw)
        {
            if ((alarm_sw_voice_timer.elapsed() > 5000 || (new_alarm_sw && alarm_sw_voice_timer.elapsed() > 1200)))
            {
                alarm_sw_voice_timer.start();
                if (parameters.Alarm_sw == param_alarm_sw_buf)
                    asr_pro.send_cmd(ASR_PRO_Alarm_Pause_Answer, new_alarm_sw);
                else
                    param_alarm_sw_buf = parameters.Alarm_sw;
                curr_alarm_sw = new_alarm_sw;
                // if (curr_sensitivity == 0)
                // {
                //     ESP_LOGI("alarm_pause", "spd: %0.2f m/s", predict_speed);
                //     asr_pro.send_cmd(ASR_PRO_Speed_Answer, predict_speed * 100);
                // }
            }
        }
        else if (curr_sensitivity <= 0)
            alarm_sw_voice_timer.start();

        if (curr_alarm_sw)
        {
            float alarm_level = 0; // 0~1, 1 being max
            if (cur_distance < ALARM_MIN_DIST)
                alarm_level = 1;
            else if (cur_distance < ALARM_MAX_DIST)
                alarm_level = 1 - (cur_distance - ALARM_MIN_DIST) / (ALARM_MAX_DIST - ALARM_MIN_DIST);

            ESP_LOGI("Logic", "ALARM_MAX_DIST:%0.2f, cur_distance:%0.2f, predict_speed: %+0.2f, sensitivity: %0.2f, alarm_level: %0.2f",
                     ALARM_MAX_DIST, cur_distance, predict_speed, curr_sensitivity, alarm_level);

            if (alarm_level != 0)
            {
                beeper.update_pulseWidth((BEEPER_MAX_PWM - BEEPER_MIN_PWM) *
                                             ((cur_distance - 200) / (SENSITIVITY_MAX_DIST - 200)) +
                                         BEEPER_MIN_PWM);
                if (!beeper.is_busy())
                    beeper.begin(BEEPER_ON_TIME, BEEPER_OFF_MIN_TIME + (1 - alarm_level) * (BEEPER_OFF_MAX_TIME - BEEPER_OFF_MIN_TIME), 1);
            }
        }
        else
            ESP_LOGI("Logic", "ALARM_MAX_DIST:%0.2f, cur_distance:%0.2f, predict_speed: %+0.2f, param_sensitivity: %0.2f, curr_sensitivity: %0.2f, FULLSTOP_SPD_THRESHHOLD: %0.2f",
                     ALARM_MAX_DIST, cur_distance, predict_speed, parameters.sensitivity, curr_sensitivity, FULLSTOP_SPD_THRESHHOLD);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void voice_interface_Task(void *param)
{
    ASR_PRO_cmd_message received_msg;

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
                ASR_is_ready = true;
                break;
            case ASR_PRO_Battery_Inquiry:
                ESP_LOGI("voice_interface_Task", "bat: %0.2f %%", battery.get_percentage());
                asr_pro.send_cmd(ASR_PRO_Battery_Answer, battery.get_percentage());
                break;
            case ASR_PRO_Speed_Inquiry:
                ESP_LOGI("voice_interface_Task", "spd: %0.2f m/s", predict_speed);
                asr_pro.send_cmd(ASR_PRO_Speed_Answer, predict_speed * 100);
                break;
            case ASR_PRO_Sensitivity_Set:
                parameters.sensitivity = received_msg.data / 100.0;
                asr_pro.send_cmd(ASR_PRO_Sensitivity_Answer, parameters.sensitivity * 100.0);
                break;
            case ASR_PRO_Alarm_Switch_Set:
                parameters.Alarm_sw = (received_msg.data != 0);
                asr_pro.send_cmd(ASR_PRO_Alarm_Switch_Answer, parameters.Alarm_sw);
                break;
            case ASR_PRO_Distance_Inquiry:
            {
                uint8_t dist_send = (cur_distance <= 5000) ? (uint8_t)(cur_distance / 20) : 0;
                // ESP_LOGI("voice_interface_Task", "cur_distance:%.0f mm  dist_send: %0.2f m", cur_distance, dist_send * 20.0 / 1000);
                ESP_LOGI("voice_interface_Task", "cur_distance:%0.2f mm  Corrected dist 1: %0.0f mm, dist 2: %0.0f mm, dist 3: %0.0f mm, dist 4: %0.0f mm",
                         cur_distance,
                         corrected_distance[0], corrected_distance[1],
                         corrected_distance[2], corrected_distance[3]);
                asr_pro.send_cmd(ASR_PRO_Distance_Answer, dist_send);
                break;
            }
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

    Timer_milli low_bat_timer;
    for (; true;)
    {
        float tmp_battery_volt = analogSensors.get_bat_V();

        battery.heartbeat(tmp_battery_volt);
        // ESP_LOGI("Battery", "bat: %0.2f volt:%0.2f", battery.get_percentage(), tmp_battery_volt);
        if (ASR_is_ready && tmp_battery_volt > 6 && battery.get_percentage() < 3 && low_bat_timer.elapsed() > 30000)
        {
            ESP_LOGI("Battery", "bat: %0.2f %% volt:%0.2f", battery.get_percentage(), tmp_battery_volt);
            asr_pro.send_cmd(ASR_PRO_Low_Battery);
            low_bat_timer.start();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}