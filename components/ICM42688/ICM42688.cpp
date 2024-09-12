/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "ICM42688.h"

#define ALPHA 0.98f /*!< Weight of gyroscope */

#define ICM42688_ID 0x47

/* ICM42688 register */
#define ICM42688_WHOAMI 0x75
#define ICM42688_GYRO_CONFIG0 0x4F
#define ICM42688_ACCEL_CONFIG0 0x50
// #define ICM42688_TEMP_CONFIG 0x54
#define ICM42688_PWR_MGMT0 0x4E
#define ICM42688_TEMP_DATA 0x1D
#define ICM42688_ACCEL_DATA 0x1F
#define ICM42688_GYRO_DATA 0x25

/* Sensitivity of the gyroscope */
#define GYRO_FS_2000_SENSITIVITY (16.4)
#define GYRO_FS_1000_SENSITIVITY (32.8)
#define GYRO_FS_500_SENSITIVITY (65.5)
#define GYRO_FS_250_SENSITIVITY (131.0)
#define GYRO_FS_125_SENSITIVITY (262.0)
#define GYRO_FS_62_5_SENSITIVITY (524.3)
#define GYRO_FS_31_25_SENSITIVITY (1048.6)
#define GYRO_FS_15_625_SENSITIVITY (2097.2)

/* Sensitivity of the accelerometer */
#define ACCE_FS_16G_SENSITIVITY (2048)
#define ACCE_FS_8G_SENSITIVITY (4096)
#define ACCE_FS_4G_SENSITIVITY (8192)
#define ACCE_FS_2G_SENSITIVITY (16384)

/*******************************************************************************
 * Types definitions
 *******************************************************************************/

typedef struct
{
    i2c_port_t bus;
    uint8_t dev_addr;
    uint32_t counter;
    float dt; /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
    float gyro_sensitivity = 0;
    float acc_sensitivity = 0;
} icm42688_dev_t;

/*******************************************************************************
 * Function definitions
 *******************************************************************************/
static esp_err_t icm42688_write(icm42688_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf, const uint8_t data_len);
static esp_err_t icm42688_read(icm42688_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len);

static esp_err_t icm42688_get_raw_value(icm42688_handle_t sensor, uint8_t reg, icm42688_raw_value_t *value);

/*******************************************************************************
 * Local variables
 *******************************************************************************/
static const char *TAG = "ICM42688";

/*******************************************************************************
 * Public API functions
 *******************************************************************************/

icm42688_handle_t icm42688_create(i2c_port_t port, const uint8_t dev_addr)
{
    icm42688_dev_t *sensor = (icm42688_dev_t *)heap_caps_calloc(1, sizeof(icm42688_dev_t), MALLOC_CAP_DEFAULT);
    sensor->bus = port;
    sensor->dev_addr = dev_addr;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));

    uint8_t dev_id = 0;
    icm42688_get_deviceid(sensor, &dev_id);
    if (dev_id != ICM42688_ID)
    {
        ESP_LOGE(TAG, "Incorrect Device ID (0x%02x).", dev_id);
        return NULL;
    }

    ESP_LOGI(TAG, "Found device ICM42688, ID: 0x%02x", dev_id);

    return (icm42688_handle_t)sensor;
}

void icm42688_delete(icm42688_handle_t sensor)
{
    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;

    if (sens->timer)
    {
        free(sens->timer);
    }

    free(sens);
}

esp_err_t icm42688_get_deviceid(icm42688_handle_t sensor, uint8_t *deviceid)
{
    esp_err_t ret = ESP_FAIL;

    assert(deviceid != NULL);

    for (int i = 0; (i < 5 && ret != ESP_OK); i++)
    {
        ret = icm42688_read(sensor, ICM42688_WHOAMI, deviceid, 1);
    }

    return ret;
}

esp_err_t icm42688_config(icm42688_handle_t sensor, const icm42688_cfg_t config)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[2];

    /* Gyroscope */
    data[0] = ((config.gyro_fs & 0x07) << 5) | (config.gyro_odr & 0x0F);
    /* Accelerometer */
    data[1] = ((config.acce_fs & 0x03) << 5) | (config.acce_odr & 0x0F);

    ret = icm42688_write(sensor, ICM42688_GYRO_CONFIG0, data, sizeof(data));

    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;
    ret = icm42688_get_acce_sensitivity(sensor, &(sens->acc_sensitivity));
    ESP_RETURN_ON_ERROR(ret, TAG, "Get acc_sensitivity error!");
    ret = icm42688_get_gyro_sensitivity(sensor, &(sens->gyro_sensitivity));
    ESP_RETURN_ON_ERROR(ret, TAG, "Get gyro_sensitivity error!");
    return ret;
}

esp_err_t icm42688_set_pwr(icm42688_handle_t sensor, bool temp_en,
                           icm42688_gyro_pwr_t gyro_state, icm42688_acce_pwr_t acc_state)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data = (temp_en) ? 0x00 : 0x10; // 5th bit set 1 to disable

    data |= (gyro_state & 0x03) << 2;
    data |= (acc_state & 0x03);

    ret = icm42688_write(sensor, ICM42688_PWR_MGMT0, &data, sizeof(data));

    return ret;
}

esp_err_t icm42688_get_acce_sensitivity(icm42688_handle_t sensor, float *sensitivity)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t acce_fs;

    assert(sensitivity != NULL);

    *sensitivity = 0;

    ret = icm42688_read(sensor, ICM42688_ACCEL_CONFIG0, &acce_fs, 1);
    if (ret == ESP_OK)
    {
        acce_fs = (acce_fs >> 5) & 0x03;
        switch (acce_fs)
        {
        case ACCE_FS_16G:
            *sensitivity = ACCE_FS_16G_SENSITIVITY;
            break;
        case ACCE_FS_8G:
            *sensitivity = ACCE_FS_8G_SENSITIVITY;
            break;
        case ACCE_FS_4G:
            *sensitivity = ACCE_FS_4G_SENSITIVITY;
            break;
        case ACCE_FS_2G:
            *sensitivity = ACCE_FS_2G_SENSITIVITY;
            break;
        }
    }

    return ret;
}

esp_err_t icm42688_get_gyro_sensitivity(icm42688_handle_t sensor, float *sensitivity)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t gyro_fs;

    assert(sensitivity != NULL);

    *sensitivity = 0;

    ret = icm42688_read(sensor, ICM42688_GYRO_CONFIG0, &gyro_fs, 1);
    if (ret == ESP_OK)
    {
        gyro_fs = (gyro_fs >> 5) & 0x07;
        switch (gyro_fs)
        {
        case GYRO_FS_2000DPS:
            *sensitivity = GYRO_FS_2000_SENSITIVITY;
            break;
        case GYRO_FS_1000DPS:
            *sensitivity = GYRO_FS_1000_SENSITIVITY;
            break;
        case GYRO_FS_500DPS:
            *sensitivity = GYRO_FS_500_SENSITIVITY;
            break;
        case GYRO_FS_250DPS:
            *sensitivity = GYRO_FS_250_SENSITIVITY;
            break;
        case GYRO_FS_125DPS:
            *sensitivity = GYRO_FS_125_SENSITIVITY;
            break;
        case GYRO_FS_62_5DPS:
            *sensitivity = GYRO_FS_62_5_SENSITIVITY;
            break;
        case GYRO_FS_31_25DPS:
            *sensitivity = GYRO_FS_31_25_SENSITIVITY;
            break;
        case GYRO_FS_15_625DPS:
            *sensitivity = GYRO_FS_15_625_SENSITIVITY;
            break;
        }
    }

    return ret;
}

esp_err_t icm42688_get_temp_raw_value(icm42688_handle_t sensor, uint16_t *value)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[2];

    assert(value != NULL);

    *value = 0;

    ret = icm42688_read(sensor, ICM42688_TEMP_DATA, data, sizeof(data));
    if (ret == ESP_OK)
    {
        *value = (uint16_t)((data[0] << 8) + data[1]);
    }

    return ret;
}

esp_err_t icm42688_get_acce_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value)
{
    return icm42688_get_raw_value(sensor, ICM42688_ACCEL_DATA, value);
}

esp_err_t icm42688_get_gyro_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value)
{
    return icm42688_get_raw_value(sensor, ICM42688_GYRO_DATA, value);
}

esp_err_t icm42688_get_acce_value(icm42688_handle_t sensor, icm42688_value_t *value)
{
    esp_err_t ret;
    icm42688_raw_value_t raw_value;

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;
    if (sens->acc_sensitivity == 0)
    {
        ret = icm42688_get_acce_sensitivity(sensor, &(sens->acc_sensitivity));
        ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");
    }

    ret = icm42688_get_acce_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    value->x = raw_value.x / sens->acc_sensitivity;
    value->y = raw_value.y / sens->acc_sensitivity;
    value->z = raw_value.z / sens->acc_sensitivity;

    return ESP_OK;
}

esp_err_t icm42688_get_gyro_value(icm42688_handle_t sensor, icm42688_value_t *value)
{
    esp_err_t ret;
    icm42688_raw_value_t raw_value;

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;
    if (sens->gyro_sensitivity == 0)
    {
        ret = icm42688_get_gyro_sensitivity(sensor, &(sens->gyro_sensitivity));
        ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");
    }

    ret = icm42688_get_gyro_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    value->x = raw_value.x / sens->gyro_sensitivity;
    value->y = raw_value.y / sens->gyro_sensitivity;
    value->z = raw_value.z / sens->gyro_sensitivity;

    return ESP_OK;
}

esp_err_t icm42688_get_acce_gyro_value(icm42688_handle_t sensor,
                                       icm42688_value_t *acc_value, icm42688_value_t *gyro_value)
{
    esp_err_t ret;
    assert(acc_value != NULL);
    assert(gyro_value != NULL);
    acc_value->x = 0;
    acc_value->y = 0;
    acc_value->z = 0;
    gyro_value->x = 0;
    gyro_value->y = 0;
    gyro_value->z = 0;

    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;
    if (sens->acc_sensitivity == 0)
    {
        ret = icm42688_get_acce_sensitivity(sensor, &(sens->acc_sensitivity));
        ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");
    }
    if (sens->gyro_sensitivity == 0)
    {
        ret = icm42688_get_gyro_sensitivity(sensor, &(sens->gyro_sensitivity));
        ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");
    }

    uint8_t data[12];
    icm42688_raw_value_t acc_raw_value;
    icm42688_raw_value_t gyro_raw_value;
    ret = icm42688_read(sensor, ICM42688_ACCEL_DATA, data, 12);
    if (ret == ESP_OK)
    {
        acc_value->x = ((int16_t)((data[0] << 8) + data[1])) / sens->acc_sensitivity;
        acc_value->y = ((int16_t)((data[2] << 8) + data[3])) / sens->acc_sensitivity;
        acc_value->z = ((int16_t)((data[4] << 8) + data[5])) / sens->acc_sensitivity;
        gyro_value->x = ((int16_t)((data[6] << 8) + data[7])) / sens->gyro_sensitivity;
        gyro_value->y = ((int16_t)((data[8] << 8) + data[9])) / sens->gyro_sensitivity;
        gyro_value->z = ((int16_t)((data[10] << 8) + data[11])) / sens->gyro_sensitivity;

        // ESP_LOGI(TAG, "ACC:(%f, %f, %f)Sens:%f   GYRO:(%f, %f, %f)Sens:%f ",
        //          acc_value->x, acc_value->y, acc_value->z, sens->acc_sensitivity,
        //          gyro_value->x, gyro_value->y, gyro_value->z, sens->gyro_sensitivity);
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    return ESP_OK;
}

esp_err_t icm42688_get_temp_value(icm42688_handle_t sensor, float *value)
{
    esp_err_t ret;
    uint16_t raw_value;

    assert(value != NULL);

    *value = 0;

    ret = icm42688_get_temp_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    *value = (raw_value / 128) + 25;

    return ESP_OK;
}

esp_err_t icm42688_complimentory_filter(icm42688_handle_t sensor, const icm42688_value_t *const acce_value,
                                        const icm42688_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;

    sens->counter++;
    if (sens->counter == 1)
    {
        acce_angle[1] = (atan2(acce_value->y, acce_value->z) * RAD_TO_DEG);
        acce_angle[0] = (atan2(acce_value->x, acce_value->z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
    }
    else
    {
        struct timeval now, dt_t;
        gettimeofday(&now, NULL);
        timersub(&now, sens->timer, &dt_t);
        sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
        gettimeofday(sens->timer, NULL);

        acce_angle[1] = (atan2(acce_value->y, acce_value->z) * RAD_TO_DEG);
        acce_angle[0] = (atan2(acce_value->x, acce_value->z) * RAD_TO_DEG);

        gyro_rate[1] = gyro_value->x;
        gyro_rate[0] = -gyro_value->y;
        gyro_angle[1] = complimentary_angle->pitch + gyro_rate[0] * sens->dt;
        gyro_angle[0] = complimentary_angle->roll + gyro_rate[1] * sens->dt;
        for (int i = 0; i < 2; i++)
        {
            if (acce_angle[i] > 90 && gyro_angle[i] < -90)
                gyro_angle[i] += 360;
            if (acce_angle[i] < -90 && gyro_angle[i] > 90)
                gyro_angle[i] -= 360;
        }

        complimentary_angle->roll = (ALPHA * gyro_angle[0]) + ((1 - ALPHA) * acce_angle[0]);
        complimentary_angle->pitch = (ALPHA * gyro_angle[1]) + ((1 - ALPHA) * acce_angle[1]);
        if (isnan(complimentary_angle->roll) || isinf(complimentary_angle->roll))
        {
            complimentary_angle->roll = 0;
            sens->counter = 0;
        }
        if (isnan(complimentary_angle->pitch) || isinf(complimentary_angle->pitch))
        {
            complimentary_angle->pitch = 0;
            sens->counter = 0;
        }

        while (complimentary_angle->roll > 180)
            complimentary_angle->roll -= 360;
        while (complimentary_angle->roll < -180)
            complimentary_angle->roll += 360;
        while (complimentary_angle->pitch > 180)
            complimentary_angle->pitch -= 360;
        while (complimentary_angle->pitch < -180)
            complimentary_angle->pitch += 360;
    }

    // ESP_LOGI(TAG, "ACC:(%+02.3f, %+02.3f, %+02.3f)  GYRO:(%+02.3f, %+02.3f, %+02.3f)  roll: %+02.3f, pitch: %+02.3f",
    //          acce_value->x, acce_value->y, acce_value->z,
    //          gyro_value->x, gyro_value->y, gyro_value->z,
    //          complimentary_angle->roll, complimentary_angle->pitch);
    // ESP_LOGI(TAG, "ACC:(%+02.3f, %+02.3f, %+02.3f)      acce_roll: %f   gyro_roll: %f   roll: %f      acce_pitch: %f   gyro_pitch: %f   pitch: %f",
    //          acce_value->x, acce_value->y, acce_value->z,
    //          acce_angle[0], gyro_angle[0], complimentary_angle->roll,
    //          acce_angle[1], gyro_angle[1], complimentary_angle->pitch);
    return ESP_OK;
}

/*******************************************************************************
 * Private functions
 *******************************************************************************/

static esp_err_t icm42688_get_raw_value(icm42688_handle_t sensor, uint8_t reg, icm42688_raw_value_t *value)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[6];

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42688_read(sensor, reg, data, sizeof(data));
    if (ret == ESP_OK)
    {
        value->x = (int16_t)((data[0] << 8) + data[1]);
        value->y = (int16_t)((data[2] << 8) + data[3]);
        value->z = (int16_t)((data[4] << 8) + data[5]);
    }

    return ret;
}

static esp_err_t icm42688_write(icm42688_handle_t sensor, const uint8_t reg_start_addr,
                                const uint8_t *data_buf, const uint8_t data_len)
{
    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;
    esp_err_t ret;

    assert(sens);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, (sens->dev_addr << 1) | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, ICM42688_I2C_TIMEOUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t icm42688_read(icm42688_handle_t sensor, const uint8_t reg_start_addr,
                               uint8_t *data_buf, const uint8_t data_len)
{
    icm42688_dev_t *sens = (icm42688_dev_t *)sensor;
    uint8_t reg_buff[] = {reg_start_addr};

    assert(sens);

    /* Write register number and read data */
    return i2c_master_write_read_device(sens->bus, sens->dev_addr, reg_buff, sizeof(reg_buff),
                                        data_buf, data_len, ICM42688_I2C_TIMEOUT / portTICK_PERIOD_MS);
}