/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define PIN_IMU_EN GPIO_NUM_47
#define PIN_IMU_INT2 GPIO_NUM_48
#define PIN_IMU_INT1 GPIO_NUM_35
#define PIN_IMU_SCL GPIO_NUM_13
#define PIN_IMU_SDA GPIO_NUM_12

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"

#define ICM42688_I2C_TIMEOUT 50

#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

#define ICM42688_I2C_ADDRESS 0x68   /*!< I2C address with AD0 pin low */
#define ICM42688_I2C_ADDRESS_1 0x69 /*!< I2C address with AD0 pin high */

    typedef enum
    {
        ACCE_FS_16G = 0, /*!< Accelerometer full scale range is +/- 16g (default)*/
        ACCE_FS_8G = 1,  /*!< Accelerometer full scale range is +/- 8g */
        ACCE_FS_4G = 2,  /*!< Accelerometer full scale range is +/- 4g */
        ACCE_FS_2G = 3,  /*!< Accelerometer full scale range is +/- 2g */
    } icm42688_acce_fs_t;

    typedef enum
    {
        ACCE_PWR_OFF = 0,      /*!< Accelerometer power off state */
        ACCE_PWR_ON = 1,       /*!< Accelerometer power on state */
        ACCE_PWR_LOWPOWER = 2, /*!< Accelerometer low-power mode */
        ACCE_PWR_LOWNOISE = 3, /*!< Accelerometer low noise state */
    } icm42688_acce_pwr_t;

    typedef enum
    {
        ACCE_ODR_32kHZ = 1,     /*!< Accelerometer ODR 32 kHz (LN mode)*/
        ACCE_ODR_16kHZ = 2,     /*!< Accelerometer ODR 16 kHz (LN mode)*/
        ACCE_ODR_8kHZ = 3,      /*!< Accelerometer ODR 8 kHz (LN mode)*/
        ACCE_ODR_4kHZ = 4,      /*!< Accelerometer ODR 4 kHz (LN mode)*/
        ACCE_ODR_2kHZ = 5,      /*!< Accelerometer ODR 2 kHz (LN mode)*/
        ACCE_ODR_1kHZ = 6,      /*!< Accelerometer ODR 1 kHz (LN mode) (default)*/
        ACCE_ODR_200HZ = 7,     /*!< Accelerometer ODR 200 Hz (LP or LN mode)*/
        ACCE_ODR_100HZ = 8,     /*!< Accelerometer ODR 100 Hz (LP or LN mode)*/
        ACCE_ODR_50HZ = 9,      /*!< Accelerometer ODR 50 Hz (LP or LN mode)*/
        ACCE_ODR_25HZ = 10,     /*!< Accelerometer ODR 25 Hz (LP or LN mode)*/
        ACCE_ODR_12_5HZ = 11,   /*!< Accelerometer ODR 12.5 Hz (LP or LN mode)*/
        ACCE_ODR_6_25HZ = 12,   /*!< Accelerometer ODR 6.25 Hz (LP mode)*/
        ACCE_ODR_3_125HZ = 13,  /*!< Accelerometer ODR 3.125 Hz (LP mode)*/
        ACCE_ODR_1_5625HZ = 14, /*!< Accelerometer ODR 1.5625 Hz (LP mode)*/
        ACCE_ODR_500HZ = 15,    /*!< Accelerometer ODR 500 Hz (LP or LN mode)*/
    } icm42688_acce_odr_t;

    typedef enum
    {
        GYRO_FS_2000DPS = 0,   /*!< Gyroscope full scale range is +/- 2000 degree per sencond (default) */
        GYRO_FS_1000DPS = 1,   /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
        GYRO_FS_500DPS = 2,    /*!< Gyroscope full scale range is +/- 500 degree per sencond */
        GYRO_FS_250DPS = 3,    /*!< Gyroscope full scale range is +/- 250 degree per sencond */
        GYRO_FS_125DPS = 4,    /*!< Gyroscope full scale range is +/- 125 degree per sencond */
        GYRO_FS_62_5DPS = 5,   /*!< Gyroscope full scale range is +/- 62.5 degree per sencond */
        GYRO_FS_31_25DPS = 6,  /*!< Gyroscope full scale range is +/- 31.25 degree per sencond */
        GYRO_FS_15_625DPS = 7, /*!< Gyroscope full scale range is +/- 15.625 degree per sencond */
    } icm42688_gyro_fs_t;

    typedef enum
    {
        GYRO_PWR_OFF = 0,      /*!< Gyroscope power off state */
        GYRO_PWR_STANDBY = 1,  /*!< Gyroscope power standby state */
        GYRO_PWR_LOWNOISE = 3, /*!< Gyroscope power low noise state */
    } icm42688_gyro_pwr_t;

    typedef enum
    {
        GYRO_ODR_32kHZ = 1,   /*!< Gyroscope ODR 32 kHz */
        GYRO_ODR_16kHZ = 2,   /*!< Gyroscope ODR 16 kHz */
        GYRO_ODR_8kHZ = 3,    /*!< Gyroscope ODR 8 kHz */
        GYRO_ODR_4kHZ = 4,    /*!< Gyroscope ODR 4 kHz */
        GYRO_ODR_2kHZ = 5,    /*!< Gyroscope ODR 2 kHz */
        GYRO_ODR_1kHZ = 6,    /*!< Gyroscope ODR 1 kHz  (default)*/
        GYRO_ODR_200HZ = 7,   /*!< Gyroscope ODR 200 Hz */
        GYRO_ODR_100HZ = 8,   /*!< Gyroscope ODR 100 Hz */
        GYRO_ODR_50HZ = 9,    /*!< Gyroscope ODR 50 Hz */
        GYRO_ODR_25HZ = 10,   /*!< Gyroscope ODR 25 Hz */
        GYRO_ODR_12_5HZ = 11, /*!< Gyroscope ODR 12.5 Hz */
        GYRO_ODR_500HZ = 15,  /*!< Gyroscope ODR 500 Hz */
    } icm42688_gyro_odr_t;

    typedef struct
    {
        icm42688_acce_fs_t acce_fs = ACCE_FS_16G;     /*!< Accelerometer full scale range */
        icm42688_acce_odr_t acce_odr = ACCE_ODR_1kHZ; /*!< Accelerometer ODR selection */
        icm42688_gyro_fs_t gyro_fs = GYRO_FS_2000DPS; /*!< Gyroscope full scale range */
        icm42688_gyro_odr_t gyro_odr = GYRO_ODR_1kHZ; /*!< Gyroscope ODR selection */
    } icm42688_cfg_t;

    typedef struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } icm42688_raw_value_t;

    typedef struct
    {
        float x;
        float y;
        float z;
    } icm42688_value_t;

    typedef struct
    {
        float roll;
        float pitch;
    } complimentary_angle_t;

    typedef enum
    {
        X_AXIS_WOM = 1,
        Y_AXIS_WOM = 2,
        Z_AXIS_WOM = 4,
        ALL_AXIS_WOM = X_AXIS_WOM | Y_AXIS_WOM | Z_AXIS_WOM,
    } icm42688_WOM_axis_t;

    typedef void *icm42688_handle_t;

    /**
     * @brief Create and init sensor object and return a sensor handle
     *
     * @param port I2C port number
     * @param dev_addr I2C device address of sensor
     *
     * @return
     *     - NULL Fail
     *     - Others Success
     */
    icm42688_handle_t icm42688_create(i2c_port_t port, const uint8_t dev_addr);

    /**
     * @brief Delete and release a sensor object
     *
     * @param sensor object handle of icm42688
     */
    void icm42688_delete(icm42688_handle_t sensor);

    /**
     * @brief Get device identification of ICM42688
     *
     * @param sensor object handle of icm42688
     * @param deviceid a pointer of device ID
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_deviceid(icm42688_handle_t sensor, uint8_t *deviceid);

    /**
     * @brief Set accelerometer power mode
     *
     * @param sensor object handle of icm42688
     * @param temp_en power enabler of Temperature sensor
     * @param gyro_state power mode of gyroscope
     * @param acc_state power mode of accelerometer
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_set_pwr(icm42688_handle_t sensor, bool temp_en,
                               icm42688_gyro_pwr_t gyro_state, icm42688_acce_pwr_t acc_state);

    /**
     * @brief Set accelerometer and gyroscope full scale range
     *
     * @param sensor object handle of icm42688
     * @param config Accelerometer and gyroscope configuration structure
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_config(icm42688_handle_t sensor, const icm42688_cfg_t config);

    /**
     * @brief Get accelerometer sensitivity
     *
     * @param sensor object handle of icm42688
     * @param sensitivity accelerometer sensitivity
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_acce_sensitivity(icm42688_handle_t sensor, float *sensitivity);

    /**
     * @brief Get gyroscope sensitivity
     *
     * @param sensor object handle of icm42688
     * @param sensitivity gyroscope sensitivity
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_gyro_sensitivity(icm42688_handle_t sensor, float *sensitivity);

    /**
     * @brief Read raw temperature measurements
     *
     * @param sensor object handle of icm42688
     * @param value raw temperature measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_temp_raw_value(icm42688_handle_t sensor, uint16_t *value);

    /**
     * @brief Read raw accelerometer measurements
     *
     * @param sensor object handle of icm42688
     * @param value raw accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_acce_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value);

    /**
     * @brief Read raw gyroscope measurements
     *
     * @param sensor object handle of icm42688
     * @param value raw gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_gyro_raw_value(icm42688_handle_t sensor, icm42688_raw_value_t *value);

    /**
     * @brief Read accelerometer measurements
     *
     * @param sensor object handle of icm42688
     * @param value accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_acce_value(icm42688_handle_t sensor, icm42688_value_t *value);

    /**
     * @brief Read gyro values
     *
     * @param sensor object handle of icm42688
     * @param value gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_gyro_value(icm42688_handle_t sensor, icm42688_value_t *value);

    /**
     * @brief Read accelerometer measurements
     *
     * @param sensor object handle of icm42688
     * @param acc_value accelerometer measurements
     * @param gyro_value gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_acce_gyro_value(icm42688_handle_t sensor,
                                           icm42688_value_t *acc_value, icm42688_value_t *gyro_value);

    /**
     * @brief Read temperature value
     *
     * @param sensor object handle of icm42688
     * @param value temperature measurements in Degrees Centigrade
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_get_temp_value(icm42688_handle_t sensor, float *value);

    /**
     * @brief use complimentory filter to caculate roll and pitch
     *
     * @param acce_value accelerometer measurements
     * @param gyro_value gyroscope measurements
     * @param complimentary_angle complimentary angle
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t icm42688_complimentory_filter(icm42688_handle_t sensor, const icm42688_value_t *acce_value,
                                            const icm42688_value_t *gyro_value, complimentary_angle_t *complimentary_angle);

#ifdef __cplusplus
}
#endif
