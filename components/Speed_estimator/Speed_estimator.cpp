
#include "Differential_Timer.h"
#include "ICM42688.h"
#include "DataFilter.h"
#include "math.h"

class Speed_estimator
{
private:
    Timer_milli D_Timer;
    icm42688_handle_t IMU;

    icm42688_value_t acc_value;
    icm42688_value_t gyro_value;
    complimentary_angle_t angle_value;
    KFP angle_roll_KFP;
    KFP angle_pitch_KFP;
    complimentary_angle_t tansformed_angle_value;
    icm42688_value_t pure_acceleration;
    icm42688_value_t tansformed_acceleration;
    Timer_micro last_INS_update_timer;
    float INS_spd_buf = 0;
    Timer_micro last_INS_odo_update_timer;
    Timer_milli Holde_mode_update_timer;
    float Holde_mode_brake_buf = 0;
    double INS_odo_buf = 0;
    float distance_buff = 0;

    // 构造器
public:
    Speed_estimator()
    {
        D_Timer.start();
        IMU = icm42688_create(I2C_NUM_0, ICM42688_I2C_ADDRESS);
        icm42688_cfg_t conf = {};
        conf.acce_fs = ACCE_FS_4G;
        conf.acce_odr = ACCE_ODR_32kHZ;
        conf.gyro_fs = GYRO_FS_1000DPS;
        conf.gyro_odr = GYRO_ODR_32kHZ;
        icm42688_config(IMU, conf);
        icm42688_set_pwr(IMU, false, GYRO_PWR_LOWNOISE, ACCE_PWR_LOWNOISE);
    }

    void IMU_heartbeat(void *param)
    {
        Speed_estimator *SE_instance = (Speed_estimator *)param;
        for (;;)
        {
            if (ESP_OK == icm42688_get_acce_gyro_value(SE_instance->IMU, &(SE_instance->acc_value), &(SE_instance->gyro_value))) // get acc and gyro data
            {
                icm42688_complimentory_filter(SE_instance->IMU, &(SE_instance->acc_value),
                                              &(SE_instance->gyro_value), &(SE_instance->angle_value)); // calculate pitch and roll using acc and gyro
                SE_instance->angle_value.roll = kalmanFilter(
                    &SE_instance->angle_roll_KFP, SE_instance->angle_value.roll);
                SE_instance->angle_value.pitch = kalmanFilter(
                    &SE_instance->angle_pitch_KFP, SE_instance->angle_value.pitch);

                { // Remove Gravity
                    complimentary_angle_t rad_angle_value;
                    rad_angle_value.roll = SE_instance->angle_value.roll / RAD_TO_DEG;
                    rad_angle_value.pitch = SE_instance->angle_value.pitch / RAD_TO_DEG;
                    complimentary_angle_t cos_angle_value;
                    cos_angle_value.roll = cos(rad_angle_value.roll);
                    cos_angle_value.pitch = cos(rad_angle_value.pitch);
                    SE_instance->pure_acceleration.x = SE_instance->acc_value.x - sin(rad_angle_value.roll) * fabs(cos_angle_value.pitch);
                    SE_instance->pure_acceleration.y = SE_instance->acc_value.y - sin(rad_angle_value.pitch) * fabs(cos_angle_value.roll);
                    if (cos_angle_value.pitch > 0 && cos_angle_value.roll > 0)
                        SE_instance->pure_acceleration.z = SE_instance->acc_value.z - cos_angle_value.pitch * cos_angle_value.roll;
                    else
                        SE_instance->pure_acceleration.z = SE_instance->acc_value.z + cos_angle_value.pitch * cos_angle_value.roll;
                }
                SE_instance->get_INS_odo();
            }
        }
    }
    void set_INS_speed(float spd)
    {
        last_INS_update_timer.start();
        INS_spd_buf = spd;
    }
    float get_INS_speed()
    {
        float dT = last_INS_update_timer.elapsed() / 1000.0;
        if (dT <= 0 || dT > 100)
            return INS_spd_buf;
        float curr_spd_x = INS_spd_buf + (tansformed_acceleration.y * dT * 9.8 / 1000);
        // ESP_LOGI(Logic_TAG, "tansformed_acceleration.x: %f, curr_spd_x: %f", pure_acceleration_x, curr_spd_x);
        return curr_spd_x;
    }
    float get_INS_odo()
    {
        float curr_spd_x = get_INS_speed();
        if (fabs(curr_spd_x) < 0.01)
            return INS_odo_buf;
        float dT = last_INS_odo_update_timer.elapsed() / 1000.0; // ms
        last_INS_odo_update_timer.start();
        // ESP_LOGE(Logic_TAG, "curr_spd_x: %f, INS_odo_buf: %f", curr_spd_x, INS_odo_buf);
        if (dT <= 0 || dT > 100 || fabsf(curr_spd_x) > 100)
            return INS_odo_buf;
        INS_odo_buf += (curr_spd_x * dT) / 1000.0;
        return INS_odo_buf;
    }
    // 计算实时速度, 先获取实时时间算出时间差
    float updateSpeed(float cur_distance) // cur_distance in cm
    {
        uint32_t dT = D_Timer.elapsed();
        float speed_distance = 0; // Speed calculated using distance
        if (dT != TIMER_MILLI_DEFAULT)
        {
            speed_distance = (cur_distance - distance_buff) / dT * 10; // cm/ms*10=m/s
        }
        distance_buff = cur_distance;

        // TODO: 角度变化时，speed_distance不生效或者减少权重
        float speed_distance_weight = 0.5;
        set_INS_speed((speed_distance_weight * speed_distance) + ((1 - speed_distance) * get_INS_speed())); // 更新惯性导航

        return get_INS_speed();
    }
    complimentary_angle_t get_angles()
    {
        return angle_value;
    }
};