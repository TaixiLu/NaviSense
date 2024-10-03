
#include "Speed_estimator.h"
#include "math.h"
#include "esp_log.h"

Speed_estimator::Speed_estimator()
{
    {
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = PIN_IMU_SDA;
        conf.scl_io_num = PIN_IMU_SCL;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 1000 * 1000;
        i2c_param_config(I2C_NUM_0, &conf);
        i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM);
    }

    {
        IMU = icm42688_create(I2C_NUM_0, ICM42688_I2C_ADDRESS);
        icm42688_cfg_t conf = {};
        conf.acce_fs = ACCE_FS_4G;
        conf.acce_odr = ACCE_ODR_32kHZ;
        conf.gyro_fs = GYRO_FS_1000DPS;
        conf.gyro_odr = GYRO_ODR_32kHZ;
        icm42688_config(IMU, conf);
        icm42688_set_pwr(IMU, false, GYRO_PWR_LOWNOISE, ACCE_PWR_LOWNOISE);
    }

    KFP_init(&angle_roll_KFP, 0.3, 0.01);
    KFP_init(&angle_pitch_KFP, 0.3, 0.01);
    KFP_init(&speed_distance_KFP, 0.03, 0.1);

    last_INS_update_timer.start();
    last_INS_odo_update_timer.start();
    D_Timer.start();
    distance_var = varianceInit(DIST_VAR_SIZE);
    acc_var = varianceInit(ACC_VAR_SIZE);
    xTaskCreatePinnedToCore(Speed_estimator::IMU_heartbeat, "IMU_heartbeat",
                            2000 + configMINIMAL_STACK_SIZE, this, configMAX_PRIORITIES - 1,
                            &IMU_heartbeat_task_handle, 1);
}
Speed_estimator::~Speed_estimator()
{
    vTaskDelete(IMU_heartbeat_task_handle);
}

void Speed_estimator::IMU_heartbeat(void *param)
{
    Speed_estimator *SE_instance = (Speed_estimator *)param;
    for (;;)
    {
        if (ESP_OK == icm42688_get_acce_gyro_value(SE_instance->IMU, &(SE_instance->acc_value), &(SE_instance->gyro_value))) // get acc and gyro data
        {
            variancePush(SE_instance->acc_var, SE_instance->acc_value.z);
            // ESP_LOGI("42688", "acc_value.x: %0.2f, y: %0.2f, z:%0.2f", SE_instance->acc_value.x, SE_instance->acc_value.y, SE_instance->acc_value.z);
            // ESP_LOGI("42688", "acc_value.z:%+0.2f", SE_instance->acc_value.z);
            icm42688_complimentory_filter(SE_instance->IMU, &(SE_instance->acc_value),
                                          &(SE_instance->gyro_value), &(SE_instance->angle_value)); // calculate pitch and roll using acc and gyro
            SE_instance->angle_value.roll = kalmanFilter(
                &SE_instance->angle_roll_KFP, SE_instance->angle_value.roll);
            SE_instance->angle_value.pitch = kalmanFilter(
                &SE_instance->angle_pitch_KFP, SE_instance->angle_value.pitch);

            // 继续处理加速度数据...
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

            SE_instance->get_INS_odo();
        }
    }
}

void Speed_estimator::set_INS_speed(float spd)
{
    last_INS_update_timer.start();
    INS_spd_buf = spd;
}
float Speed_estimator::get_INS_speed()
{
    float dT = last_INS_update_timer.elapsed() / 1000.0;
    if (dT <= 0 || dT > 100)
        return INS_spd_buf;
    float curr_spd_z = INS_spd_buf - (pure_acceleration.z * dT * 9.8 / 1000);
    // ESP_LOGI("Speed Estimator", "INS_spd_buf:%+0.2f pure_acceleration.z: %+0.3f, curr_spd_z: %+0.2f", INS_spd_buf, pure_acceleration.z, curr_spd_z);
    return curr_spd_z;
}
float Speed_estimator::get_INS_odo()
{
    float curr_spd_z = get_INS_speed();
    if (fabs(curr_spd_z) < 0.01)
        return INS_odo_buf;
    float dT = last_INS_odo_update_timer.elapsed() / 1000.0; // ms
    last_INS_odo_update_timer.start();
    // ESP_LOGE(Logic_TAG, "curr_spd_x: %f, INS_odo_buf: %f", curr_spd_x, INS_odo_buf);
    if (dT <= 0 || dT > 100 || fabsf(curr_spd_z) > 100)
        return INS_odo_buf;
    INS_odo_buf += (curr_spd_z * dT) / 1000.0;
    return INS_odo_buf;
}

// 计算实时速度, 结合距离变化和加速度
void Speed_estimator::set_distance(float cur_distance) // cur_distance in mm
{
    if (isnanf(cur_distance) || isinff(cur_distance))
    {
        D_Timer.clear();
        return;
    }
    uint32_t dT = D_Timer.elapsed();
    if (dT != TIMER_MILLI_DEFAULT)
    {
        speed_distance = (distance_buff - cur_distance) / dT; // mm/ms=m/s
        speed_distance = kalmanFilter(&speed_distance_KFP, speed_distance);
        // ESP_LOGI("Speed_estimator", "speed_distance: %+0.2f cur_distance:%+0.2f Dt:%+0.3f", speed_distance, cur_distance, dT / 1000.0);
    }
    D_Timer.start();
    distance_buff = cur_distance;
    variancePush(distance_var, cur_distance);
    // ESP_LOGI("Speed_estimator", "cur_distance: %+0.2f  distance_variance:%+0.2f", cur_distance, varianceGet(distance_var));
}
float Speed_estimator::updateSpeed()
{
    // 计算距离的方差
    float distance_variance = varianceGet(distance_var);
    float acc_variance = varianceGet(acc_var);
    float turning_spd = fabs(gyro_value.y);

    // 检测旋转和方差，调整速度估算的权重
    float speed_distance_weight = DIST_SPD_MAX_WEIGHT;
    if (turning_spd > ANG_SPD_INTERVAL_UPPER || distance_variance >= DIST_VAR_INTERVAL_UPPER)
    {
        D_Timer.clear();
        speed_distance_weight = 0;
    }
    else
    {
        speed_distance_weight *= 1 - map_interval(turning_spd, 0, ANG_SPD_INTERVAL_UPPER);
        if (acc_variance > 0)
        {
            float var_param = map_interval(distance_variance, 0, DIST_VAR_INTERVAL_UPPER) /
                              map_interval(acc_variance, 0, ACC_VAR_INTERVAL_UPPER);
            if (var_param > 1)
                speed_distance_weight *= VAR_WEIGHT_MIN;
            else
                speed_distance_weight *= (1 - VAR_WEIGHT_MIN) * (1 - var_param) + VAR_WEIGHT_MIN;
            // ESP_LOGI("SE", "distance_variance_cast: %+0.2f acc_variance_cast: %+0.2f  speed_distance_weight_ratio: %+0.2f",
            //          map_interval(distance_variance, 0, DIST_VAR_INTERVAL_UPPER), map_interval(acc_variance, 0, ACC_VAR_INTERVAL_UPPER),
            //          (1 - VAR_WEIGHT_MIN) * (1 - map_interval(distance_variance, 0, DIST_VAR_INTERVAL_UPPER) / map_interval(acc_variance, 0, ACC_VAR_INTERVAL_UPPER)) + VAR_WEIGHT_MIN);
        }
        else
            speed_distance_weight *= VAR_WEIGHT_MIN;
    }

    if (isnanf(speed_distance) || isinff(speed_distance))
        speed_distance = 0;
    float final_speed = (speed_distance_weight * speed_distance) + ((1 - speed_distance_weight) * get_INS_speed());

    if (final_speed > MAX_SPD_HURD)
        final_speed = MAX_SPD_HURD;
    else if (final_speed < MIN_SPD_HURD)
        final_speed = MIN_SPD_HURD;

    // ESP_LOGI("SE", "distance_variance_cast: %+0.2f acc_variance_cast: %+0.4f  turning_spd_cast:%+0.2f  speed_distance_weight:%+0.2f speed_distance:%+0.2f final_speed:%+0.2f",
    //          map_interval(distance_variance, 0, DIST_VAR_INTERVAL_UPPER), map_interval(acc_variance, 0, ACC_VAR_INTERVAL_UPPER),
    //          map_interval(turning_spd, 0, ANG_SPD_INTERVAL_UPPER), speed_distance_weight, speed_distance, final_speed);
    set_INS_speed(final_speed); // 更新INS速度

    return final_speed;
}

float Speed_estimator::map_interval(float curr_val, float lower_bound, float upper_bound)
{
    if (curr_val <= lower_bound)
        return 0;
    else if (curr_val >= upper_bound)
        return 1;
    else
        return (curr_val - lower_bound) / (upper_bound - lower_bound);
}

complimentary_angle_t Speed_estimator::get_angles()
{
    return angle_value;
}