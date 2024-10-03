
#pragma once

#include "Differential_Timer.h"
#include "ICM42688.h"
#include "DataFilter.h"

#define DIST_SPD_MAX_WEIGHT 0.3
#define ANG_SPD_INTERVAL_UPPER 20
#define DIST_VAR_INTERVAL_UPPER 15000
#define DIST_VAR_SIZE 3
#define ACC_VAR_INTERVAL_UPPER 0.05
#define ACC_VAR_SIZE 300
#define VAR_WEIGHT_MIN 0.1

#define MAX_SPD_HURD 1.5 // m/s

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
    Timer_micro last_INS_update_timer;
    float INS_spd_buf = 0;
    Timer_micro last_INS_odo_update_timer;
    Timer_milli Holde_mode_update_timer;
    float Holde_mode_brake_buf = 0;
    double INS_odo_buf = 0;
    float distance_buff = 0;
    TaskHandle_t IMU_heartbeat_task_handle = nullptr;
    float speed_distance = 0; // 基于距离计算的速度
    KFP speed_distance_KFP;
    variance *distance_var;
    variance *acc_var;

    float map_interval(float curr_val, float lower_bound, float upper_bound);

public:
    Speed_estimator();
    ~Speed_estimator();

    static void IMU_heartbeat(void *param);
    void set_INS_speed(float spd);
    float get_INS_speed();
    float get_INS_odo();
    // 计算实时速度, 先获取实时时间算出时间差
    void set_distance(float cur_distance); // cur_distance in mm
    float updateSpeed();
    complimentary_angle_t get_angles();
};
