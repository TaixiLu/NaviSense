
#pragma once

#include "Differential_Timer.h"
#include "ICM42688.h"
#include "DataFilter.h"

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
    KFP expected_speed_KFP;
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
    TaskHandle_t IMU_heartbeat_task_handle = nullptr;



    // 新增的成员变量
    float angular_velocity_z;    // 存储Z轴角速度
    float distance_history[5];   // 保存最近5次距离数据
    int history_index = 0;       // 记录当前历史数据索引
    float distance_variance;     // 距离变化的方差

public:
    Speed_estimator();
    ~Speed_estimator();

    static void IMU_heartbeat(void *param);
    void set_INS_speed(float spd);
    float get_INS_speed();
    float get_INS_odo();
    // 计算实时速度, 先获取实时时间算出时间差
    float updateSpeed(float cur_distance); // cur_distance in cm
    complimentary_angle_t get_angles();

    // 计算方差的函数
    float calculateVariance(float* data, int length);
};