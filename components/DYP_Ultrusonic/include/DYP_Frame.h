#pragma once
#define DYP_UART_SOF 0xFF // Start of Frame

struct __attribute__((packed, aligned(1))) DYP_UART_frame
{
    uint8_t SOF = DYP_UART_SOF; // 帧头
    uint8_t sensor1_high;       // 1号传感器数据高8位
    uint8_t sensor1_low;        // 1号传感器数据低8位
    uint8_t sensor2_high;       // 2号传感器数据高8位
    uint8_t sensor2_low;        // 2号传感器数据低8位
    uint8_t sensor3_high;       // 3号传感器数据高8位
    uint8_t sensor3_low;        // 3号传感器数据低8位
    uint8_t sensor4_high;       // 4号传感器数据高8位
    uint8_t sensor4_low;        // 4号传感器数据低8位
    uint8_t checksum;           // 校验和

    // 构造函数
    DYP_UART_frame(uint8_t s1h = 0, uint8_t s1l = 0, uint8_t s2h = 0, uint8_t s2l = 0,
                   uint8_t s3h = 0, uint8_t s3l = 0, uint8_t s4h = 0, uint8_t s4l = 0)
    {
        sensor1_high = s1h;
        sensor1_low = s1l;
        sensor2_high = s2h;
        sensor2_low = s2l;
        sensor3_high = s3h;
        sensor3_low = s3l;
        sensor4_high = s4h;
        sensor4_low = s4l;
        update_checksum();
    }

    // 更新校验和
    void update_checksum()
    {
        checksum = SOF + sensor1_high + sensor1_low + sensor2_high + sensor2_low +
                   sensor3_high + sensor3_low + sensor4_high + sensor4_low;
    }

    // 校验校验和
    bool check_checksum() const
    {
        return checksum == (uint8_t)(SOF + sensor1_high + sensor1_low + sensor2_high + sensor2_low +
                                     sensor3_high + sensor3_low + sensor4_high + sensor4_low);
    }

    // 获取传感器数据
    uint16_t get_sensor_distance(uint8_t sensor_index) const
    {
        uint16_t rtn = 0xFFFF; // 返回0xFFFF表示错误的传感器编号或传感器检测不到值
        switch (sensor_index)
        {
        case 0:
            rtn = (sensor1_high << 8) | sensor1_low;
            break;
        case 1:
            rtn = (sensor2_high << 8) | sensor2_low;
            break;
        case 2:
            rtn = (sensor3_high << 8) | sensor3_low;
            break;
        case 3:
            rtn = (sensor4_high << 8) | sensor4_low;
            break;
        }
        if (rtn == 0)
            rtn = 0xFFFF;
        return rtn;
    }
};
