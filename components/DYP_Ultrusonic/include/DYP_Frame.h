#pragma once
#define DYP_UART_SOF 0xFF // Start of Frame

struct DYP_UART_frame
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
        return checksum == (SOF + sensor1_high + sensor1_low + sensor2_high + sensor2_low +
                            sensor3_high + sensor3_low + sensor4_high + sensor4_low);
    }

    // 获取传感器数据
    uint16_t get_sensor_distance(uint8_t sensor_index) const
    {
        switch (sensor_index)
        {
        case 0:
            return (sensor1_high << 8) | sensor1_low;
        case 1:
            return (sensor2_high << 8) | sensor2_low;
        case 2:
            return (sensor3_high << 8) | sensor3_low;
        case 3:
            return (sensor4_high << 8) | sensor4_low;
        default:
            return 0xFFFF; // 返回0xFFFF表示错误的传感器编号
        }
    }
};
