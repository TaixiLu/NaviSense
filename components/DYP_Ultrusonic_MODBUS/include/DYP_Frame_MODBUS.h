#pragma once
#include <stdint.h>
#define DYP_MODBUS_ADDR 0x01

typedef enum : uint8_t
{
    DYP_MODBUS_READ = 0x03,
    DYP_MODBUS_WRITE = 0x06
} DYP_MODBUS_CMD;
typedef enum : uint16_t
{
    DYP_REG_ADDR_SENS1 = 0x0106,
    DYP_REG_ADDR_SENS2 = 0x0107,
    DYP_REG_ADDR_SENS3 = 0x0108,
    DYP_REG_ADDR_SENS4 = 0x0109,
} DYP_REG_ADDR;

// CRC16 计算函数 (标准 Modbus CRC16)
uint16_t calculate_CRC16(const uint8_t *data, uint16_t length);

struct __attribute__((packed, aligned(1))) DYP_MODBUS_Frame_Master_Out
{
    uint8_t addr = DYP_MODBUS_ADDR;
    DYP_MODBUS_CMD cmd = DYP_MODBUS_READ;
    uint8_t reg_addr_H;
    uint8_t reg_addr_L;
    uint8_t num_reg_to_read_H = 0;
    uint8_t num_reg_to_read_L = 1;
    uint16_t crc16;

    // 构造函数
    DYP_MODBUS_Frame_Master_Out(DYP_REG_ADDR reg_addr_src = DYP_REG_ADDR_SENS1)
    {
        set_reg_addr(reg_addr_src);
        update_CRC();
    }

    void set_reg_addr(DYP_REG_ADDR src)
    {
        reg_addr_H = src >> 8;
        reg_addr_L = (uint8_t)src;
        get_sens_index();
    };
    DYP_REG_ADDR get_reg_addr() { return (DYP_REG_ADDR)(reg_addr_H << 8 | reg_addr_L); }

    // 更新校验和
    void update_CRC()
    {
        crc16 = calculate_CRC16((uint8_t *)this, sizeof(DYP_MODBUS_Frame_Master_Out) - sizeof(uint16_t));
    }
    uint8_t get_sens_index()
    {
        switch (get_reg_addr())
        {
        case DYP_REG_ADDR_SENS4:
            return 3;
        case DYP_REG_ADDR_SENS3:
            return 2;
        case DYP_REG_ADDR_SENS2:
            return 1;
        case DYP_REG_ADDR_SENS1:
            return 0;
        default:
            set_reg_addr(DYP_REG_ADDR_SENS1);
            return 0;
        }
    }
    void switch_sens()
    {
        switch (get_reg_addr())
        {
        case DYP_REG_ADDR_SENS1:
            set_reg_addr(DYP_REG_ADDR_SENS2);
            break;
        case DYP_REG_ADDR_SENS2:
            set_reg_addr(DYP_REG_ADDR_SENS3);
            break;
        case DYP_REG_ADDR_SENS3:
            set_reg_addr(DYP_REG_ADDR_SENS4);
            break;
        default:
            set_reg_addr(DYP_REG_ADDR_SENS1);
        }
        update_CRC();
    }
};

struct __attribute__((packed, aligned(1))) DYP_MODBUS_Frame_Master_In_Single_Sens
{
    uint8_t addr = DYP_MODBUS_ADDR;
    DYP_MODBUS_CMD cmd = DYP_MODBUS_READ;
    uint8_t bytes_red = 2;
    uint8_t data_H;
    uint8_t data_L;
    uint16_t crc16;

    // 校验校验和
    bool check_CRC() const
    {
        return crc16 == calculate_CRC16((uint8_t *)this,
                                        sizeof(DYP_MODBUS_Frame_Master_In_Single_Sens) - sizeof(uint16_t));
    }
    uint16_t get_data() { return (uint16_t)(data_H << 8 | data_L); }
};