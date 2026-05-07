#include "as5600.h"

static uint8_t as5600_i2c_rx_buf[2] = {0};

/**
 * @brief 拼接I2C接收缓冲区得到原始计数 0-4095
 */
static inline uint16_t as5600_build_rawCount(const uint8_t *recv_buffer)
{
    return ((((uint16_t)recv_buffer[0]) << 8) | recv_buffer[1]) & 0x0FFF;
}

/**
 * @brief 发起一次原始角度异步读取
 */
static void as5600_startRead_rawCount(void)
{
    i2c_read_bytesAsync(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, as5600_i2c_rx_buf, 2);
}

/**
 * @brief 初始化AS5600硬件接口
 */
void as5600_init(void)
{
    i2c_init();
    as5600_startRead_rawCount();
    HAL_Delay(1); // 确保首次I2C读写完成
}

/**
 * @brief 异步读取最新原始角度计数
 * @note 尝试获取一次新数据，如果数据尚未准备好，则返回0
 * @retval 1: 获取到新数据; 0: 数据尚未准备好
 */
uint8_t as5600_read_rawCountAsync(uint16_t *raw_count)
{
    i2c_readState_e read_state = i2c_get_readState();

    if (read_state == I2C_READ_STATE_DONE)
    {
        *raw_count = as5600_build_rawCount(as5600_i2c_rx_buf);
        as5600_startRead_rawCount(); // 消费完立即发起下一次读取
        return 1;
    }

    if ((read_state == I2C_READ_STATE_IDLE) || (read_state == I2C_READ_STATE_ERROR))
    {
        as5600_startRead_rawCount(); // 空闲或错误态下重新拉起读取；错误态会先触发I2C恢复
    }

    if ((read_state == I2C_READ_STATE_BUSY) || (read_state == I2C_READ_STATE_RECOVERING))
    {
        return 0;
    }

    return 0;
}

/**
 * @brief 阻塞读取一次原始角度计数
 * @retval 1: 成功获取到测量值; 0: 读取失败
 * @note 该接口用于零点对齐这类非实时流程，确保拿到的是实际测量角度，而不是PLL预测角度
 */
uint8_t as5600_read_rawCountBlock(uint16_t *raw_count)
{
    uint8_t recv_buffer[2] = {0};

    i2c_read_bytesBlock(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, recv_buffer, 2);
    *raw_count = as5600_build_rawCount(recv_buffer);
    return 1U;
}
