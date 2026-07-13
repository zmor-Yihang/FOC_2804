#include "i2c.h"

I2C_HandleTypeDef hi2c3;

#define I2C3_ASYNC_TIMEOUT_MS 5U

static volatile i2c_readState_e i2c3_read_state     = I2C_READ_STATE_IDLE;
static volatile uint32_t        i2c3_async_start_ms = 0U;
static volatile uint32_t        i2c3_error_count    = 0U;
static volatile uint32_t        i2c3_timeout_count  = 0U;
static volatile uint32_t        i2c3_recover_count  = 0U;

static HAL_StatusTypeDef i2c3_recover_bus(void) {
    i2c3_read_state = I2C_READ_STATE_RECOVERING;

    HAL_I2C_DeInit(&hi2c3);
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        i2c3_read_state = I2C_READ_STATE_ERROR;
        return HAL_ERROR;
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        i2c3_read_state = I2C_READ_STATE_ERROR;
        return HAL_ERROR;
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
        i2c3_read_state = I2C_READ_STATE_ERROR;
        return HAL_ERROR;
    }

    i2c3_recover_count++;
    i2c3_read_state = I2C_READ_STATE_IDLE;
    return HAL_OK;
}

void i2c_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_I2C3_CLK_ENABLE();

    // PC8: I2C3_SCL, PC9: I2C3_SDA
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    hi2c3.Instance              = I2C3;                    // 选择I2C3
    hi2c3.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;   // 不使能无拉伸模式, 即允许时钟拉伸
    hi2c3.Init.Timing           = 0x10C31027;              // 时序参数: 1000kHz
    hi2c3.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT; // 7位地址模式
    hi2c3.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE; // 单地址模式
    hi2c3.Init.OwnAddress1      = 0;                       // 自身地址
    hi2c3.Init.OwnAddress2      = 0;                       // 第二地址
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;          // 地址掩码
    hi2c3.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE; // 禁止广播呼叫

    HAL_I2C_Init(&hi2c3);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0);

    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 3, 0); // 优先级比adc回调高, 因为adc回调依赖角度，且该中断快
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);

    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 3, 1);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

    i2c3_read_state = I2C_READ_STATE_IDLE;
}

/**
 * @brief 阻塞读取
 */
void i2c_read_bytesBlock(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len) {
    HAL_I2C_Mem_Read(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, recv_buffer, len, 1);
}

/**
 * @brief 异步读取
 */
void i2c_read_bytesAsync(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len) {
    i2c_readState_e read_state = i2c_get_readState();

    // 如果总线状态忙或者正在错误恢复，直接返回
    if ((read_state == I2C_READ_STATE_BUSY) || (read_state == I2C_READ_STATE_RECOVERING)) {
        return;
    }

    // 如果总线状态错误，尝试恢复总线
    if (read_state == I2C_READ_STATE_ERROR) {
        if (i2c3_recover_bus() != HAL_OK) {
            return;
        }
    }

    // 尝试读取
    if (HAL_I2C_Mem_Read_IT(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, recv_buffer, len) == HAL_OK) {
        i2c3_async_start_ms = HAL_GetTick();       // 记录开始时间
        i2c3_read_state     = I2C_READ_STATE_BUSY; // 设置总线状态为忙
    } else                                         // 读取失败
    {
        i2c3_error_count++;                     // 错误计数加1
        i2c3_read_state = I2C_READ_STATE_ERROR; // 设置总线状态为错误
    }
}

/**
 * @brief 获取总线状态
 * @note 错误状态有两种，一个未知错误，一个是总线超时
 */
i2c_readState_e i2c_get_readState(void) {
    if (i2c3_read_state == I2C_READ_STATE_BUSY) {
        // 获取当前时间
        uint32_t now_ms = HAL_GetTick();

        // 判断异步读取是否超时
        if ((now_ms - i2c3_async_start_ms) > I2C3_ASYNC_TIMEOUT_MS) {
            i2c3_timeout_count++;                   // 超时计数加1
            i2c3_error_count++;                     // 错误计数加1
            i2c3_read_state = I2C_READ_STATE_ERROR; // 设置总线状态为错误
        }
    }

    return i2c3_read_state;
}

/**
 * @brief 读取总线状态
 */
void i2c_set_readState(i2c_readState_e current_state) {
    i2c3_read_state = current_state;
}

uint32_t i2cDebug_get_errorCount(void) {
    return i2c3_error_count;
}

uint32_t i2cDebug_get_timeoutCount(void) {
    return i2c3_timeout_count;
}

uint32_t i2cDebug_get_recoverCount(void) {
    return i2c3_recover_count;
}

/**
 * @brief I2C3事件中断处理函数
 */
void I2C3_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&hi2c3);
}

/**
 * @brief I2C3错误中断处理函数
 */
void I2C3_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(&hi2c3);
}

/**
 * @brief I2C3接收完成中断处理函数
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C3) {
        i2c3_read_state = I2C_READ_STATE_DONE;
    }
}

/**
 * @brief I2C3错误回调
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C3) {
        i2c3_error_count++;
        i2c3_read_state = I2C_READ_STATE_ERROR;
    }
}
