#ifndef __COGGING_COMP_H__
#define __COGGING_COMP_H__

#include "stm32g431xx.h"
#include "../app/user_config.h"
#include "./sensor/encoder.h"

extern const uint16_t g_cogging_comp_raw_count_table[COGGING_COMP_TABLE_SIZE];
extern const float g_cogging_comp_iq_table[COGGING_COMP_TABLE_SIZE];

/**
 * @brief 根据编码器单圈 raw count 查表得到齿槽 q 轴前馈补偿电流
 * @param raw_count 编码器单圈原始计数，范围 0 ~ ENCODER_CPR-1
 * @return q 轴补偿电流(A)
 */
float coggingComp_getIqByRawCount(uint16_t raw_count);

#endif /* __COGGING_COMP_H__ */