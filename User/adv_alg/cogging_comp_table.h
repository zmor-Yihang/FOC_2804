#ifndef __COGGING_COMP_TABLE_H__
#define __COGGING_COMP_TABLE_H__

#include "stm32g4xx_hal.h"

#define COGGING_COMP_TABLE_SIZE 512

extern const uint16_t g_cogging_comp_raw_count_table[COGGING_COMP_TABLE_SIZE];
extern const float g_cogging_comp_iq_table[COGGING_COMP_TABLE_SIZE];

#endif /* __COGGING_COMP_TABLE_H__ */
