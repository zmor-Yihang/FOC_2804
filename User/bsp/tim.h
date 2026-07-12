#ifndef __TIM_H__
#define __TIM_H__

// m1 PA5-PA7    TIM2_CH1 TIM3_CH1 TIM3_CH2
#include "../app/user_config.h"
#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/*
 * TIM1 PWM配置参数
 * TIM_CLK = 170 MHz / 定时器预分频因子
 * 系统时钟: 170MHz
 * PWM频率由 user_config.h 中的 FOC_CURRENT_LOOP_FREQ_HZ 指定（中心对齐模式下ARR计两次）
 */
#define TIM1_PRESCALER 0U                                                                                                 /* 预分频值 */
#define TIM1_PERIOD ((uint32_t)((170000000.0f / ((TIM1_PRESCALER) + 1.0f) / ((FOC_CURRENT_LOOP_FREQ_HZ) * 2.0f)) - 1.0f)) /* 自动重装载值（ARR） */

void tim_init(void);
void tim_set_pwmDuty(float duty1, float duty2, float duty3);

#endif /* __TIM_H__ */
