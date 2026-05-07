#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32g4xx_hal.h"
#include "../app/user_config.h"
#include "../utils/angle_utils.h"
#include "../utils/math_utils.h"
#include "../bsp/as5600.h"

// 编码器本体参数
#define ENCODER_CPR 4096U

// 编码器PLL派生参数
#define ENCODER_SPEED_SAMPLE_TIME (1.0f / FOC_CURRENT_LOOP_FREQ_HZ) // encoder_update() 调用周期(s)
#define ENCODER_PLL_SPEED_LIMIT_RAD_S ((ENCODER_PLL_SPEED_LIMIT_RPM / 60.0f) * MATH_TWO_PI * MOTOR_POLE_PAIRS)

void encoder_init(void);
void encoder_update(void);
float encoder_get_pllAngle(void);
float encoder_get_pllSpeed(void);
float encoder_get_mechanicalAngle(void);
float encoder_get_mechanicalAngleBlock(void);
float encoder_get_mechanicalPosition(void);
void encoder_reset_mechanicalPosition(float position_rad);
uint16_t encoder_get_rawCount(void);

#endif /* __ENCODER_H__ */
