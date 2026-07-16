#ifndef __PHASE_PLL_H__
#define __PHASE_PLL_H__

#include "../utils/angle_utils.h"
#include "../utils/math_utils.h"

/**
 * @brief 二阶相位锁定环参数
 * @note 输入、状态和输出均使用电角度制：相位为rad，速度为rad/s
 */
typedef struct {
    float kp;                // 相位误差比例增益(1/s)
    float ki;                // 相位误差积分增益(1/s²)
    float sample_time_s;     // 迭代周期(s)
    float speed_limit_rad_s; // 速度积分状态限幅(rad/s)
} phase_pll_config_t;

/**
 * @brief 二阶相位锁定环实例
 */
typedef struct {
    phase_pll_config_t config;
    float              phase_rad;       // 估计相位[0, 2π)
    float              speed_rad_s;     // PI积分状态，稳态下收敛为角速度估计
    float              phase_error_rad; // 最近一次输入的相位误差[-π, π]
} phase_pll_t;

void  phasePll_init(phase_pll_t *pll, phase_pll_config_t *config, float initial_phase_rad);
void  phasePll_update(phase_pll_t *pll, float input_phase_rad);
float phasePll_get_phase(phase_pll_t *pll);
float phasePll_get_speed(phase_pll_t *pll);

#endif /* __PHASE_PLL_H__ */