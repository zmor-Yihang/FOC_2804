#ifndef __MXLEMMING_OBSERVER_H_
#define __MXLEMMING_OBSERVER_H_

#include <math.h>
#include "../alg/phase_pll.h"
#include "../app/user_config.h"
#include "../utils/angle_utils.h"
#include "../utils/math_utils.h"
#include "stm32g4xx_hal.h"

/**
 * @brief MXLEMMING 观测器配置参数
 */
typedef struct {
    float rs;                  // 定子电阻 (Ω)
    float ls;                  // 定子电感 (H)，表贴式取 (Ld+Lq)/2
    float lambda;              // 永磁体磁链幅值 (Wb)
    float poles;               // 电机极对数
    float ts;                  // 控制周期 (s)
    float pll_kp;              // PLL 比例系数
    float pll_ki;              // PLL 积分系数
    float pll_speed_limit_rpm; // PLL 机械转速限幅 (rpm)
} mxlemming_cfg_t;

/**
 * @brief MXLEMMING 观测器运行状态
 * @note x1/x2 直接表示纯永磁磁链 ψ_pm_α / ψ_pm_β
 */
typedef struct {
    mxlemming_cfg_t *cfg;

    // 上一拍电流（差分用）
    float i_alpha_last;
    float i_beta_last;

    // 状态：纯永磁磁链 αβ 分量
    float x1;
    float x2;

    // 角度与速度估计
    float       theta_est; // atan2直接输出角度(rad)
    phase_pll_t pll;       // 平滑角度与电角速度估计
} mxlemming_obs_t;

void  mxlemmingObserver_init(mxlemming_obs_t *obs, mxlemming_cfg_t *cfg);
void  mxlemmingObserver_update(mxlemming_obs_t *obs, alphabeta_t current, alphabeta_t applied_voltage);
float mxlemmingObserver_get_angle(mxlemming_obs_t *obs);
float mxlemmingObserver_get_speed(mxlemming_obs_t *obs);

#endif