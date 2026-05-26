#ifndef __MXLEMMING_OBSERVER_H_
#define __MXLEMMING_OBSERVER_H_

#include <math.h>
#include "stm32g4xx_hal.h"
#include "../app/user_config.h"
#include "../utils/angle_utils.h"
#include "../utils/math_utils.h"

/**
 * @brief MXLEMMING 观测器配置参数
 */
typedef struct
{
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
typedef struct
{
    const mxlemming_cfg_t *cfg;

    // 输入
    float i_alpha;
    float i_beta;
    float u_alpha;
    float u_beta;

    // 上一拍电流（差分用）
    float i_alpha_last;
    float i_beta_last;

    // 状态：纯永磁磁链 αβ 分量
    float x1;
    float x2;

    // PLL 状态
    float theta_est;     // atan2 直接输出角度 (rad)
    float theta_pll;     // PLL 跟踪输出角度 (rad)
    float speed_rad_s;   // PLL 积分状态 = 估算电角速度 (rad/s)
    float speed_est;     // 估算机械转速 (rpm)
    float pll_out_limit; // PLL 输出限幅 (rad/s)
} mxlemming_obs_t;

void mxlemmingObserver_init(mxlemming_obs_t *obs, const mxlemming_cfg_t *cfg);
void mxlemmingObserver_update(mxlemming_obs_t *obs);
float mxlemmingObserver_get_angle(mxlemming_obs_t *obs);
float mxlemmingObserver_get_speed(mxlemming_obs_t *obs);

#endif