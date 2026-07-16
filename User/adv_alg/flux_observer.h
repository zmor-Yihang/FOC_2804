#ifndef __FLUX_OBSERVER_H_
#define __FLUX_OBSERVER_H_

#include <math.h>
#include "../alg/phase_pll.h"
#include "../app/user_config.h"
#include "../utils/angle_utils.h"

/**
 * @brief 非线性磁链观测器配置参数
 * @note 建议在系统初始化时配置一次，运行中保持不变
 */
typedef struct {
    float rs;                  // 定子电阻 (Ω)
    float ls;                  // 定子电感 (H)
    float psi_m;               // 永磁体磁链 (Wb)
    float poles;               // 电机极对数
    float ts;                  // 控制周期 (s)
    float gamma;               // 非线性增益
    float pll_kp;              // PLL 比例系数 Kp
    float pll_ki;              // PLL 积分系数 Ki
    float pll_speed_limit_rpm; // PLL 机械转速限幅 (rpm)
} fluxobserver_cfg_t;

/**
 * @brief 非线性磁链观测器运行状态
 * @note 基于磁链圆约束的非线性观测器
 */
typedef struct {
    const fluxobserver_cfg_t *cfg;

    float i_alpha; // 实测电流 alpha
    float i_beta;  // 实测电流 beta
    float u_alpha; // 电压 alpha
    float u_beta;  // 电压 beta

    float xhat_alpha; // 估算扩展磁链 alpha
    float xhat_beta;  // 估算扩展磁链 beta

    float       theta_est; // atan2直接估算角度(rad)
    phase_pll_t pll;       // 平滑角度与电角速度估计
} fluxobserver_t;

/**
 * @brief 初始化非线性磁链观测器
 * @param obs 观测器句柄
 * @param cfg 配置参数（需在观测器生命周期内保持有效）
 */
void fluxObserver_init(fluxobserver_t *obs, const fluxobserver_cfg_t *cfg);

/**
 * @brief 运行非线性磁链观测器
 * @param obs 观测器句柄
 */
void fluxObserver_estimate(fluxobserver_t *obs);

float fluxObserver_get_angle(fluxobserver_t *obs);

float fluxObserver_get_speed(fluxobserver_t *obs);

#endif
