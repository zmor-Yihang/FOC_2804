#ifndef __LUENBERGER_DOB_H__
#define __LUENBERGER_DOB_H__

#include <math.h>
#include "../app/user_config.h"
#include "../utils/math_utils.h"

/**
 * @brief 龙伯格扰动观测器配置参数
 * @note 建议在系统初始化时配置一次，运行中保持不变
 *
 * 机械模型：
 *   omega_dot = a * omega + b * iq + d
 * 其中：
 *   a = -B / J
 *   b = Kt / J
 *   d = -TL / J
 */
typedef struct {
    float ts;            // 观测器执行周期 (s)
    float j;             // 转动惯量 (kg·m²)
    float damping_b;     // 粘性摩擦系数 (N·m·s/rad)
    float kt;            // q轴转矩常数 (N·m/A)
    float bandwidth_hz;  // 观测器带宽 (Hz)
    float zeta;          // 观测器阻尼系数
    float iq_comp_limit; // q轴补偿电流限幅 (A)
    float comp_gain;     // 补偿开启比例
} luenberger_dob_cfg_t;

/**
 * @brief 龙伯格扰动观测器运行状态
 * @note d_hat 为加速度维度扰动；iq_comp 由 d_hat 换算得到
 */
typedef struct {
    luenberger_dob_cfg_t *cfg;

    // 由配置预计算的模型与增益
    float a;       // -B / J
    float plant_b; // Kt / J
    float l1;      // 速度误差反馈增益
    float l2;      // 扰动误差反馈增益

    // 估计状态
    float omega_hat; // 估计机械角速度 (rad/s)
    float d_hat;     // 估计扰动 (rad/s²)，约等于 -TL/J
    float iq_comp;   // 补偿用 q 轴电流 (A)
} luenberger_dob_t;

/**
 * @brief 初始化龙伯格扰动观测器
 * @param obs 观测器句柄
 * @param cfg 配置参数（需在观测器生命周期内保持有效）
 */
void luenbergerDOB_init(luenberger_dob_t *obs, luenberger_dob_cfg_t *cfg);

/**
 * @brief 运行龙伯格扰动观测器
 * @param obs 观测器句柄
 * @param omega_mech_rad_s 当前机械角速度 (rad/s)
 * @param iq_actual 实际 q 轴电流 (A)
 * @return q 轴补偿电流 (A)
 */
float luenbergerDOB_estimate(luenberger_dob_t *obs, float omega_mech_rad_s, float iq_actual);

float luenbergerDOB_get_d_hat(luenberger_dob_t *obs);
float luenbergerDOB_get_omega_hat(luenberger_dob_t *obs);
float luenbergerDOB_get_iq_comp(luenberger_dob_t *obs);

#endif /* __LUENBERGER_DOB_H__ */