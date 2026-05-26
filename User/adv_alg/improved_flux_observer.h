#ifndef __IMPROVED_FLUX_OBSERVER_H__
#define __IMPROVED_FLUX_OBSERVER_H__

#include <math.h>
#include "../app/user_config.h"
#include "../utils/angle_utils.h"

typedef struct
{
    float rs;                  // 定子电阻 (Ω)
    float ls;                  // 定子电感 (H)，SPMSM 取 (Ld+Lq)/2
    float psi_m;               // 永磁体磁链幅值 ψ_e (Wb)
    float poles;               // 电机极对数
    float ts;                  // 观测器执行周期 (s)
    float observer_gain;       // 幅值搜索增益 λ，对应论文 Eq.(15) 中的 λ
    float phase_gain_k;        // 正交相位搜索增益 k，对应论文 Eq.(15)；k=0 退化为 Ortega
    float pll_kp;              // PLL 比例系数
    float pll_ki;              // PLL 积分系数
    float pll_speed_limit_rpm; // PLL 机械转速限幅 (rpm)，用于积分抗饱和
} improved_fluxobserver_cfg_t;

typedef struct
{
    const improved_fluxobserver_cfg_t *cfg;

    // 输入：αβ 静止坐标系下电流和上一拍输出电压
    float i_alpha;
    float i_beta;
    float u_alpha;
    float u_beta;

    // 估算的定子磁链 ψ̂_s（积分状态）
    float xhat_alpha;
    float xhat_beta;

    // 估算的转子磁链 ψ̂_r 与磁链幅值平方误差
    float psi_r_alpha;
    float psi_r_beta;
    float flux_error; // ψ_e^2 − |ψ̂_r|^2，用于诊断观测器是否收敛到磁链圆

    // 角度与速度输出
    float theta_est;   // atan2 直接角度 (rad)
    float theta_pll;   // PLL 输出角度 (rad)
    float speed_rad_s; // 估算电角速度 (rad/s)
    float speed_est;   // 估算机械转速 (rpm)

    float pll_out_limit; // PLL 输出限幅 (rad/s)，由 cfg.pll_speed_limit_rpm 换算
} improved_fluxobserver_t;

void improvedFluxObserver_init(improved_fluxobserver_t *obs, const improved_fluxobserver_cfg_t *cfg);
void improvedFluxObserver_set_initial_angle(improved_fluxobserver_t *obs, float theta_e0);
void improvedFluxObserver_estimate(improved_fluxobserver_t *obs);
float improvedFluxObserver_get_angle(improved_fluxobserver_t *obs);
float improvedFluxObserver_get_speed(improved_fluxobserver_t *obs);

#endif /* __IMPROVED_FLUX_OBSERVER_H__ */