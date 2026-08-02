#include "luenberger_dob.h"

/**
 * @brief 初始化龙伯格扰动观测器
 */
void luenbergerDOB_init(luenberger_dob_t *obs, luenberger_dob_cfg_t *cfg) {
    obs->cfg = cfg;

    // 机械模型系数
    obs->a       = -cfg->damping_b / cfg->j;
    obs->plant_b = cfg->kt / cfg->j;

    /*
     * 按二阶特征多项式配置观测器极点：
     *   s^2 + 2*zeta*omega_o*s + omega_o^2
     * 对应增益：
     *   l1 = 2*zeta*omega_o + a
     *   l2 = omega_o^2
     */
    float omega_o = MATH_TWO_PI * cfg->bandwidth_hz;
    obs->l1       = 2.0f * cfg->zeta * omega_o + obs->a;
    obs->l2       = omega_o * omega_o;

    // 初始化状态量
    obs->omega_hat = 0.0f;
    obs->d_hat     = 0.0f;
    obs->iq_comp   = 0.0f;
}

/**
 * @brief 运行龙伯格扰动观测器
 * @note d_hat = -TL/J，抵消该扰动所需的 q 轴电流为 -(J/Kt)*d_hat
 */
float luenbergerDOB_estimate(luenberger_dob_t *obs, float omega_mech_rad_s, float iq_actual) {
    luenberger_dob_cfg_t *cfg = obs->cfg;

    float e = omega_mech_rad_s - obs->omega_hat;

    // 按前向欧拉更新估计状态
    float omega_hat_dot = obs->a * obs->omega_hat + obs->plant_b * iq_actual + obs->d_hat + obs->l1 * e;
    float d_hat_dot     = obs->l2 * e;

    obs->omega_hat += cfg->ts * omega_hat_dot;
    obs->d_hat += cfg->ts * d_hat_dot;

    // 将加速度维度扰动换算为 q 轴补偿电流
    obs->iq_comp = -(cfg->j / cfg->kt) * obs->d_hat * cfg->comp_gain;
    obs->iq_comp = utils_clampf(obs->iq_comp, -fabsf(cfg->iq_comp_limit), fabsf(cfg->iq_comp_limit));

    return obs->iq_comp;
}

/**
 * @brief 获取估计扰动 (rad/s²)
 */
float luenbergerDOB_get_d_hat(luenberger_dob_t *obs) {
    return obs->d_hat;
}

/**
 * @brief 获取估计机械角速度 (rad/s)
 */
float luenbergerDOB_get_omega_hat(luenberger_dob_t *obs) {
    return obs->omega_hat;
}

/**
 * @brief 获取 q 轴补偿电流 (A)
 */
float luenbergerDOB_get_iq_comp(luenberger_dob_t *obs) {
    return obs->iq_comp;
}