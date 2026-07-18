#include "flux_observer.h"

/**
 * @brief 初始化非线性磁链观测器
 */
void fluxObserver_init(fluxobserver_t *obs, fluxobserver_cfg_t *cfg) {
    obs->cfg = cfg;

    // 初始化状态量为0
    obs->xhat_alpha = 0.0f;
    obs->xhat_beta  = 0.0f;

    obs->theta_est = 0.0f;

    // PLL内部使用电角度(rad)和电角速度(rad/s)
    phase_pll_config_t pll_config = {
        .kp                = cfg->pll_kp,
        .ki                = cfg->pll_ki,
        .sample_time_s     = cfg->ts,
        .speed_limit_rad_s = cfg->pll_speed_limit_rpm * MATH_TWO_PI * cfg->poles / 60.0f,
    };
    phasePll_init(&obs->pll, &pll_config, 0.0f);
}

/**
 * @brief 运行非线性磁链观测器
 */
void fluxObserver_estimate(fluxobserver_t *obs, alphabeta_t current, alphabeta_t applied_voltage) {
    fluxobserver_cfg_t *cfg = obs->cfg;

    // 转子磁链
    // η = xhat - L*i
    float eta_alpha = obs->xhat_alpha - cfg->ls * current.alpha;
    float eta_beta  = obs->xhat_beta - cfg->ls * current.beta;

    // 计算观测的磁链模长的平方
    // r2 = ||η||^2 = η_alpha^2 + η_beta^2，观测的磁链模长的平方
    // s = psi_m^2 - r2
    float psi_m2 = cfg->psi_m * cfg->psi_m; // 实际磁链模长的平方
    float r2     = eta_alpha * eta_alpha + eta_beta * eta_beta;
    float s      = psi_m2 - r2;

    // 按前向欧拉计算下一拍定子磁链状态
    float xhat_alpha_next = obs->xhat_alpha + cfg->ts * (applied_voltage.alpha - cfg->rs * current.alpha + 0.5f * cfg->gamma * eta_alpha * s);
    float xhat_beta_next  = obs->xhat_beta + cfg->ts * (applied_voltage.beta - cfg->rs * current.beta + 0.5f * cfg->gamma * eta_beta * s);

    // 更新下一拍磁链状态
    obs->xhat_alpha = xhat_alpha_next;
    obs->xhat_beta  = xhat_beta_next;

    // 估算值来计算角度
    // theta_hat = atan2(η_beta, η_alpha)
    obs->theta_est = atan2f(eta_beta, eta_alpha);

    phasePll_update(&obs->pll, obs->theta_est);
}

/**
 * @brief 获取估算的角度 (rad)
 * @note 返回PLL平滑角度，并保持原接口的[-π, π]范围
 */
float fluxObserver_get_angle(fluxobserver_t *obs) {
    return wrap_neg_pi_to_pi(phasePll_get_phase(&obs->pll));
}

/**
 * @brief 获取估算的机械转速 (rpm)
 */
float fluxObserver_get_speed(fluxobserver_t *obs) {
    return phasePll_get_speed(&obs->pll) * 60.0f / (MATH_TWO_PI * obs->cfg->poles);
}
