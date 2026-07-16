#include "flux_observer.h"

/**
 * @brief 初始化非线性磁链观测器
 */
void fluxObserver_init(fluxobserver_t *obs, const fluxobserver_cfg_t *cfg) {
    obs->cfg = cfg;

    // 初始化状态量为0
    obs->xhat_alpha = 0.0f;
    obs->xhat_beta  = 0.0f;

    obs->theta_est = 0.0f;

    // PLL内部使用电角度(rad)和电角速度(rad/s)
    const phase_pll_config_t pll_config = {
        .kp                = cfg->pll_kp,
        .ki                = cfg->pll_ki,
        .sample_time_s     = cfg->ts,
        .speed_limit_rad_s = cfg->pll_speed_limit_rpm * MATH_TWO_PI * cfg->poles / 60.0f,
    };
    phasePll_init(&obs->pll, &pll_config, 0.0f);

    // 初始化输入为0
    obs->i_alpha = 0.0f;
    obs->i_beta  = 0.0f;
    obs->u_alpha = 0.0f;
    obs->u_beta  = 0.0f;
}

/**
 * @brief 运行非线性磁链观测器
 */
void fluxObserver_estimate(fluxobserver_t *obs) {
    const fluxobserver_cfg_t *cfg = obs->cfg;

    // y = -Rs*i + u
    float y_alpha = -cfg->rs * obs->i_alpha + obs->u_alpha;
    float y_beta  = -cfg->rs * obs->i_beta + obs->u_beta;

    // η = xhat - L*i
    float eta_alpha = obs->xhat_alpha - cfg->ls * obs->i_alpha;
    float eta_beta  = obs->xhat_beta - cfg->ls * obs->i_beta;

    // r2 = ||η||^2 = η_alpha^2 + η_beta^2，观测的磁链模长的平方
    float r2 = eta_alpha * eta_alpha + eta_beta * eta_beta;

    // s = psi_m^2 - r2
    float psi_m2 = cfg->psi_m * cfg->psi_m; // 实际磁链模长的平方
    float s      = psi_m2 - r2;             // 误差项，s越大表示估计越偏离实际磁链圆

    // dxhat = y + 0.5 * gamma * η * s
    float dxhat_alpha = y_alpha + 0.5f * cfg->gamma * eta_alpha * s;
    float dxhat_beta  = y_beta + 0.5f * cfg->gamma * eta_beta * s;

    // xhat[k+1] = xhat[k] + Ts * dxhat
    // 更新下一拍的xhat，实际就是积分，会有积分偏移，待改进
    obs->xhat_alpha += cfg->ts * dxhat_alpha;
    obs->xhat_beta += cfg->ts * dxhat_beta;

    // 使用 η 作为磁链估算值来计算角度
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
