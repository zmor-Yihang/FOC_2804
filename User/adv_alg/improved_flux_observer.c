#include "improved_flux_observer.h"

/**
 * @brief 初始化改进非线性磁链观测器
 */
void improvedFluxObserver_init(improved_fluxobserver_t *obs, improved_fluxobserver_cfg_t *cfg) {
    obs->cfg = cfg;

    // 初始化状态量，默认转子磁链角度为0
    obs->xhat_alpha = cfg->psi_m;
    obs->xhat_beta  = 0.0f;
    obs->theta_est  = 0.0f;

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
 * @brief 运行改进非线性磁链观测器
 */
void improvedFluxObserver_estimate(improved_fluxobserver_t *obs, alphabeta_t current, alphabeta_t applied_voltage) {
    improved_fluxobserver_cfg_t *cfg = obs->cfg;

    // 计算当前拍转子磁链估计：psi_r = psi_s - Ls * i
    float psi_r_alpha = obs->xhat_alpha - cfg->ls * current.alpha;
    float psi_r_beta  = obs->xhat_beta - cfg->ls * current.beta;

    // 磁链圆约束误差：psi_m^2 - |psi_r|^2
    float psi_r2 = psi_r_alpha * psi_r_alpha + psi_r_beta * psi_r_beta;
    float psi_m2 = cfg->psi_m * cfg->psi_m;
    float s      = psi_m2 - psi_r2;

    // 转子磁链旋转90度，构造正交相位搜索方向
    float psi_rj_alpha = -psi_r_beta;
    float psi_rj_beta  = psi_r_alpha;

    // 幅值搜索方向与正交相位搜索方向叠加
    float search_alpha = psi_r_alpha + cfg->phase_gain_k * psi_rj_alpha;
    float search_beta  = psi_r_beta + cfg->phase_gain_k * psi_rj_beta;

    // 按 sqrt(1 + k²) 归一化，保持合成搜索方向的模长量级
    float normalized_observer_gain = cfg->observer_gain / sqrtf(1.0f + cfg->phase_gain_k * cfg->phase_gain_k);

    // 按前向欧拉计算下一拍定子磁链状态
    float xhat_alpha_next = obs->xhat_alpha + cfg->ts * (applied_voltage.alpha - cfg->rs * current.alpha + 0.5f * normalized_observer_gain * search_alpha * s);
    float xhat_beta_next  = obs->xhat_beta + cfg->ts * (applied_voltage.beta - cfg->rs * current.beta + 0.5f * normalized_observer_gain * search_beta * s);

    // 更新下一拍定子磁链状态
    obs->xhat_alpha = xhat_alpha_next;
    obs->xhat_beta  = xhat_beta_next;

    // 使用当前拍转子磁链计算角度
    obs->theta_est = atan2f(psi_r_beta, psi_r_alpha);

    phasePll_update(&obs->pll, obs->theta_est);
}

/**
 * @brief 获取观测电角度
 * @note 返回PLL平滑角度，并保持原接口的[-pi, pi]范围
 */
float improvedFluxObserver_get_angle(improved_fluxobserver_t *obs) {
    return wrap_neg_pi_to_pi(phasePll_get_phase(&obs->pll));
}

/**
 * @brief 获取观测机械转速 (rpm)
 */
float improvedFluxObserver_get_speed(improved_fluxobserver_t *obs) {
    return phasePll_get_speed(&obs->pll) * 60.0f / (MATH_TWO_PI * obs->cfg->poles);
}