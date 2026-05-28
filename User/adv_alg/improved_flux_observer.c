#include "improved_flux_observer.h"

/**
 * @brief 初始化观测器
 * @note  ψ̂_s 默认初值取 (ψ_e, 0)，等价于初始角度 0；若已知初始角度，
 *        建议在 init 之后再调用 improvedFluxObserver_set_initial_angle
 */
void improvedFluxObserver_init(improved_fluxobserver_t *obs, const improved_fluxobserver_cfg_t *cfg)
{
    obs->cfg = cfg;

    obs->i_alpha = 0.0f;
    obs->i_beta = 0.0f;
    obs->u_alpha = 0.0f;
    obs->u_beta = 0.0f;

    obs->xhat_alpha = cfg->psi_m;
    obs->xhat_beta = 0.0f;
    obs->psi_r_alpha = cfg->psi_m;
    obs->psi_r_beta = 0.0f;
    obs->flux_error = 0.0f;

    obs->theta_est = 0.0f;
    obs->theta_pll = 0.0f;
    obs->speed_rad_s = 0.0f;
    obs->speed_est = 0.0f;

    // 机械转速限幅(rpm) → 电角速度限幅(rad/s)
    obs->pll_out_limit = cfg->pll_speed_limit_rpm * MATH_TWO_PI * cfg->poles / 60.0f;
}

/**
 * @brief 按论文 Eq.(17) 设定初始磁链方向
 * @note  ψ̂_s = L_s · i_αβ + ψ_e · [cosθ, sinθ]^T
 *        L_s · i_αβ 是定子电感产生的磁链分量，不能丢，否则 ψ̂_r 初值会偏离磁链圆
 */
void improvedFluxObserver_set_initial_angle(improved_fluxobserver_t *obs, float theta_e0)
{
    const improved_fluxobserver_cfg_t *cfg = obs->cfg;
    float theta = wrap_neg_pi_to_pi(theta_e0);

    obs->xhat_alpha = cfg->ls * obs->i_alpha + cfg->psi_m * cosf(theta);
    obs->xhat_beta = cfg->ls * obs->i_beta + cfg->psi_m * sinf(theta);
    obs->psi_r_alpha = cfg->psi_m * cosf(theta);
    obs->psi_r_beta = cfg->psi_m * sinf(theta);
    obs->flux_error = 0.0f;
    obs->theta_est = theta;
    obs->theta_pll = theta;
}

/**
 * @brief 运行一拍观测器
 *
 * 计算流程：
 *   ψ̂_r        ← ψ̂_s − L_s · i_αβ
 *   |ψ̂_r|^2   = ψ̂_rα^2 + ψ̂_rβ^2
 *   flux_error = ψ_e^2 − |ψ̂_r|^2                       磁链圆误差
 *   ψ̂_rj      = [−ψ̂_rβ, ψ̂_rα]                          转子磁链旋转 +π/2
 *   search    = ψ̂_r + k · ψ̂_rj                         混合搜索方向（幅值 + 相位）
 *   y         = u_αβ − R_s · i_αβ                      电压方程开环项
 *   ψ̂_s     ← ψ̂_s + Ts · (y + 0.5·λ·search·flux_error) 前向 Euler 积分 Eq.(15)
 *   theta_est ← atan2(ψ̂_rβ, ψ̂_rα)                      Eq.(8)
 *   PLL 跟踪 theta_est，输出 theta_pll 与 speed_rad_s
 *
 * 关键点：
 *   - 搜索方向中的 0.5·λ 来自 Eq.(15) 的 λ/2
 *   - flux_error 收敛到 0 即表示 |ψ̂_r| 锁在磁链圆 ψ_e 上
 *   - ψ̂_r 在状态更新前后各算一次：前者用于构造修正项，后者用于角度提取与诊断
 */
void improvedFluxObserver_estimate(improved_fluxobserver_t *obs)
{
    const improved_fluxobserver_cfg_t *cfg = obs->cfg;

    // ψ̂_r = ψ̂_s − L_s · i_αβ
    obs->psi_r_alpha = obs->xhat_alpha - cfg->ls * obs->i_alpha;
    obs->psi_r_beta = obs->xhat_beta - cfg->ls * obs->i_beta;

    float psi_r2 = obs->psi_r_alpha * obs->psi_r_alpha + obs->psi_r_beta * obs->psi_r_beta;
    float psi_m2 = cfg->psi_m * cfg->psi_m;
    float flux_error = psi_m2 - psi_r2;

    // ψ̂_rj = ψ̂_r · e^{jπ/2}，正交相位搜索方向（Eq.(16)）
    float psi_rj_alpha = -obs->psi_r_beta;
    float psi_rj_beta = obs->psi_r_alpha;

    // 幅值搜索方向 + 相位搜索方向，Eq.(15) 中的 (ψ̂_r + k·ψ̂_rj)
    float search_alpha = obs->psi_r_alpha + cfg->phase_gain_k * psi_rj_alpha;
    float search_beta = obs->psi_r_beta + cfg->phase_gain_k * psi_rj_beta;

    // 电压方程开环项 y = u − R_s · i
    float y_alpha = obs->u_alpha - cfg->rs * obs->i_alpha;
    float y_beta = obs->u_beta - cfg->rs * obs->i_beta;

    // 前向 Euler 积分 Eq.(15)
    obs->xhat_alpha += cfg->ts * (y_alpha + 0.5f * cfg->observer_gain * search_alpha * flux_error);
    obs->xhat_beta += cfg->ts * (y_beta + 0.5f * cfg->observer_gain * search_beta * flux_error);

    // 用更新后的 ψ̂_s 重新计算 ψ̂_r、磁链误差和角度
    obs->psi_r_alpha = obs->xhat_alpha - cfg->ls * obs->i_alpha;
    obs->psi_r_beta = obs->xhat_beta - cfg->ls * obs->i_beta;
    obs->flux_error = psi_m2 - (obs->psi_r_alpha * obs->psi_r_alpha + obs->psi_r_beta * obs->psi_r_beta);
    obs->theta_est = atan2f(obs->psi_r_beta, obs->psi_r_alpha);

    // PLL 跟踪 theta_est，输出平滑角度 theta_pll 与电角速度 speed_rad_s
    float e_theta = wrap_neg_pi_to_pi(obs->theta_est - obs->theta_pll);
    float speed_integral_step = cfg->pll_ki * e_theta * cfg->ts;

    // 条件抗饱和：达到限幅后只允许反向积分释放
    if (!((obs->speed_rad_s >= obs->pll_out_limit && speed_integral_step > 0.0f) ||
          (obs->speed_rad_s <= -obs->pll_out_limit && speed_integral_step < 0.0f)))
    {
        obs->speed_rad_s += speed_integral_step;

        if (obs->speed_rad_s > obs->pll_out_limit)
        {
            obs->speed_rad_s = obs->pll_out_limit;
        }
        else if (obs->speed_rad_s < -obs->pll_out_limit)
        {
            obs->speed_rad_s = -obs->pll_out_limit;
        }
    }

    obs->theta_pll = wrap_neg_pi_to_pi(obs->theta_pll + (obs->speed_rad_s + cfg->pll_kp * e_theta) * cfg->ts);
    obs->speed_est = obs->speed_rad_s * 60.0f / (MATH_TWO_PI * cfg->poles);
}

/**
 * @brief 获取观测电角度
 * @note  返回 PLL 输出 theta_pll，相比 atan2 直接角度更平滑、不会跳变
 */
float improvedFluxObserver_get_angle(improved_fluxobserver_t *obs)
{
    return obs->theta_pll;
}

/**
 * @brief 获取观测机械转速 (rpm)
 */
float improvedFluxObserver_get_speed(improved_fluxobserver_t *obs)
{
    return obs->speed_est;
}