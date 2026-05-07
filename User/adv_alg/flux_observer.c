#include "flux_observer.h"

/**
 * @brief 初始化非线性磁链观测器
 */
void fluxObserver_init(fluxobserver_t *obs, const fluxobserver_cfg_t *cfg)
{
    obs->cfg = cfg;

    // 初始化状态量为0
    obs->xhat_alpha = 0.0f;
    obs->xhat_beta = 0.0f;

    // 初始化角度和速度
    obs->theta_est = 0.0f;
    obs->z1 = 0.0f; // PLL角度状态
    obs->z2 = 0.0f; // PLL积分状态
    obs->speed_rad_s = 0.0f;
    obs->speed_est = 0.0f;

    // 初始化 PLL PI 参数
    obs->k_pll_kp = cfg->pll_kp;
    obs->k_pll_ki = cfg->pll_ki;

    // 速度限幅：机械转速(rpm) -> 电角速度(rad/s)
    float max_speed_rad_s = cfg->pll_speed_limit_rpm * MATH_TWO_PI * cfg->poles / 60.0f;
    obs->pll_out_limit = max_speed_rad_s;

    // 初始化输入为0
    obs->i_alpha = 0.0f;
    obs->i_beta = 0.0f;
    obs->u_alpha = 0.0f;
    obs->u_beta = 0.0f;
}

/**
 * @brief 运行非线性磁链观测器
 */
void fluxObserver_estimate(fluxobserver_t *obs)
{
    const fluxobserver_cfg_t *cfg = obs->cfg;

    // y = -Rs*i + u
    float y_alpha = -cfg->rs * obs->i_alpha + obs->u_alpha;
    float y_beta = -cfg->rs * obs->i_beta + obs->u_beta;

    // η = xhat - L*i
    float eta_alpha = obs->xhat_alpha - cfg->ls * obs->i_alpha;
    float eta_beta = obs->xhat_beta - cfg->ls * obs->i_beta;

    // r2 = ||η||^2 = η_alpha^2 + η_beta^2，观测的磁链模长的平方
    float r2 = eta_alpha * eta_alpha + eta_beta * eta_beta;

    // s = psi_m^2 - r2
    float psi_m2 = cfg->psi_m * cfg->psi_m; // 实际磁链模长的平方
    float s = psi_m2 - r2;                  // 误差项，s越大表示估计越偏离实际磁链圆

    // dxhat = y + 0.5 * gamma * η * s
    float dxhat_alpha = y_alpha + 0.5f * cfg->gamma * eta_alpha * s;
    float dxhat_beta = y_beta + 0.5f * cfg->gamma * eta_beta * s;

    // xhat[k+1] = xhat[k] + Ts * dxhat
    // 更新下一拍的xhat，实际就是积分，会有积分偏移，待改进
    obs->xhat_alpha += cfg->ts * dxhat_alpha;
    obs->xhat_beta += cfg->ts * dxhat_beta;

    // 使用 η 作为磁链估算值来计算角度
    // theta_hat = atan2(η_beta, η_alpha)
    obs->theta_est = atan2f(eta_beta, eta_alpha);

    // 角度误差 e_theta = theta_hat - z1
    float e_theta = wrap_neg_pi_to_pi(obs->theta_est - obs->z1);
    float speed_integral_step = obs->k_pll_ki * e_theta * cfg->ts;

    // 与 encoder PLL 保持一致：到达限幅后仅允许反向积分释放，避免积分器继续累积
    if (!((obs->speed_rad_s >= obs->pll_out_limit && speed_integral_step > 0.0f) ||
          (obs->speed_rad_s <= -obs->pll_out_limit && speed_integral_step < 0.0f)))
    {
        obs->speed_rad_s += speed_integral_step;

        if (obs->speed_rad_s > obs->pll_out_limit)
            obs->speed_rad_s = obs->pll_out_limit;
        else if (obs->speed_rad_s < -obs->pll_out_limit)
            obs->speed_rad_s = -obs->pll_out_limit;
    }

    // 每个控制周期都使用比例校正项 + 当前速度估计推进 PLL 相位
    obs->z1 = wrap_neg_pi_to_pi(obs->z1 + (obs->speed_rad_s + obs->k_pll_kp * e_theta) * cfg->ts);
    obs->z2 = obs->speed_rad_s;

    // 电角速度 -> 机械转速 (rpm)
    // omega_mech = omega_elec / poles
    // rpm = omega_mech * 60 / (2*pi) = omega_elec * 60 / (2*pi*poles)
    float speed_rpm = obs->speed_rad_s * 60.0f / (MATH_TWO_PI * cfg->poles);
    obs->speed_est = speed_rpm;
}

/**
 * @brief 获取估算的角度 (rad)
 * @note 返回PLL输出的平滑角度z1，而非直接从磁链计算的theta_est
 *       这样可以保证角度连续平滑，避免静差问题
 */
float fluxObserver_get_angle(fluxobserver_t *obs)
{
    return obs->z1;
}

/**
 * @brief 获取估算的速度 (rpm)
 */
float fluxObserver_get_speed(fluxobserver_t *obs)
{
    return obs->speed_est;
}
