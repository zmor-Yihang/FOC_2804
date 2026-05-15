#include "luenberger_dob.h"

/**
 * 速度环负载扰动观测器。
 *
 * 机械模型：
 *   omega_dot = a * omega + b * iq + d
 *
 * 其中：
 *   a = -B / J
 *   b = Kt / J
 *   d = -TL / J
 *
 * d_hat 估计的是机械加速度维度的总扰动项。若正转时外部负载增大，
 * 通常 d_hat 会变负；补偿电流按 iq_comp = -(J / Kt) * d_hat 换算，
 * 因此会得到正向 q 轴电流补偿。
 */
void luenbergerDOB_init(luenberger_dob_t *obs, float ts, float inertia_j, float damping_b, float kt, float bandwidth_hz, float zeta, float iq_comp_limit, float comp_gain)
{
    obs->ts = ts;
    obs->j = inertia_j;
    obs->damping_b = damping_b;
    obs->kt = kt;

    obs->a = -damping_b / inertia_j;
    obs->plant_b = kt / inertia_j;

    /*
     * 按二阶特征多项式配置观测器极点：
     *   s^2 + 2*zeta*omega_o*s + omega_o^2
     * 对应增益：
     *   l1 = 2*zeta*omega_o + a
     *   l2 = omega_o^2
     */
    float omega_o = MATH_TWO_PI * bandwidth_hz;
    obs->l1 = 2.0f * zeta * omega_o + obs->a;
    obs->l2 = omega_o * omega_o;

    obs->iq_comp_limit = fabsf(iq_comp_limit);
    obs->comp_gain = comp_gain;

    luenbergerDOB_reset(obs, 0.0f);
}

void luenbergerDOB_reset(luenberger_dob_t *obs, float omega_mech_rad_s)
{
    /* 使能或重新对齐后，用当前实测速度初始化 omega_hat，避免启动瞬间产生估计误差冲击 */
    obs->omega_hat = omega_mech_rad_s;
    obs->d_hat = 0.0f;
    obs->iq_comp = 0.0f;
}

float luenbergerDOB_update(luenberger_dob_t *obs, float omega_mech_rad_s, float iq_actual)
{
    float e = omega_mech_rad_s - obs->omega_hat;

    float omega_hat_dot = obs->a * obs->omega_hat + obs->plant_b * iq_actual + obs->d_hat + obs->l1 * e;
    float d_hat_dot = obs->l2 * e;

    obs->omega_hat += obs->ts * omega_hat_dot;
    obs->d_hat += obs->ts * d_hat_dot;

    /* d_hat = -TL/J，因此抵消该扰动所需的 q 轴电流为 -(J/Kt)*d_hat。 */
    obs->iq_comp = -(obs->j / obs->kt) * obs->d_hat * obs->comp_gain;
    obs->iq_comp = utils_clampf(obs->iq_comp, -obs->iq_comp_limit, obs->iq_comp_limit);

    return obs->iq_comp;
}

float luenbergerDOB_get_d_hat(luenberger_dob_t *obs)
{
    return obs->d_hat;
}

float luenbergerDOB_get_omega_hat(luenberger_dob_t *obs)
{
    return obs->omega_hat;
}

float luenbergerDOB_get_iq_comp(luenberger_dob_t *obs)
{
    return obs->iq_comp;
}