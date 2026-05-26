#include "mxlemming_observer.h"

void mxlemmingObserver_init(mxlemming_obs_t *obs, const mxlemming_cfg_t *cfg)
{
    obs->cfg = cfg;

    obs->x1 = 0.0f;
    obs->x2 = 0.0f;

    obs->i_alpha_last = 0.0f;
    obs->i_beta_last = 0.0f;

    obs->i_alpha = 0.0f;
    obs->i_beta = 0.0f;
    obs->u_alpha = 0.0f;
    obs->u_beta = 0.0f;

    obs->theta_est = 0.0f;
    obs->theta_pll = 0.0f;
    obs->speed_rad_s = 0.0f;
    obs->speed_est = 0.0f;

    obs->pll_out_limit = cfg->pll_speed_limit_rpm * MATH_TWO_PI * cfg->poles / 60.0f;
}

void mxlemmingObserver_update(mxlemming_obs_t *obs)
{
    const mxlemming_cfg_t *cfg = obs->cfg;
    float dt = cfg->ts;

    // 电流差分法积分
    obs->x1 += (obs->u_alpha - cfg->rs * obs->i_alpha) * dt - cfg->ls * (obs->i_alpha - obs->i_alpha_last);
    obs->x2 += (obs->u_beta - cfg->rs * obs->i_beta) * dt - cfg->ls * (obs->i_beta - obs->i_beta_last);

    // 保存当前电流供下一拍差分
    obs->i_alpha_last = obs->i_alpha;
    obs->i_beta_last = obs->i_beta;

    // 幅值约束：圆形 clamp，半径 lambda
    float r2 = obs->x1 * obs->x1 + obs->x2 * obs->x2;
    float lambda2 = cfg->lambda * cfg->lambda;
    if (r2 > lambda2)
    {
        float scale = cfg->lambda / sqrtf(r2);
        obs->x1 *= scale;
        obs->x2 *= scale;
    }

    // 角度提取
    obs->theta_est = atan2f(obs->x2, obs->x1);

    // PLL 跟踪
    float e_theta = wrap_neg_pi_to_pi(obs->theta_est - obs->theta_pll);
    float speed_integral_step = cfg->pll_ki * e_theta * dt;

    // 条件积分抗饱和
    if (!((obs->speed_rad_s >= obs->pll_out_limit && speed_integral_step > 0.0f) ||
          (obs->speed_rad_s <= -obs->pll_out_limit && speed_integral_step < 0.0f)))
    {
        obs->speed_rad_s += speed_integral_step;
        if (obs->speed_rad_s > obs->pll_out_limit)
            obs->speed_rad_s = obs->pll_out_limit;
        else if (obs->speed_rad_s < -obs->pll_out_limit)
            obs->speed_rad_s = -obs->pll_out_limit;
    }

    // PLL 角度推进：比例校正 + 速度积分
    obs->theta_pll = wrap_neg_pi_to_pi(obs->theta_pll + (obs->speed_rad_s + cfg->pll_kp * e_theta) * dt);

    // 电角速度 -> 机械转速 (rpm)
    obs->speed_est = obs->speed_rad_s * 60.0f / (MATH_TWO_PI * cfg->poles);
}

float mxlemmingObserver_get_angle(mxlemming_obs_t *obs)
{
    return obs->theta_pll;
}

float mxlemmingObserver_get_speed(mxlemming_obs_t *obs)
{
    return obs->speed_est;
}