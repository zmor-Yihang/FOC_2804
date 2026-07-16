#include "mxlemming_observer.h"

void mxlemmingObserver_init(mxlemming_obs_t *obs, const mxlemming_cfg_t *cfg) {
    obs->cfg = cfg;

    obs->x1 = 0.0f;
    obs->x2 = 0.0f;

    obs->i_alpha_last = 0.0f;
    obs->i_beta_last  = 0.0f;

    obs->theta_est = 0.0f;

    const phase_pll_config_t pll_config = {
        .kp                = cfg->pll_kp,
        .ki                = cfg->pll_ki,
        .sample_time_s     = cfg->ts,
        .speed_limit_rad_s = cfg->pll_speed_limit_rpm * MATH_TWO_PI * cfg->poles / 60.0f,
    };
    phasePll_init(&obs->pll, &pll_config, 0.0f);
}

void mxlemmingObserver_update(mxlemming_obs_t *obs, alphabeta_t current, alphabeta_t applied_voltage) {
    const mxlemming_cfg_t *cfg = obs->cfg;
    float                  dt  = cfg->ts;

    // 电流差分法积分
    obs->x1 += (applied_voltage.alpha - cfg->rs * current.alpha) * dt - cfg->ls * (current.alpha - obs->i_alpha_last);
    obs->x2 += (applied_voltage.beta - cfg->rs * current.beta) * dt - cfg->ls * (current.beta - obs->i_beta_last);

    // 保存当前电流供下一拍差分
    obs->i_alpha_last = current.alpha;
    obs->i_beta_last  = current.beta;

    // 幅值约束：圆形 clamp，半径 lambda
    float r2      = obs->x1 * obs->x1 + obs->x2 * obs->x2;
    float lambda2 = cfg->lambda * cfg->lambda;
    if (r2 > lambda2) {
        float scale = cfg->lambda / sqrtf(r2);
        obs->x1 *= scale;
        obs->x2 *= scale;
    }

    // 角度提取
    obs->theta_est = atan2f(obs->x2, obs->x1);

    phasePll_update(&obs->pll, obs->theta_est);
}

float mxlemmingObserver_get_angle(const mxlemming_obs_t *obs) {
    return wrap_neg_pi_to_pi(phasePll_get_phase(&obs->pll));
}

float mxlemmingObserver_get_speed(const mxlemming_obs_t *obs) {
    return phasePll_get_speed(&obs->pll) * 60.0f / (MATH_TWO_PI * obs->cfg->poles);
}