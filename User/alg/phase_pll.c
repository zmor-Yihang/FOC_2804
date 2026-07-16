#include "phase_pll.h"

void phasePll_init(phase_pll_t *pll, phase_pll_config_t *config, float initial_phase_rad) {
    pll->config          = *config;
    pll->phase_rad       = wrap_0_2pi(initial_phase_rad);
    pll->speed_rad_s     = 0.0f;
    pll->phase_error_rad = 0.0f;
}

/**
 * @brief 使用输入相位执行一次PLL状态迭代
 */
void phasePll_update(phase_pll_t *pll, float input_phase_rad) {

    // 采样时间，对应积分环节的时间离散化
    float dt = pll->config.sample_time_s;

    // 计算相位误差
    pll->phase_error_rad = wrap_neg_pi_to_pi(input_phase_rad - pll->phase_rad);

    // 积分项累加
    pll->speed_rad_s += pll->config.ki * pll->phase_error_rad * dt;

    // 积分限幅
    pll->speed_rad_s = utils_clampf(pll->speed_rad_s, -pll->config.speed_limit_rad_s, pll->config.speed_limit_rad_s);

    // PI 控制器输出
    float phase_rate_rad_s = pll->speed_rad_s + pll->config.kp * pll->phase_error_rad;

    // 对PI输出积分，得到相位
    pll->phase_rad               = wrap_0_2pi(pll->phase_rad + phase_rate_rad_s * dt);
}

float phasePll_get_phase(phase_pll_t *pll) {
    return pll->phase_rad;
}

// 注意这里返回的是积分累加值，不是PI控制器输出，
// PLL稳态时，积分累加值等于PI控制器输出。
// 动态时，只用积分项，速度平滑过渡
// VESC和ODrive的PLL都是这样处理的
// 很多文章的PLL部分是有问题的，动态时PLL的PI控制器输出无实际意义，并不是速度
// 只有稳态时，PLL的PI控制器输出（也就是积分累加值）才是速度
float phasePll_get_speed(phase_pll_t *pll) {
    return pll->speed_rad_s;
}