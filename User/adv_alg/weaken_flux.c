#include <math.h>

#include "weaken_flux.h"
#include "../app/user_config.h"

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

void fluxWeaken_init(flux_weak_t *flux_weak, float u_dc, float u_ref_ratio, float kp, float ki, float id_min)
{
    flux_weak->id_ref = 0.0f;
    flux_weak->u_dc = u_dc;
    flux_weak->u_ref_ratio = u_ref_ratio;

    /* 复用 pid_controller_t 存储弱磁参数与限幅范围 */
    pid_init(&flux_weak->pid, PID_MODE_PI, kp, ki, 0.0f, id_min, 0.0f, PID_LIMIT_ENABLE);

    flux_weak->u_current_filtered = 0.0f;
    flux_weak->voltage_filter_const = FLUX_WEAK_VOLTAGE_FILTER_CONST;
}

float fluxWeak_calculate(flux_weak_t *flux_weak, float v_d, float v_q, float dt)
{
    float u_mag = sqrtf(v_d * v_d + v_q * v_q);
    float alpha = clampf(flux_weak->voltage_filter_const, 0.0f, 1.0f);
    float u_ref = flux_weak->u_dc * flux_weak->u_ref_ratio;

    flux_weak->u_current_filtered = flux_weak->u_current_filtered * (1.0f - alpha) + u_mag * alpha;

    /* 弱磁PI直接复用通用 pid_calculate()，输出限幅为 [id_min, 0] */
    flux_weak->id_ref = pid_calculate(&flux_weak->pid, u_ref, flux_weak->u_current_filtered, dt);

    return flux_weak->id_ref;
}

void fluxWeak_reset(flux_weak_t *flux_weak)
{
    flux_weak->id_ref = 0.0f;
    flux_weak->u_current_filtered = 0.0f;
    pid_reset(&flux_weak->pid);
}
