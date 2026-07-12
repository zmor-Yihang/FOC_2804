#ifndef __WEAKEN_FLUX_H__
#define __WEAKEN_FLUX_H__

#include "../alg/pid.h"

typedef struct flux_weak {
    float id_ref;
    float u_dc;
    float u_ref_ratio;

    float u_current_filtered;
    float voltage_filter_const;

    pid_controller_t pid;
} flux_weak_t;

void  fluxWeaken_init(flux_weak_t *flux_weak, float u_dc, float u_ref_ratio, float kp, float ki, float id_min);
float fluxWeak_calculate(flux_weak_t *flux_weak, float v_d, float v_q, float dt);
void  fluxWeak_reset(flux_weak_t *flux_weak);

#endif /* __WEAKEN_FLUX_H__ */
