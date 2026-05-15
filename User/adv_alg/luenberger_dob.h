#ifndef __LUENBERGER_DOB_H__
#define __LUENBERGER_DOB_H__

#include <math.h>

#include "../app/user_config.h"
#include "../utils/math_utils.h"

typedef struct
{
    float ts;
    float j;
    float damping_b;
    float kt;

    float a;
    float plant_b;
    float l1;
    float l2;

    float omega_hat;
    float d_hat;
    float iq_comp;

    float iq_comp_limit;
    float comp_gain;
} luenberger_dob_t;

void luenbergerDOB_init(luenberger_dob_t *obs, float ts, float inertia_j, float damping_b, float kt, float bandwidth_hz, float zeta, float iq_comp_limit, float comp_gain);

void luenbergerDOB_reset(luenberger_dob_t *obs, float omega_mech_rad_s);
float luenbergerDOB_update(luenberger_dob_t *obs, float omega_mech_rad_s, float iq_actual);

float luenbergerDOB_get_d_hat(luenberger_dob_t *obs);
float luenbergerDOB_get_omega_hat(luenberger_dob_t *obs);
float luenbergerDOB_get_iq_comp(luenberger_dob_t *obs);


#endif /* __LUENBERGER_DOB_H__ */