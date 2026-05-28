#include "inductance_measure.h"

#include <math.h>

static uint8_t indMeas_config_is_valid(const ind_meas_cfg_t *cfg)
{
    return (cfg != 0) &&
           (cfg->voltage > 0.0f) &&
           (cfg->ts > 0.0f) &&
           (cfg->pulse_ticks > 0U) &&
           (cfg->sample_count > 0U) &&
           (cfg->result_scale > 0.0f) &&
           (cfg->min_delta_current >= 0.0f);
}

static void indMeas_clear(ind_meas_t *im)
{
    im->state = IND_MEAS_IDLE;
    im->fault = IND_MEAS_FAULT_NONE;
    im->axis = IND_MEAS_AXIS_D;
    im->polarity = 1;

    im->tick_counter = 0U;
    im->pair_count = 0U;

    im->start_current = 0.0f;
    im->v_d_ref = 0.0f;
    im->v_q_ref = 0.0f;

    im->sum_ld = 0.0f;
    im->sum_lq = 0.0f;
    im->valid_d_samples = 0U;
    im->valid_q_samples = 0U;

    im->current_delta_sum = 0.0f;
    im->current_delta_samples = 0U;

    im->ld = 0.0f;
    im->lq = 0.0f;
    im->inductance = 0.0f;
    im->ld_lq_diff = 0.0f;
    im->current_used = 0.0f;

    im->last_delta_current = 0.0f;
    im->last_l_sample = 0.0f;
}

static void indMeas_set_zero_voltage(ind_meas_t *im)
{
    im->v_d_ref = 0.0f;
    im->v_q_ref = 0.0f;
}

static void indMeas_set_pulse_voltage(ind_meas_t *im)
{
    float v = (float)im->polarity * im->cfg->voltage;

    if (im->axis == IND_MEAS_AXIS_D)
    {
        im->v_d_ref = v;
        im->v_q_ref = 0.0f;
    }
    else
    {
        im->v_d_ref = 0.0f;
        im->v_q_ref = v;
    }
}

static float indMeas_get_axis_current(const ind_meas_t *im, float id_fb, float iq_fb)
{
    return (im->axis == IND_MEAS_AXIS_D) ? id_fb : iq_fb;
}

static void indMeas_finish(ind_meas_t *im)
{
    if ((im->valid_d_samples == 0U) ||
        ((im->cfg->measure_q_axis != 0U) && (im->valid_q_samples == 0U)))
    {
        im->fault = IND_MEAS_FAULT_NO_VALID_SAMPLE;
        im->state = IND_MEAS_FAULT;
        indMeas_set_zero_voltage(im);
        return;
    }

    im->ld = im->sum_ld / (float)im->valid_d_samples;

    if (im->cfg->measure_q_axis != 0U)
    {
        im->lq = im->sum_lq / (float)im->valid_q_samples;
        im->inductance = 0.5f * (im->ld + im->lq);
        im->ld_lq_diff = im->lq - im->ld;
    }
    else
    {
        im->lq = im->ld;
        im->inductance = im->ld;
        im->ld_lq_diff = 0.0f;
    }

    if (im->current_delta_samples > 0U)
    {
        im->current_used = im->current_delta_sum / (float)im->current_delta_samples;
    }

    im->state = IND_MEAS_DONE;
    indMeas_set_zero_voltage(im);
}

static void indMeas_next_pulse(ind_meas_t *im)
{
    im->tick_counter = 0U;
    indMeas_set_zero_voltage(im);

    if (im->polarity > 0)
    {
        im->polarity = -1;
        im->state = IND_MEAS_REST;
        return;
    }

    im->pair_count++;
    im->polarity = 1;

    if (im->pair_count < im->cfg->sample_count)
    {
        im->state = IND_MEAS_REST;
        return;
    }

    if ((im->axis == IND_MEAS_AXIS_D) && (im->cfg->measure_q_axis != 0U))
    {
        im->axis = IND_MEAS_AXIS_Q;
        im->pair_count = 0U;
        im->state = IND_MEAS_REST;
        return;
    }

    indMeas_finish(im);
}

static void indMeas_accumulate_sample(ind_meas_t *im, float end_current)
{
    float delta_i = end_current - im->start_current;
    float abs_delta_i = fabsf(delta_i);
    im->last_delta_current = delta_i;

    if (abs_delta_i < im->cfg->min_delta_current)
    {
        return;
    }

    float pulse_time = im->cfg->ts * (float)im->cfg->pulse_ticks;
    float axis_voltage = (float)im->polarity * im->cfg->voltage;
    float avg_current = 0.5f * (im->start_current + end_current);
    float effective_voltage = axis_voltage - im->cfg->phase_resistance * avg_current;
    float l_sample = (effective_voltage * pulse_time / delta_i) * im->cfg->result_scale;

    if (isfinite(l_sample) && (l_sample > 0.0f))
    {
        im->last_l_sample = l_sample;

        if (im->axis == IND_MEAS_AXIS_D)
        {
            im->sum_ld += l_sample;
            im->valid_d_samples++;
        }
        else
        {
            im->sum_lq += l_sample;
            im->valid_q_samples++;
        }

        im->current_delta_sum += abs_delta_i;
        im->current_delta_samples++;
    }
}

void indMeas_init(ind_meas_t *im, const ind_meas_cfg_t *cfg)
{
    im->cfg = cfg;
    indMeas_clear(im);

    if (!indMeas_config_is_valid(cfg))
    {
        im->fault = IND_MEAS_FAULT_BAD_CONFIG;
        im->state = IND_MEAS_FAULT;
    }
}

void indMeas_start(ind_meas_t *im)
{
    const ind_meas_cfg_t *cfg = im->cfg;
    indMeas_clear(im);
    im->cfg = cfg;

    if (!indMeas_config_is_valid(cfg))
    {
        im->fault = IND_MEAS_FAULT_BAD_CONFIG;
        im->state = IND_MEAS_FAULT;
        return;
    }

    im->state = IND_MEAS_REST;
}

void indMeas_update(ind_meas_t *im, float id_fb, float iq_fb)
{
    if ((im->state == IND_MEAS_IDLE) ||
        (im->state == IND_MEAS_DONE) ||
        (im->state == IND_MEAS_FAULT))
    {
        indMeas_set_zero_voltage(im);
        return;
    }

    if ((im->cfg->max_abs_current > 0.0f) &&
        ((fabsf(id_fb) > im->cfg->max_abs_current) || (fabsf(iq_fb) > im->cfg->max_abs_current)))
    {
        im->fault = IND_MEAS_FAULT_OVER_CURRENT;
        im->state = IND_MEAS_FAULT;
        indMeas_set_zero_voltage(im);
        return;
    }

    float axis_current = indMeas_get_axis_current(im, id_fb, iq_fb);

    switch (im->state)
    {
    case IND_MEAS_REST:
        indMeas_set_zero_voltage(im);
        im->tick_counter++;
        if (im->tick_counter >= im->cfg->rest_ticks)
        {
            im->tick_counter = 0U;
            im->start_current = axis_current;
            im->state = IND_MEAS_PULSE;
            indMeas_set_pulse_voltage(im);
        }
        break;

    case IND_MEAS_PULSE:
        indMeas_set_pulse_voltage(im);
        im->tick_counter++;
        if (im->tick_counter >= im->cfg->pulse_ticks)
        {
            indMeas_accumulate_sample(im, axis_current);
            indMeas_next_pulse(im);
        }
        break;

    case IND_MEAS_IDLE:
    case IND_MEAS_DONE:
    case IND_MEAS_FAULT:
    default:
        indMeas_set_zero_voltage(im);
        break;
    }
}

ind_meas_state_t indMeas_get_state(const ind_meas_t *im)
{
    return im->state;
}

ind_meas_fault_t indMeas_get_fault(const ind_meas_t *im)
{
    return im->fault;
}

ind_meas_axis_t indMeas_get_axis(const ind_meas_t *im)
{
    return im->axis;
}

float indMeas_get_vd_ref(const ind_meas_t *im)
{
    return im->v_d_ref;
}

float indMeas_get_vq_ref(const ind_meas_t *im)
{
    return im->v_q_ref;
}

float indMeas_get_ld(const ind_meas_t *im)
{
    return im->ld;
}

float indMeas_get_lq(const ind_meas_t *im)
{
    return im->lq;
}

float indMeas_get_inductance(const ind_meas_t *im)
{
    return im->inductance;
}

float indMeas_get_ld_lq_diff(const ind_meas_t *im)
{
    return im->ld_lq_diff;
}

float indMeas_get_current_used(const ind_meas_t *im)
{
    return im->current_used;
}

float indMeas_get_last_delta_current(const ind_meas_t *im)
{
    return im->last_delta_current;
}

float indMeas_get_last_l_sample(const ind_meas_t *im)
{
    return im->last_l_sample;
}