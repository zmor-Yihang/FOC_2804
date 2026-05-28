#include "inductance_measure_mode.h"

static foc_t foc_ind_meas_handle;
static ind_meas_t ind_meas;

static const ind_meas_cfg_t ind_meas_cfg = {
    .voltage = IND_MEAS_VOLTAGE,
    .phase_resistance = IND_MEAS_PHASE_RESISTANCE,
    .ts = IND_MEAS_TS_S,
    .result_scale = IND_MEAS_RESULT_SCALE,
    .min_delta_current = IND_MEAS_MIN_DELTA_CURRENT,
    .max_abs_current = IND_MEAS_MAX_ABS_CURRENT,
    .pulse_ticks = IND_MEAS_PULSE_TICKS,
    .rest_ticks = IND_MEAS_REST_TICKS,
    .sample_count = IND_MEAS_SAMPLE_COUNT,
    .measure_q_axis = IND_MEAS_MEASURE_Q_AXIS,
};

static float state_temp = 0.0f;
static float fault_temp = 0.0f;
static float axis_temp = 0.0f;
static float id_fb_temp = 0.0f;
static float iq_fb_temp = 0.0f;
static float vd_ref_temp = 0.0f;
static float vq_ref_temp = 0.0f;
static float current_used_temp = 0.0f;
static float last_delta_current_temp = 0.0f;
static float last_l_sample_uH_temp = 0.0f;
static float ld_uH_temp = 0.0f;
static float lq_uH_temp = 0.0f;
static float l_avg_uH_temp = 0.0f;
static float ld_lq_diff_uH_temp = 0.0f;

static void inductance_measure_callback(void)
{
    encoder_update();

    float angle_el = wrap_0_2pi(encoder_get_pllAngle() - foc_ind_meas_handle.angle_offset);

    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);
    alphabeta_t i_alphabeta = clark_transform(i_abc);
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    indMeas_update(&ind_meas, i_dq.d, i_dq.q);

    foc_ind_meas_handle.v_d_out = indMeas_get_vd_ref(&ind_meas);
    foc_ind_meas_handle.v_q_out = indMeas_get_vq_ref(&ind_meas);

    if ((indMeas_get_state(&ind_meas) == IND_MEAS_DONE) ||
        (indMeas_get_state(&ind_meas) == IND_MEAS_FAULT))
    {
        foc_ind_meas_handle.duty_cycle = gateDrive_stop();
    }
    else
    {
        dq_t u_dq = {
            .d = foc_ind_meas_handle.v_d_out,
            .q = foc_ind_meas_handle.v_q_out,
        };
        foc_ind_meas_handle.duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, angle_el));
    }

    state_temp = (float)indMeas_get_state(&ind_meas);
    fault_temp = (float)indMeas_get_fault(&ind_meas);
    axis_temp = (float)indMeas_get_axis(&ind_meas);
    id_fb_temp = i_dq.d;
    iq_fb_temp = i_dq.q;
    vd_ref_temp = foc_ind_meas_handle.v_d_out;
    vq_ref_temp = foc_ind_meas_handle.v_q_out;
    current_used_temp = indMeas_get_current_used(&ind_meas);
    last_delta_current_temp = indMeas_get_last_delta_current(&ind_meas);
    last_l_sample_uH_temp = indMeas_get_last_l_sample(&ind_meas) * 1.0e6f;
    ld_uH_temp = indMeas_get_ld(&ind_meas) * 1.0e6f;
    lq_uH_temp = indMeas_get_lq(&ind_meas) * 1.0e6f;
    l_avg_uH_temp = indMeas_get_inductance(&ind_meas) * 1.0e6f;
    ld_lq_diff_uH_temp = indMeas_get_ld_lq_diff(&ind_meas) * 1.0e6f;
}

void inductanceMeasureMode_init(void)
{
    foc_init(&foc_ind_meas_handle, NULL, NULL, NULL);
    zero_alignment(&foc_ind_meas_handle);

    indMeas_init(&ind_meas, &ind_meas_cfg);
    indMeas_start(&ind_meas);

    adc_register_injectedCallback(inductance_measure_callback);
}

void inductanceMeasureModeDebug_print_info(void)
{
    float data[14] = {
        state_temp,
        fault_temp,
        axis_temp,
        id_fb_temp,
        iq_fb_temp,
        vd_ref_temp,
        vq_ref_temp,
        current_used_temp,
        last_delta_current_temp,
        last_l_sample_uH_temp,
        ld_uH_temp,
        lq_uH_temp,
        l_avg_uH_temp,
        ld_lq_diff_uH_temp,
    };

    vofa_send(data, 14);
}