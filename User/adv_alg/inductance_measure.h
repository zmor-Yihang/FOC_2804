#ifndef __INDUCTANCE_MEASURE_H__
#define __INDUCTANCE_MEASURE_H__

#include <stdint.h>

typedef enum
{
    IND_MEAS_IDLE = 0,
    IND_MEAS_REST,
    IND_MEAS_PULSE,
    IND_MEAS_DONE,
    IND_MEAS_FAULT
} ind_meas_state_t;

typedef enum
{
    IND_MEAS_AXIS_D = 0,
    IND_MEAS_AXIS_Q
} ind_meas_axis_t;

typedef enum
{
    IND_MEAS_FAULT_NONE = 0,
    IND_MEAS_FAULT_BAD_CONFIG,
    IND_MEAS_FAULT_OVER_CURRENT,
    IND_MEAS_FAULT_NO_VALID_SAMPLE
} ind_meas_fault_t;

typedef struct
{
    float voltage;            // 脉冲电压幅值(V)，VESC duty 等效为 duty * Udc / sqrt(3)
    float phase_resistance;   // 相电阻(Ω)，用于扣除 R*i 压降
    float ts;                 // 执行周期(s)
    float result_scale;       // 结果缩放，VESC 默认偏保守取 0.9
    float min_delta_current;  // 有效电流增量阈值(A)
    float max_abs_current;    // 保护电流阈值(A)，<=0 表示关闭
    uint16_t pulse_ticks;     // 单个正/负脉冲持续拍数
    uint16_t rest_ticks;      // 脉冲之间零矢量等待拍数
    uint16_t sample_count;    // 每个轴累计的正负脉冲对数
    uint8_t measure_q_axis;   // 1: 同时辨识 Lq 和 Lq-Ld；0: 只辨识 Ld
} ind_meas_cfg_t;

typedef struct
{
    const ind_meas_cfg_t *cfg;

    ind_meas_state_t state;
    ind_meas_fault_t fault;
    ind_meas_axis_t axis;
    int8_t polarity;

    uint16_t tick_counter;
    uint16_t pair_count;

    float start_current;
    float v_d_ref;
    float v_q_ref;

    float sum_ld;
    float sum_lq;
    uint16_t valid_d_samples;
    uint16_t valid_q_samples;

    float current_delta_sum;
    uint16_t current_delta_samples;

    float ld;
    float lq;
    float inductance;
    float ld_lq_diff;
    float current_used;

    float last_delta_current;
    float last_l_sample;
} ind_meas_t;

void indMeas_init(ind_meas_t *im, const ind_meas_cfg_t *cfg);
void indMeas_start(ind_meas_t *im);
void indMeas_update(ind_meas_t *im, float id_fb, float iq_fb);

ind_meas_state_t indMeas_get_state(const ind_meas_t *im);
ind_meas_fault_t indMeas_get_fault(const ind_meas_t *im);
ind_meas_axis_t indMeas_get_axis(const ind_meas_t *im);
float indMeas_get_vd_ref(const ind_meas_t *im);
float indMeas_get_vq_ref(const ind_meas_t *im);
float indMeas_get_ld(const ind_meas_t *im);
float indMeas_get_lq(const ind_meas_t *im);
float indMeas_get_inductance(const ind_meas_t *im);
float indMeas_get_ld_lq_diff(const ind_meas_t *im);
float indMeas_get_current_used(const ind_meas_t *im);
float indMeas_get_last_delta_current(const ind_meas_t *im);
float indMeas_get_last_l_sample(const ind_meas_t *im);

#endif /* __INDUCTANCE_MEASURE_H__ */