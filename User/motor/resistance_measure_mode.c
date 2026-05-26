#include "resistance_measure_mode.h"

// FOC 句柄
static foc_t foc_res_meas_handle;

// PI 控制器（仅电流环）
static pid_controller_t pid_id;
static pid_controller_t pid_iq;

// 电阻辨识状态机
static res_meas_t res_meas;

static const res_meas_cfg_t res_meas_cfg = {
    .target_current = RES_MEAS_TARGET_CURRENT,
    .ramp_rate = RES_MEAS_RAMP_RATE,
    .ts = RES_MEAS_TS_S,
    .settle_ticks = RES_MEAS_SETTLE_TICKS,
    .sample_count = RES_MEAS_SAMPLE_COUNT,
};

// 调试变量
static float id_fb_temp = 0.0f;
static float vd_out_temp = 0.0f;
static float state_temp = 0.0f;
static float resistance_temp = 0.0f;

static void resistance_measure_callback(void)
{
    encoder_update();

    // 电流采样
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // 锁定电角度为 0（d 轴对齐，电机不转）
    float angle_el = 0.0f;

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 状态机输出电流参考作为 target_id
    foc_res_meas_handle.target_id = resMeas_get_current_ref(&res_meas);
    foc_res_meas_handle.target_iq = 0.0f;

    // 电流环
    loopControl_run_currentLoop(&foc_res_meas_handle, i_dq, angle_el, 0.0f);

    // 用电流环输出电压和电流反馈驱动状态机
    resMeas_update(&res_meas, foc_res_meas_handle.v_d_out, i_dq.d);

    // 缓存调试数据
    id_fb_temp = i_dq.d;
    vd_out_temp = foc_res_meas_handle.v_d_out;
    state_temp = (float)resMeas_get_state(&res_meas);
    resistance_temp = resMeas_get_result(&res_meas);
}

void resistanceMeasureMode_init(void)
{
    pid_init(&pid_id, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f,
             CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f,
             CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);

    foc_init(&foc_res_meas_handle, &pid_id, &pid_iq, NULL);

    resMeas_init(&res_meas, &res_meas_cfg);
    resMeas_start(&res_meas);

    adc_register_injectedCallback(resistance_measure_callback);
}

void resistanceMeasureModeDebug_print_info(void)
{
    float data[4] = {state_temp, id_fb_temp, vd_out_temp, resistance_temp};
    vofa_send(data, 4);
}