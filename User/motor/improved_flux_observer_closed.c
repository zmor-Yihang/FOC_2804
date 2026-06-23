#include "improved_flux_observer_closed.h"

// FOC 句柄
static foc_t foc_improvedFlux_handle;

// PI 控制器
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 改进非线性磁链观测器
static improved_fluxobserver_t improved_observer;

static const improved_fluxobserver_cfg_t improved_observer_cfg = {
    .rs = MOTOR_RS_Ω,
    .ls = 0.5f * (MOTOR_LD_H + MOTOR_LQ_H),
    .psi_m = MOTOR_PSI_F,
    .poles = MOTOR_POLE_PAIRS,
    .ts = IMPROVED_FLUX_OBSERVER_TS_S,
    .observer_gain = IMPROVED_FLUX_OBSERVER_GAIN,
    .phase_gain_k = IMPROVED_FLUX_OBSERVER_PHASE_GAIN_K,
    .pll_kp = IMPROVED_FLUX_OBSERVER_PLL_KP,
    .pll_ki = IMPROVED_FLUX_OBSERVER_PLL_KI,
    .pll_speed_limit_rpm = IMPROVED_FLUX_OBSERVER_PLL_SPEED_LIMIT_RPM,
};

// 调试变量
static float speed_encoder_temp = 0.0f;
static float speed_obs_temp = 0.0f;
static float angle_encoder_temp = 0.0f;
static float angle_obs_temp = 0.0f;
static float id_temp = 0.0f;
static float iq_temp = 0.0f;

static void improvedFlux_closed_callback(void)
{
    encoder_update();

    // 电流采样
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // 更新观测器输入：电流与上一拍输出电压
    improved_observer.i_alpha = i_alphabeta.alpha;
    improved_observer.i_beta = i_alphabeta.beta;
    improvedFluxObserver_estimate(&improved_observer);

    // 编码器反馈（调试对比用）
    float angle_encoder = wrap_neg_pi_to_pi(encoder_get_pllAngle() - foc_improvedFlux_handle.angle_offset);
    float speed_encoder = encoder_get_pllSpeed();

    // 观测器输出作为控制反馈
    float angle_el = wrap_neg_pi_to_pi(improvedFluxObserver_get_angle(&improved_observer));
    float speed_feedback = improvedFluxObserver_get_speed(&improved_observer);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 速度闭环
    loopControl_run_speedLoop(&foc_improvedFlux_handle, i_dq, angle_el, speed_feedback, FOC_SPEED_LOOP_DIVIDER);

    // 将输出电压写回观测器供下一拍使用
    alphabeta_t u_alphabeta = ipark_transform(
        (dq_t){.d = foc_improvedFlux_handle.v_d_out, .q = foc_improvedFlux_handle.v_q_out}, angle_el);
    improved_observer.u_alpha = u_alphabeta.alpha;
    improved_observer.u_beta = u_alphabeta.beta;

    // 保存调试数据
    speed_encoder_temp = speed_encoder;
    speed_obs_temp = speed_feedback;
    angle_encoder_temp = angle_encoder;
    angle_obs_temp = angle_el;
    id_temp = i_dq.d;
    iq_temp = i_dq.q;
}

void improvedFluxObserverClosed_init(float speed_rpm)
{
    pid_init(&pid_id, PID_MODE_PI, IMPROVED_FLUX_OBSERVER_CURRENT_PID_KP, IMPROVED_FLUX_OBSERVER_CURRENT_PID_KI, 0.0f, -U_DC / 2.0f, U_DC / 2.0f, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, IMPROVED_FLUX_OBSERVER_CURRENT_PID_KP, IMPROVED_FLUX_OBSERVER_CURRENT_PID_KI, 0.0f, -U_DC / 2.0f, U_DC / 2.0f, PID_LIMIT_DISABLE);
    pid_init(&pid_speed, PID_MODE_PI, IMPROVED_FLUX_OBSERVER_SPEED_PID_KP, IMPROVED_FLUX_OBSERVER_SPEED_PID_KI, 0.0f, IMPROVED_FLUX_OBSERVER_SPEED_PID_OUT_MIN, IMPROVED_FLUX_OBSERVER_SPEED_PID_OUT_MAX, PID_LIMIT_ENABLE);

    foc_init(&foc_improvedFlux_handle, &pid_id, &pid_iq, &pid_speed);
    foc_set_id(&foc_improvedFlux_handle, 0.0f);
    foc_set_speed(&foc_improvedFlux_handle, speed_rpm);

    // zero_alignment(&foc_improvedFlux_handle);
    improvedFluxObserver_init(&improved_observer, &improved_observer_cfg);

    // 初始电压置零
    improved_observer.u_alpha = 0.0f;
    improved_observer.u_beta = 0.0f;

    adc_register_injectedCallback(improvedFlux_closed_callback);
}

void improvedFluxObserverClosedDebug_print_info(void)
{
    float angle_encoder_deg = wrap_0_2pi(angle_encoder_temp) * 57.2958f;
    float angle_obs_deg = wrap_0_2pi(angle_obs_temp) * 57.2958f;

    float data[6] = {speed_encoder_temp, speed_obs_temp, angle_encoder_deg, angle_obs_deg, id_temp, iq_temp};
    vofa_send(data, 6);
}