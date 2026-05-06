#include "flux_observer_closed.h"

// foc 句柄
static foc_t foc_fluxObserver_handle;

// PI 控制器
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

static fluxobserver_t flux_observer; // 观测器句柄

// 观测器配置结构体
static const fluxobserver_cfg_t flux_observer_cfg = {
    .rs = MOTOR_RS_Ω,
    .ls = 0.5f * (MOTOR_LD_H + MOTOR_LQ_H),
    .psi_m = MOTOR_PSI_F,
    .poles = MOTOR_POLE_PAIRS,
    .ts = FLUX_OBSERVER_TS_S,
    .gamma = FLUX_OBSERVER_GAMMA,
    .pll_kp = FLUX_OBSERVER_PLL_KP,
    .pll_ki = FLUX_OBSERVER_PLL_KI,
    .pll_speed_limit_rpm = FLUX_OBSERVER_PLL_SPEED_LIMIT_RPM,
};

// 打印用（速度、角度、磁链）
static float speed_rpm_temp = 0.0f;
static float speed_obs_rpm_temp = 0.0f;
static float angle_encoder_temp = 0.0f;
static float angle_observer_temp = 0.0f;
static float flux_angle_temp = 0.0f;
static float flux_linkage_temp = 0.0f;

static void fluxObserver_closed_callback(void)
{
    // 更新编码器状态
    encoder_update();

    // 获取电流反馈值
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    // Clark 变换
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // 更新观测器输入：电流与上一控制周期输出电压
    flux_observer.i_alpha = i_alphabeta.alpha;
    flux_observer.i_beta = i_alphabeta.beta;
    fluxObserver_estimate(&flux_observer);

    // 调试对比：保留编码器反馈
    float angle_encoder = wrap_pm_pi(encoder_get_pllAngle() - foc_fluxObserver_handle.angle_offset);
    float speed_encoder = encoder_get_pllSpeed();

    // 控制反馈角度与速度：改用观测器输出
    float angle_el = wrap_pm_pi(fluxObserver_get_angle(&flux_observer));
    float speed_feedback = fluxObserver_get_speed(&flux_observer);

    float angle_observer = angle_el;
    float speed_observer = speed_feedback;

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 速度闭环
    loopControl_run_speedLoop(&foc_fluxObserver_handle, i_dq, angle_el, speed_feedback, FOC_SPEED_LOOP_DIVIDER);

    // 将当前周期输出电压写回观测器，供下一周期估算使用
    alphabeta_t u_alphabeta = ipark_transform((dq_t){.d = foc_fluxObserver_handle.v_d_out, .q = foc_fluxObserver_handle.v_q_out}, angle_el);
    flux_observer.u_alpha = u_alphabeta.alpha;
    flux_observer.u_beta = u_alphabeta.beta;

    // 保存调试数据（速度、角度、磁链）
    speed_rpm_temp = speed_encoder;
    speed_obs_rpm_temp = speed_observer;
    angle_encoder_temp = angle_encoder;
    angle_observer_temp = angle_observer;
    flux_angle_temp = wrap_pm_pi(flux_observer.theta_est);
    flux_linkage_temp = sqrtf(flux_observer.xhat_alpha * flux_observer.xhat_alpha + flux_observer.xhat_beta * flux_observer.xhat_beta);
}

void fluxObseverClosed_init(float speed_rpm)
{
    // 初始化电流环与速度环 PID 控制器
    pid_init(&pid_id, PID_MODE_PI, FLUX_OBSERVER_CURRENT_PID_KP, FLUX_OBSERVER_CURRENT_PID_KI, 0.0f, -U_DC / 2.0f, U_DC / 2.0f, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, FLUX_OBSERVER_CURRENT_PID_KP, FLUX_OBSERVER_CURRENT_PID_KI, 0.0f, -U_DC / 2.0f, U_DC / 2.0f, PID_LIMIT_DISABLE);
    pid_init(&pid_speed, PID_MODE_PI, FLUX_OBSERVER_SPEED_PID_KP, FLUX_OBSERVER_SPEED_PID_KI, 0.0f, FLUX_OBSERVER_SPEED_PID_OUT_MIN, FLUX_OBSERVER_SPEED_PID_OUT_MAX, PID_LIMIT_ENABLE);

    // 初始化 FOC 控制句柄
    foc_init(&foc_fluxObserver_handle, &pid_id, &pid_iq, &pid_speed);

    // 设置目标值
    foc_set_id(&foc_fluxObserver_handle, 0.0f);
    foc_set_speed(&foc_fluxObserver_handle, speed_rpm);

    // 零点对齐
    // zero_alignment(&foc_fluxObserver_handle);

    // 初始化磁链观测器
    fluxObserver_init(&flux_observer, &flux_observer_cfg);

    // 初始电压置零
    flux_observer.u_alpha = 0.0f;
    flux_observer.u_beta = 0.0f;

    // 注册回调函数
    adc_register_injectedCallback(fluxObserver_closed_callback);
}

void fluxObseverClosedDebug_print_info(void)
{
    // 角度统一转换为角度制
    float angle_encoder_deg = wrap_0_2pi(angle_encoder_temp) * 57.2958f;
    float angle_observer_deg = wrap_0_2pi(angle_observer_temp) * 57.2958f;
    float flux_angle_deg = wrap_0_2pi(flux_angle_temp) * 57.2958f;

    float data[6] = {speed_rpm_temp, speed_obs_rpm_temp, angle_encoder_deg, angle_observer_deg, flux_angle_deg, flux_linkage_temp};
    vofa_send(data, 6);
}
