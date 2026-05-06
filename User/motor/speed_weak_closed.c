#include "speed_weak_closed.h"
#include "../adv_alg/weaken_flux.h"
#include "../bsp/i2c.h"

static foc_t foc_speed_weak_closed_handle;
static flux_weak_t flux_weak_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

static pid_controller_t pid_speed;

// 打印用
static float speed_rpm_temp = 0.0f;
static float pll_angle_el_temp = 0.0f;
static float id_temp = 0.0f;
static float iq_temp = 0.0f;
static float ia_temp = 0.0f;
static float ib_temp = 0.0f;
static float ic_temp = 0.0f;
static float v_d_pi_temp = 0.0f;
static float v_q_pi_temp = 0.0f;
static float v_d_ff_temp = 0.0f;
static float v_q_ff_temp = 0.0f;
static float v_d_out_temp = 0.0f;
static float v_q_out_temp = 0.0f;
static float v_mag_temp = 0.0f;
static float i2c_read_state_temp = 0.0f;

static void speed_weak_closed_callback(void)
{
    // 更新速度
    encoder_update();

    // 控制使用PLL估计角度；编码器实测角度只用于调试观察
    float angle_el = encoder_get_pllAngle() - foc_speed_weak_closed_handle.angle_offset;
    float speed_feedback = encoder_get_pllSpeed();

    // 获取电流反馈值
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    // Clark 变换
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 保存电流值用于打印
    id_temp = i_dq.d;
    iq_temp = i_dq.q;
    ia_temp = i_abc.a;
    ib_temp = i_abc.b;
    ic_temp = i_abc.c;
    speed_rpm_temp = speed_feedback;
    pll_angle_el_temp = angle_el;

    // 带弱磁速度闭环
    loopControl_run_speedWeakLoop(&foc_speed_weak_closed_handle, i_dq, angle_el, speed_feedback, FOC_SPEED_LOOP_DIVIDER);

    v_d_pi_temp = foc_speed_weak_closed_handle.v_d_pi;
    v_q_pi_temp = foc_speed_weak_closed_handle.v_q_pi;
    v_d_ff_temp = foc_speed_weak_closed_handle.v_d_ff;
    v_q_ff_temp = foc_speed_weak_closed_handle.v_q_ff;
    v_d_out_temp = foc_speed_weak_closed_handle.v_d_out;
    v_q_out_temp = foc_speed_weak_closed_handle.v_q_out;
    v_mag_temp = sqrtf(v_d_out_temp * v_d_out_temp + v_q_out_temp * v_q_out_temp);
    i2c_read_state_temp = (float)i2c_get_readState();
}

void speedWeakClosed_init(float speed_rpm)
{
    // 初始化速度环 PID 控制器
    pid_init(&pid_id, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE); // 按电流环带宽1000Hz整定

    pid_init(&pid_speed, PID_MODE_PI, SPEED_PID_KP, SPEED_PID_KI, 0.0f, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, PID_LIMIT_ENABLE); // 按 δ = 16 整定的

    // 初始化 FOC 控制句柄
    foc_init(&foc_speed_weak_closed_handle, &pid_id, &pid_iq, &pid_speed);

    // 初始化弱磁控制器
    fluxWeaken_init(&flux_weak_handle, U_DC * FOC_VOLTAGE_LIMIT_SVPWM_SCALE, FLUX_WEAK_U_REF_RATIO, FLUX_WEAK_KP, FLUX_WEAK_KI, FLUX_WEAK_ID_MIN);
    foc_set_fluxWeak(&foc_speed_weak_closed_handle, &flux_weak_handle);

    // 设置目标速度
    foc_set_id(&foc_speed_weak_closed_handle, 0.0f);
    foc_set_speed(&foc_speed_weak_closed_handle, speed_rpm);

    // 零点对齐
    zero_alignment(&foc_speed_weak_closed_handle);

    // 注册回调函数
    adc_register_injectedCallback(speed_weak_closed_callback);
}

void speedWeakClosedDebug_print_info(void)
{
    // PLL估计电角度也归一化并转换到角度制
    float pll_angle_normalized = fmodf(pll_angle_el_temp, MATH_TWO_PI);
    if (pll_angle_normalized < 0.0f)
    {
        pll_angle_normalized += MATH_TWO_PI;
    }
    float pll_angle_deg = pll_angle_normalized * 57.2958f;

    float data[15] = {speed_rpm_temp, pll_angle_deg, id_temp, iq_temp, ia_temp, ib_temp, ic_temp,
                      v_d_pi_temp, v_q_pi_temp, v_d_ff_temp, v_q_ff_temp, v_d_out_temp, v_q_out_temp, v_mag_temp, i2c_read_state_temp};
    vofa_send(data, 15);
}
