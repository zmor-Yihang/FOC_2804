#include "speed_closed.h"

static foc_t foc_speed_closed_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

static pid_controller_t pid_speed;

static luenberger_dob_t luenberger_dob;
static uint8_t speed_loop_div = 0;

// 打印用
static float speed_rpm_temp = 0.0f;
static float pll_angle_el_temp = 0.0f;
static float id_temp = 0.0f;
static float iq_temp = 0.0f;
static float ia_temp = 0.0f;
static float ib_temp = 0.0f;
static float ic_temp = 0.0f;
static float target_iq_temp = 0.0f;
static float target_id_temp = 0.0f;
static float v_d_pi_temp = 0.0f;
static float v_q_pi_temp = 0.0f;
static float v_d_ff_temp = 0.0f;
static float v_q_ff_temp = 0.0f;
static float v_d_out_temp = 0.0f;
static float v_q_out_temp = 0.0f;
static float v_mag_temp = 0.0f;
static float speed_loop_iq_temp = 0.0f;
static float cogging_comp_iq_temp = 0.0f;
static float dob_iq_comp_temp = 0.0f;
static float dob_d_hat_temp = 0.0f;
static float dob_omega_hat_temp = 0.0f;

static void speed_closed_callback(void)
{
    // 更新速度
    encoder_update();

    // 控制使用PLL估计角度；编码器实测角度只用于调试观察
    float angle_el = encoder_get_pllAngle() - foc_speed_closed_handle.angle_offset;
    float speed_feedback = encoder_get_pllSpeed();

    // 获取电流反馈值
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    // Clark 变换
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 速度闭环
    if (++speed_loop_div >= FOC_SPEED_LOOP_DIVIDER)
    {
        speed_loop_div = 0;
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)FOC_SPEED_LOOP_DIVIDER;
        float omega_mech_rad_s = speed_feedback * (MATH_TWO_PI / 60.0f);
        speed_loop_iq_temp = pid_calculate(foc_speed_closed_handle.pid_speed, foc_speed_closed_handle.target_speed, speed_feedback, speed_loop_dt);

    // 龙伯格扰动观测器
#if (LUENBERGER_DOB_ENABLE == 1)
        dob_iq_comp_temp = luenbergerDOB_update(&luenberger_dob, omega_mech_rad_s, i_dq.q);
        dob_d_hat_temp = luenbergerDOB_get_d_hat(&luenberger_dob);
        dob_omega_hat_temp = luenbergerDOB_get_omega_hat(&luenberger_dob);
#else
        dob_iq_comp_temp = 0.0f;
        dob_d_hat_temp = 0.0f;
        dob_omega_hat_temp = omega_mech_rad_s;
#endif /* LUENBERGER_DOB_ENABLE */
    }

    foc_speed_closed_handle.target_id = 0.0f;

    /* 齿槽转矩补偿：根据编码器原始计数插值出对应的 q 轴补偿电流 */
#if (COGGING_COMP_ENABLE == 1U)
    uint16_t raw_count = encoder_get_rawCount();
    cogging_comp_iq_temp = coggingComp_getIqByRawCount(raw_count);
#else
    cogging_comp_iq_temp = 0.0f;
#endif /* COGGING_COMP_ENABLE */
    foc_speed_closed_handle.target_iq = speed_loop_iq_temp + cogging_comp_iq_temp + dob_iq_comp_temp;

    if (foc_speed_closed_handle.target_iq > SPEED_PID_OUT_MAX)
    {
        foc_speed_closed_handle.target_iq = SPEED_PID_OUT_MAX;
    }
    else if (foc_speed_closed_handle.target_iq < SPEED_PID_OUT_MIN)
    {
        foc_speed_closed_handle.target_iq = SPEED_PID_OUT_MIN;
    }

    // 复用电流闭环
    loopControl_run_currentLoop(&foc_speed_closed_handle, i_dq, angle_el, speed_feedback);

    // 保存电流值用于打印
    id_temp = i_dq.d;
    iq_temp = i_dq.q;
    ia_temp = i_abc.a;
    ib_temp = i_abc.b;
    ic_temp = i_abc.c;
    speed_rpm_temp = speed_feedback;
    pll_angle_el_temp = angle_el;
    target_iq_temp = foc_speed_closed_handle.target_iq;
    target_id_temp = foc_speed_closed_handle.target_id;
    v_d_pi_temp = foc_speed_closed_handle.v_d_pi;
    v_q_pi_temp = foc_speed_closed_handle.v_q_pi;
    v_d_ff_temp = foc_speed_closed_handle.v_d_ff;
    v_q_ff_temp = foc_speed_closed_handle.v_q_ff;
    v_d_out_temp = foc_speed_closed_handle.v_d_out;
    v_q_out_temp = foc_speed_closed_handle.v_q_out;
    v_mag_temp = sqrtf(v_d_out_temp * v_d_out_temp + v_q_out_temp * v_q_out_temp);
}

void speedClosed_init(float speed_rpm)
{
    // 初始化速度环 PID 控制器
    pid_init(&pid_id, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE); // 按电流环带宽1000Hz整定

    pid_init(&pid_speed, PID_MODE_PI, SPEED_PID_KP, SPEED_PID_KI, 0.0f, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, PID_LIMIT_ENABLE); // 按 δ = 16 整定的

    // 初始化 FOC 控制句柄
    foc_init(&foc_speed_closed_handle, &pid_id, &pid_iq, &pid_speed);

    // 设置目标速度
    foc_set_id(&foc_speed_closed_handle, 0.0f);
    foc_set_speed(&foc_speed_closed_handle, speed_rpm);
    speed_loop_div = 0U;
    speed_loop_iq_temp = 0.0f;
    cogging_comp_iq_temp = 0.0f;
    dob_iq_comp_temp = 0.0f;
    dob_d_hat_temp = 0.0f;
    dob_omega_hat_temp = 0.0f;

    luenbergerDOB_init(&luenberger_dob,
                       LUENBERGER_DOB_TS_S,
                       LUENBERGER_DOB_J_KGM2,
                       LUENBERGER_DOB_B_NM_S_RAD,
                       LUENBERGER_DOB_KT_NM_A,
                       LUENBERGER_DOB_BANDWIDTH_HZ,
                       LUENBERGER_DOB_ZETA,
                       LUENBERGER_DOB_IQ_COMP_MAX,
                       LUENBERGER_DOB_COMP_GAIN);

    // 零点对齐
    zero_alignment(&foc_speed_closed_handle);

    float omega_mech_rad_s = encoder_get_pllSpeed() * (MATH_TWO_PI / 60.0f);
    luenbergerDOB_reset(&luenberger_dob, omega_mech_rad_s);
    dob_omega_hat_temp = luenbergerDOB_get_omega_hat(&luenberger_dob);

    // 注册回调函数
    adc_register_injectedCallback(speed_closed_callback);
}

void speedClosedDebug_print_info(void)
{
    // PLL估计电角度也归一化并转换到角度制
    float pll_angle_normalized = fmodf(pll_angle_el_temp, MATH_TWO_PI);
    if (pll_angle_normalized < 0.0f)
    {
        pll_angle_normalized += MATH_TWO_PI;
    }
    float pll_angle_deg = pll_angle_normalized * 57.2958f;

    float data[19] = {speed_rpm_temp, pll_angle_deg, id_temp, iq_temp, ia_temp, ib_temp, ic_temp,
                      target_iq_temp, target_id_temp, v_d_pi_temp, v_q_pi_temp, v_d_ff_temp, v_q_ff_temp, v_d_out_temp, v_q_out_temp, v_mag_temp,
                      dob_iq_comp_temp, dob_d_hat_temp, dob_omega_hat_temp};
    vofa_send(data, 19);
}