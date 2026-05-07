#include "speed_closed.h"

static foc_t foc_speed_closed_handle;

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
    static uint8_t speed_loop_div = 0;
    if (++speed_loop_div >= FOC_SPEED_LOOP_DIVIDER)
    {
        speed_loop_div = 0;
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)FOC_SPEED_LOOP_DIVIDER;
        speed_loop_iq_temp = pid_calculate(foc_speed_closed_handle.pid_speed, foc_speed_closed_handle.target_speed, speed_feedback, speed_loop_dt);
    }

    foc_speed_closed_handle.target_id = 0.0f;

    /* 齿槽转矩补偿：根据编码器原始计数插值出对应的 q 轴补偿电流 */
    uint16_t raw_count = encoder_get_rawCount();
    cogging_comp_iq_temp = coggingComp_getIqByRawCount(raw_count);
    foc_speed_closed_handle.target_iq = speed_loop_iq_temp + cogging_comp_iq_temp;

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
    speed_loop_iq_temp = 0.0f;
    cogging_comp_iq_temp = 0.0f;

    // 零点对齐
    zero_alignment(&foc_speed_closed_handle);

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

    float data[16] = {speed_rpm_temp, pll_angle_deg, id_temp, iq_temp, ia_temp, ib_temp, ic_temp,
                      target_iq_temp, target_id_temp, v_d_pi_temp, v_q_pi_temp, v_d_ff_temp, v_q_ff_temp, v_d_out_temp, v_q_out_temp, v_mag_temp};
    vofa_send(data, 16);
}