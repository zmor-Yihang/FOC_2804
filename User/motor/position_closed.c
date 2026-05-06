#include "position_closed.h"

static foc_t foc_position_closed_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;
static pid_controller_t pid_position;

// 打印用
static float position_rad_temp = 0.0f;
static float target_position_rad_temp = 0.0f;
static float position_error_rad_temp = 0.0f;
static float speed_rpm_temp = 0.0f;
static float target_speed_temp = 0.0f;
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

static void position_closed_callback(void)
{
    // 更新编码器角度、速度和机械多圈位置
    encoder_update();

    // 控制使用PLL估计角度；编码器实测角度只用于调试观察
    float angle_el = encoder_get_pllAngle() - foc_position_closed_handle.angle_offset;
    float speed_feedback = encoder_get_pllSpeed();
    float position_feedback = encoder_get_mechanicalPosition();

    // 获取电流反馈值
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    // Clark 变换
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 位置闭环：位置PID直接输出iq目标，电流环输出电压
    loopControl_run_positionLoop(&foc_position_closed_handle, i_dq, angle_el, speed_feedback, position_feedback, FOC_POSITION_LOOP_DIVIDER);

    // 保存调试打印数据
    id_temp = i_dq.d;
    iq_temp = i_dq.q;
    ia_temp = i_abc.a;
    ib_temp = i_abc.b;
    ic_temp = i_abc.c;
    speed_rpm_temp = speed_feedback;
    position_rad_temp = position_feedback;
    target_position_rad_temp = foc_position_closed_handle.target_position;
    position_error_rad_temp = foc_position_closed_handle.target_position - position_feedback;
    pll_angle_el_temp = angle_el;
    target_speed_temp = foc_position_closed_handle.target_speed;
    v_d_pi_temp = foc_position_closed_handle.v_d_pi;
    v_q_pi_temp = foc_position_closed_handle.v_q_pi;
    v_d_ff_temp = foc_position_closed_handle.v_d_ff;
    v_q_ff_temp = foc_position_closed_handle.v_q_ff;
    v_d_out_temp = foc_position_closed_handle.v_d_out;
    v_q_out_temp = foc_position_closed_handle.v_q_out;
    v_mag_temp = sqrtf(v_d_out_temp * v_d_out_temp + v_q_out_temp * v_q_out_temp);
}

void positionClosed_init(float position_rad)
{
    // 初始化电流环和位置环 PID 控制器，位置环使用PID模式并直接输出iq目标
    pid_init(&pid_id, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);
    pid_init(&pid_speed, PID_MODE_PI, SPEED_PID_KP, SPEED_PID_KI, 0.0f, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, PID_LIMIT_ENABLE);
    pid_init(&pid_position, PID_MODE_PID, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD, POSITION_PID_OUT_MIN, POSITION_PID_OUT_MAX, PID_LIMIT_ENABLE);

    // 初始化 FOC 控制句柄
    foc_init(&foc_position_closed_handle, &pid_id, &pid_iq, &pid_speed);
    foc_set_positionPid(&foc_position_closed_handle, &pid_position);

    // 零点对齐
    zero_alignment(&foc_position_closed_handle);

    // 以当前位置作为机械多圈位置零点，避免上电后目标位置突跳
    encoder_update();
    encoder_reset_mechanicalPosition(0.0f);

    // 设置目标位置
    foc_set_id(&foc_position_closed_handle, 0.0f);
    foc_set_speed(&foc_position_closed_handle, 0.0f);
    foc_set_position(&foc_position_closed_handle, position_rad);

    // 注册回调函数
    adc_register_injectedCallback(position_closed_callback);
}

void positionClosedDebug_print_info(void)
{
    // 归一化角度到 [0, 2π) 范围
    // PLL估计电角度也归一化并转换到角度制
    float pll_angle_normalized = fmodf(pll_angle_el_temp, MATH_TWO_PI);
    if (pll_angle_normalized < 0.0f)
    {
        pll_angle_normalized += MATH_TWO_PI;
    }
    float pll_angle_deg = pll_angle_normalized * 57.2958f;

    float data[20] = {position_rad_temp, target_position_rad_temp, position_error_rad_temp,
                      speed_rpm_temp, target_speed_temp, pll_angle_deg,
                      id_temp, iq_temp, ia_temp, ib_temp, ic_temp,
                      v_d_pi_temp, v_q_pi_temp, v_d_ff_temp, v_q_ff_temp,
                      v_d_out_temp, v_q_out_temp, v_mag_temp};
    vofa_send(data, 18);
}
