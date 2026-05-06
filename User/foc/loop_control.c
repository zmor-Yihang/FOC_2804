#include <math.h>

#include "foc.h"
#include "gate_drive.h"
#include "../adv_alg/weaken_flux.h"

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 机械速度反馈 (RPM), 仅在启用前馈解耦时使用
 */
void loopControl_run_currentLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm)
{
    float v_d_pi;
    float v_q_pi;
    float v_d_ff;
    float v_q_ff;
    float v_d_unsat;
    float v_q_unsat;

    /* 电流环 PI */
    v_d_pi = pid_calculate(handle->pid_id, handle->target_id, i_dq.d, FOC_CURRENT_LOOP_DT_S);
    v_q_pi = pid_calculate(handle->pid_iq, handle->target_iq, i_dq.q, FOC_CURRENT_LOOP_DT_S);

#if (FOC_DECOUPLING_ENABLE == 1)
    /* 前馈解耦 */
    float omega_e = speed_rpm * (MATH_TWO_PI / 60.0f) * MOTOR_POLE_PAIRS; /* 电角速度 rad/s */

    v_d_ff = -omega_e * MOTOR_LQ_H * i_dq.q;
    v_q_ff = omega_e * (MOTOR_LD_H * i_dq.d + MOTOR_PSI_F);
#else
    /* 关闭前馈解耦 */
    (void)speed_rpm;
    v_d_ff = 0.0f;
    v_q_ff = 0.0f;
#endif /* FOC_DECOUPLING_ENABLE */

    v_d_unsat = v_d_pi + v_d_ff;
    v_q_unsat = v_q_pi + v_q_ff;

    handle->v_d_pi = v_d_pi;
    handle->v_q_pi = v_q_pi;
    handle->v_d_ff = v_d_ff;
    handle->v_q_ff = v_q_ff;
    handle->v_d_cmd = v_d_unsat;
    handle->v_q_cmd = v_q_unsat;
    handle->v_d_out = v_d_unsat;
    handle->v_q_out = v_q_unsat;

    /* dq 电压矢量联合限幅，参考 VESC 开源代码 */
    float v_limit = U_DC * FOC_VOLTAGE_LIMIT_SVPWM_SCALE;
    float v_mag_sq = handle->v_d_out * handle->v_d_out + handle->v_q_out * handle->v_q_out;
    float v_limit_sq = v_limit * v_limit;

    if (v_mag_sq > v_limit_sq)
    {
        float scale = v_limit / sqrtf(v_mag_sq);
        handle->v_d_out *= scale;
        handle->v_q_out *= scale;
    }

    handle->pid_id->backcalc_error = v_d_unsat - handle->v_d_out;
    handle->pid_iq->backcalc_error = v_q_unsat - handle->v_q_out;

    /* 逆 Park 变换后交由输出层完成调制与PWM下发 */
    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);
    handle->duty_cycle = gateDrive_set_voltage(v_alphabeta);
}

/**
 * @brief 速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 * @param speed_loop_divider 速度环相对电流环的分频系数
 */
void loopControl_run_speedLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider)
{
    static uint8_t speed_loop_div = 0;

    /* 速度环按分频执行，其他周期保持上一次 iq 目标 */
    if (++speed_loop_div >= speed_loop_divider)
    {
        speed_loop_div = 0;
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm, speed_loop_dt);
    }

    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}

/**
 * @brief 位置闭环运行，位置PID直接输出q轴目标电流，D项继续使用PLL机械速度避免目标阶跃冲击
 * @param handle                 FOC 控制句柄
 * @param i_dq                   dq 轴电流反馈
 * @param angle_el               电角度 (rad)
 * @param speed_rpm              速度反馈 (RPM)
 * @param position_rad           机械多圈位置反馈 (rad)
 * @param position_loop_divider  位置环相对电流环的分频系数
 */
void loopControl_run_positionLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, float position_rad, uint8_t position_loop_divider)
{
    static uint16_t position_loop_div = 0; // 位置环分频计数器

    /* 位置环按分频执行，未到执行周期时保持上一次 target_iq */
    if (++position_loop_div >= position_loop_divider)
    {
        pid_controller_t *pid = handle->pid_position;
        float position_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)position_loop_divider;
        float position_error;
        float position_speed_rad_s;
        float integral_increment;
        float out_unclamped;

        position_loop_div = 0; // 复位分频计数器
        position_error = handle->target_position - position_rad;
        position_speed_rad_s = speed_rpm * (MATH_TWO_PI / 60.0f); // PLL机械速度，单位 rad/s

        /* P项：由位置误差决定基础转矩请求 */
        pid->error = position_error;
        pid->p_term = pid->kp * pid->error;

        /* I项：累积位置误差，消除恒定负载下的稳态误差 */
        integral_increment = (pid->kp * pid->ki * pid->error - pid->kt * pid->backcalc_error) * position_loop_dt;
        pid->integral += integral_increment;
        pid->integral = utils_clampf(pid->integral, -pid->integral_max, pid->integral_max);
        pid->i_term = pid->integral;

        /* D项继续取负的反馈速度，保留原有阻尼特性并避免目标阶跃微分冲击 */
        pid->derivative = -position_speed_rad_s;
        pid->d_term = pid->kd * pid->derivative;

        out_unclamped = pid->p_term + pid->i_term + pid->d_term;
        pid->out = utils_clampf(out_unclamped, pid->out_min, pid->out_max);
        pid->backcalc_error = out_unclamped - pid->out;
        pid->prev_error = pid->error;

        /* 位置环直接给电流环 q 轴电流目标 */
        handle->target_iq = pid->out;
    }

    /* 位置控制当前只使用 q 轴转矩电流，d 轴目标保持为 0 */
    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}

/**
 * @brief 带弱磁的速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 * @param speed_loop_divider 速度环相对电流环的分频系数
 */
void loopControl_run_speedWeakLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider)
{
    static uint8_t speed_loop_div = 0;

    /* 速度环按分频执行，其他周期保持上一次 iq 目标 */
    if (++speed_loop_div >= speed_loop_divider)
    {
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
        speed_loop_div = 0;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm, speed_loop_dt);
    }

    if (handle->flux_weak != NULL)
    {
        /* 弱磁环目前使用限幅前电压请求值作为电压裕量反馈。 */
        handle->target_id = fluxWeak_calculate(handle->flux_weak, handle->v_d_cmd, handle->v_q_cmd, FOC_CURRENT_LOOP_DT_S);
    }
    else
    {
        handle->target_id = 0.0f;
    }

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}
