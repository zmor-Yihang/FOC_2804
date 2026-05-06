#include "pid.h"

void pid_init(pid_controller_t *pid, pid_mode_t mode, float kp, float ki, float kd, float out_min, float out_max, pid_limit_mode_t enable_limit)
{
    pid->mode = mode;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->out = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->enable_limit = enable_limit;

    // kt = ki，Microchip AN1078 笔记的取法
    pid->kt = ki;
    pid->backcalc_error = 0.0f;

    // 积分限幅要足够大，不要限制 back-calc 的收敛
    pid->integral_max = fmaxf(fabsf(out_min), fabsf(out_max));
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kt = ki;
}

void pid_set_output_limit(pid_controller_t *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_set_limit_enable(pid_controller_t *pid, pid_limit_mode_t enable_limit)
{
    pid->enable_limit = enable_limit;
}

void pid_set_integral_limit(pid_controller_t *pid, float integral_max)
{
    pid->integral_max = fabsf(integral_max);
}

void pid_set_mode(pid_controller_t *pid, pid_mode_t mode)
{
    pid->mode = mode;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float feedback, float dt)
{
    pid->error = setpoint - feedback; // 误差

    pid->p_term = pid->kp * pid->error; // 比例项
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;

    if (dt < 0.0f)
        dt = 0.0f;

    if ((pid->mode == PID_MODE_PI) || (pid->mode == PID_MODE_PID))
    {
        // back-calc 自动调节
        float integral_increment = (pid->kp * pid->ki * pid->error - pid->kt * pid->backcalc_error) * dt; // 该周期的积分项 + back-calc 修正

        pid->integral += integral_increment; // 积分项累加

        // 积分限幅，同时要足够大以允许 back-calc 收敛
        pid->integral = utils_clampf(pid->integral, -pid->integral_max, pid->integral_max);

        pid->i_term = pid->integral;
    }

    if (((pid->mode == PID_MODE_PD) || (pid->mode == PID_MODE_PID)) && (dt > 0.0f))
    {
        pid->derivative = (pid->error - pid->prev_error) / dt;
        pid->d_term = pid->kd * pid->derivative;
    }
    else
    {
        pid->derivative = 0.0f;
    }

    float out_unclamped = pid->p_term + pid->i_term + pid->d_term; // 未限幅的PID输出

    // 如果是电流环PI，直接输出未限幅的值，由外部负责限幅，因为电流环输出是电压，需要做矢量限幅
    if (pid->enable_limit == PID_LIMIT_DISABLE)
    {
        pid->out = out_unclamped;
        pid->backcalc_error = 0.0f;
        pid->prev_error = pid->error;
        return pid->out;
    }

    // 速度环输出限幅
    pid->out = utils_clampf(out_unclamped, pid->out_min, pid->out_max);

    pid->backcalc_error = out_unclamped - pid->out; // 饱和误差，用于下一周期的 back-calc 修正
    pid->prev_error = pid->error;

    return pid->out;
}

// PID复位
void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->out = 0.0f;
    pid->backcalc_error = 0.0f;
}
