#ifndef __PID__H__ 
#define __PID__H__

#include "math.h"
#include "stdint.h"
#include "../utils/math_utils.h"

typedef enum
{
    PID_MODE_P = 0,
    PID_MODE_PI,
    PID_MODE_PD,
    PID_MODE_PID
} pid_mode_t;

typedef enum
{
    PID_LIMIT_DISABLE = 0,
    PID_LIMIT_ENABLE
} pid_limit_mode_t;

/* P/PI/PD/PID控制器结构体 */
typedef struct
{
    pid_mode_t mode;    /* 控制律模式 */
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */

    float error;        /* 当前误差 */
    float prev_error;   /* 上一拍误差 */
    float integral;     /* 积分项累加 */
    float derivative;   /* 微分项 */

    float p_term;       /* 比例项 */
    float i_term;       /* 积分项 */
    float d_term;       /* 微分项输出 */

    float out;           /* PID输出 */
    float out_min;       /* 输出下限 */
    float out_max;       /* 输出上限 */
    float integral_max;  /* 积分限幅 */
    pid_limit_mode_t enable_limit;/* 输出限幅使能 */

    float kt;            /* Back-calculation 增益 */
    float backcalc_error;/* 上一拍饱和误差 */
} pid_controller_t;

/* P/PI/PD/PID控制器初始化 */
void pid_init(pid_controller_t *pid, pid_mode_t mode, float kp, float ki, float kd, float out_min, float out_max, pid_limit_mode_t enable_limit);

/* 设置PID参数 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/* 设置输出限幅 */
void pid_set_output_limit(pid_controller_t *pid, float out_min, float out_max);

/* 设置输出限幅使能 */
void pid_set_limit_enable(pid_controller_t *pid, pid_limit_mode_t enable_limit);

/* 设置积分限幅 */
void pid_set_integral_limit(pid_controller_t *pid, float integral_max);

/* 设置控制律模式 */
void pid_set_mode(pid_controller_t *pid, pid_mode_t mode);

/* P/PI/PD/PID计算，dt单位：秒 */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback, float dt);

/* PID复位 */
void pid_reset(pid_controller_t *pid);

#endif /* __PID__H__ */
