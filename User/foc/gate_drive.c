#include "gate_drive.h"

/**
 * @brief 将电压矢量转换为 SVPWM 占空比并下发到定时器
 * @param voltage_alphabeta 2D 电压矢量 (alpha-beta 坐标系)
 * @return 三相占空比
 */
abc_t gateDrive_set_voltage(alphabeta_t voltage_alphabeta) {
    // 电压矢量 -> SVPWM 占空比
    abc_t duty_cycle = svpwm_update(voltage_alphabeta);

    // 将占空比下发到三相 PWM
    tim_set_pwmDuty(duty_cycle.a, duty_cycle.b, duty_cycle.c);

    return duty_cycle;
}

/**
 * @brief 停止输出
 * @return 三相占空比
 */
abc_t gateDrive_stop(void) {
    // 50% 占空比使三相桥臂回到中点，避免继续施加电压矢量
    abc_t duty_cycle = {.a = 0.5f, .b = 0.5f, .c = 0.5f};

    tim_set_pwmDuty(duty_cycle.a, duty_cycle.b, duty_cycle.c);

    return duty_cycle;
}
